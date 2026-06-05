# convobs-rs — implementation reference

Companion to `PLAN.md`. This is a **reference for the algorithms, constants, and
gotchas** behind the conversions — **not a port spec**. The Go source
(`../satpulse`) is where to read the reference algorithm and the exact constants;
the **man page** (`docs/convobs.1.md`) is the behaviour spec; output is validated
**semantically by diffobs**, so it need not match Go's bytes. Build idiomatic
Rust against these notes, not a transliteration.

The one place "match Go exactly" genuinely applies is the **arithmetic that
produces obsj float values**: obsj is compared at exact f64, so a converter must
compute the *same f64 value* Go does — same operation order, constants, and
`f32` narrowings. That's about the computed number, not the serialized bytes.

## Where the algorithms live in the Go reference

Repo root: `../satpulse`.

**Pipeline / CLI**
- `internal/convobscmd/convobs.go` — flags, week inference, packet-log loop, raw
  auto-detect + metadata buffering, orchestration. (Behaviour reference; we do
  *not* reproduce its exact error strings.)
- `internal/convobscmd/convobs_test.go` — behaviour & golden tests.

**Model + I/O**
- `gps/lib/rinex/obs.go` — `SignalObservation`, `SignalValues`, `Metadata`,
  `Time`, `Sink`, `SatelliteID`/`SignalID`, `MergeMetadata`: the observation model.
- `gps/lib/opt/opt.go` — `opt.Val[T]` = Rust `Option`, omitted-when-unset in JSON.
- `gps/lib/rinex/write.go`, `read.go` — fixed-column RINEX header/field rules and
  obsj semantics (the DIY `rinexobs` backend follows these: arc↔LLI, CN0-from-SSI,
  time-system→GPS).
- `gps/lib/rinex/decimate.go`, `requirecp.go` — the two stream filters.
- `gps/lib/rinex/freq.go` — signal→frequency table (incl. GLONASS FDMA math).
- `gps/lib/rinex/diff.go`, `diffobs/main.go` — the comparator. (Ours is simpler:
  arc↔ll is centralized in the obsj crate — see PLAN.md.)

**RTCM**
- `gps/lib/rnxrtcm/rtcm.go` — the converter algorithm (~350 lines: cell math,
  slip detection, week resolution, metadata extraction).
- `gps/lib/rtcmbin/rinex.go` — RTCM→RINEX sat/signal mapping tables.
- `gps/lib/rtcmbin/mt.go`, `msmconv.go` — MSM field layout (to map `rtcm-rs`'s
  fields onto the converter's inputs).
- `gps/app/gpsio/log.go` — `PacketLogEntry` JSONL schema (CLI packet-log input).

**UBX**
- `gps/lib/rnxubx/ubx.go` — converter; `gps/lib/ubxbin/rxm.go` — RxmRawx fields;
  `gps/lib/ubxbin/rinex.go` — UBX→RINEX mapping tables.

## Crate reference (verify versions at impl time)

- **`rtcm-rs` 0.11** (MIT/Apache, `no_std`, dormant — vendor if a gap appears).
  Framing + CRC-24Q via `MessageFrame`/`MsgFrameIter`. Exposes RAW: DF407 lock
  time (`u16`), DF420 half-cycle, DF397 rough range int (`u8`), DF399 rough
  phase-rate (`i16`), DF419 GLONASS channel. **Caveat:** the *fine* per-cell
  fields DF405/406/404 and DF398/DF408 come back **pre-scaled to `f64`** (×2⁻²⁹
  etc.), not raw ints — recover the raw int (multiply back by the exact
  power-of-two) so the converter reproduces Go's exact f64. Per cell you get
  `(satellite_id, signal_id)` (signal id is a per-constellation enum) — map to
  the RINEX 2-char code. Metadata 1005/1006/1007/1008/1013/1033/1230 decoded.
- **`rinex` 0.22** (MPL-2.0; keep features scoped — `nav` pulls nalgebra/anise,
  avoid). Model: `BTreeMap<ObsKey{epoch,flag}, Observations{clock,
  signals: Vec<SignalObservation{sv, observable, value: f64, lli, snr}>>}`.
  **Key limitation:** `value` is a mandatory `f64`, so it **cannot represent a
  missing carrier phase that carries only a loss-of-lock flag** (a blank phase
  field on a pseudorange-only signal): the reader invents `cp = 1.0`, the writer
  can't emit the blank. This is *the* reason for two RINEX backends (PLAN.md) —
  the DIY `rinexobs` (optional value, faithful) and the `rinex-obsj` bridge
  (full-featured, inherits the limitation; `diffobs --ignore-blank-phase` covers
  it in tests). Otherwise trustworthy; validated at 5e-4. Its header formatting
  (`M (MIXED)`, omitted lines) is fine — validation is semantic.
- **`ublox` 0.10** (MIT): `trk_stat` flags (PR_VALID/CP_VALID/HALF_CYCLE/
  SUB_HALF_CYC), `cp_stdev` (`& 0x0F` slip nibble), `lock_time()`/`cno()`/
  `freq_id()`, framing via `Parser::consume_ubx`. **Caveat:** `sigId` is reachable
  but mislabeled `reserved2()` (offset-22 byte) — verify per message version.
- **Unicore:** no Rust crate; out of scope (not needed for the goal).

## Load-bearing details / gotchas

- **Time:** i64 ticks of 100 ns since GPS epoch 1980-01-06, **no leap adjustment**.
  obsj `t` = ISO 8601 zoneless `YYYY-MM-DDTHH:MM:SS.fffffff` (7 frac digits) —
  lossless, compares exactly. RINEX read converts file time-system→GPS (GLO/UTC
  add leap seconds, BDT +14 s).
- **obsj record:** observation fields `t, sat, sig, frq, pr, cp, do, cn0, arc, hc,
  bt` (each omitted when absent/zero); metadata record has no `t`, nested objects
  (`run`/`marker`/`receiver`/`antenna`); dates are ISO 8601 zoneless. NaN/Inf must
  error. Reader rejects legacy keys `ssi`/`lli`/`ll`.
- **arc / ll / LLI:** loss of lock is canonical as `arc` (monotonic per
  `(sat,sig)`). RINEX LLI bit0 ⇔ arc changed vs the previous *kept* epoch, bit1 ⇔
  `hc`, bit2 ⇔ `bt`. **In our design `ll ↔ arc` is centralized in the obsj crate**
  (converters emit `ll`; the crate accumulates `arc`); the Go per-converter arc
  maps are reference for *slip detection only*. CN0-from-SSI on RINEX read when no
  `S`: `ssi≤1→6, ≥9→57, else ssi*6+3`.
- **Decimation:** interval ≥1 s, multiple of 100 ns, divides 24 h. Keep rule:
  round `t` to nearest 100 ms grid, keep iff `rounded % interval == 0`. Metadata
  and flush pass through. (arc makes this correct across dropped gaps.)
- **requireCP (`--ppp-ar`):** drop observations with no `cp`.
- **RTCM converter math:** `rangeMS = c*0.001`, `p2_10/29/31`; doppler
  `prr = SatPhaseRate + SigPhaseRate*0.0001`, sign-flipped iff `--rtcm-strict-prr`,
  `d = (f32)(prr*freq/c)` — **the `f32` narrowing is load-bearing for exact-f64
  obsj**; `cn0 = CNR*0.0625`; `--rtcm-omit-zero-do` drops numeric-zero Doppler.
  GLONASS channel = `ExtInfo − 7`. Slip from DF407 lock-time (decrease ⇒ slip;
  defer the slip flag if no phase this epoch). **Week resolution** (trickiest):
  resolve GPS week from a `TimeInterval` constraint or epoch continuity; GLONASS
  epoch day==7 ⇒ 7 candidate offsets; BeiDou +14000 ms. See
  `resolveWeek`/`epochWeekOffsets`.
- **Doppler default sign is receiver-polarity** (`--rtcm-strict-prr` off by default).
- **Raw-mode metadata buffering:** RTCM metadata (1005 etc.) buffered until an MSM7
  commits the RTCM family; dropped if a non-RTCM family commits first. Mixed
  observation families ⇒ warn, keep the first.
- **Edge cases** (briefings in `tmp/rinex/*.md`): BDS B2b ⇒ label `7D` (not `7P`);
  BDS B3I `C6I` is emitted (RTKLIB drops it — golden whitelist); QZSS L2C-M ⇒ `2S`;
  UBX `subHalfCyc`/`halfCyc` arc/HC logic; Galileo E1C/E5Q CP fractional
  consistency UBX-vs-RTCM.

## diffobs

- Lives in the **obsj crate**; the `diffobs` CLI is a thin wrapper. Each input's
  format is set by an explicit option (never inferred from the extension);
  compression may be detected from content. Reads obsj and RINEX (whichever
  backend is built) into the obsj model.
- Align epochs by exact `Time`; align keys by `(sat,sig)`. Tolerances: **0 (exact
  f64) for obsj, 5e-4 for RINEX**; ApproxPos/AntennaDelta 5e-5; `frq`/`hc`/`bt`
  exact.
- Loss of lock compares the **`ll` transition** (via the centralized transform),
  not absolute `arc`. Metadata diff ignores `Run`/`Comment` (and Marker for RTCM
  goldens). **`--ignore-blank-phase`** skips `cp`/`ll` when one side has no carrier
  phase (covers the rinex-crate path).
- Output: one JSON object per difference; exit 0=identical, 1=differences, 2=error.

## Fixtures & oracles

- **Committed fixtures** (`cli/testdata/`, ~11 MB):
  `m8t/f9t-20251217.{ubx,obs.gz}`, `packet-rtcm-20260519.{rtcm,obs.gz}`,
  `um980-rtcm-20260527.{rtcm,obs.gz}` — four input/golden pairs that back the
  in-process golden test in `cli/tests/golden.rs` (`cargo test`, no Go and no
  convbin at test time). Goldens = RTKLIB-EX `convbin`; exact convbin flags in
  `cli/testdata/Makefile`; convobs-side flags + tolerances in `golden.rs`. (The
  Unicore `um980-uncb` pair and its `C6I`/`C7D`/`C7P` signal whitelist are out
  of scope here — convobs-rs has no Unicore input.)
- **Large fixtures** (`tmp/`, gitignored): maasdam/serpa/x20p/ttyAMA0 packet logs
  + `.obs.gz`/`.obsj`, `packet-rtcm-20260519-3h` — big-scale validation and the
  `--interval 30` perf gate, exercised both as packet logs and as
  `satpulsetool pack`-generated raw streams. `tmp/rinex/rinex.toml` is an example
  `--header-file`.
- **Go as test oracle:** build `satpulsetool` and run `convobs` side-by-side to
  produce reference output for any input, then compare with diffobs (exact-f64
  obsj, 5e-4 RINEX). This is how we leverage Go — as an oracle, not a template.

## CLI

Flags come from the man page (`docs/convobs.1.md`). `convobs.go` is the reference
for the *validation logic* (mutually-exclusive week options, the "valid only
with …" gating, decimation-interval checks), but we write natural Rust error
messages — **matching Go's exact error strings is not a goal**.

## Protocol specs (local)

`../gps-protocol-docs/`: `igs/rinex_4.02.md`, `rtcm/RTCM_SC-104_v3.2.md` (3.2,
slightly behind 3.4), `u-blox/F9-HPG-1.51.md` (+M8/X20).

# convobs-rs — porting reference

Companion to `PLAN.md`. Captures the source map, crate caveats, and load-bearing
details discovered during investigation, so the implementation session doesn't have
to rediscover them. The Go source is the ground truth — read the listed files;
this doc is pointers + the things that are subtle or *not* in the Go source.

## Go source map (what to port, by stage)

Repo root: `../satpulse`. Man page (behaviour spec):
`../satpulse/docs/man/satpulsetool-convobs.1.md`.

**Command / CLI / pipeline**
- `internal/convobscmd/convobs.go` — flags, error strings, week inference, packet-log
  loop, raw auto-detect + metadata buffering, the whole orchestration.
- `internal/convobscmd/convobs_test.go` — behaviour & golden tests (mirror these).

**Midpoint + I/O (Stage 1a)**
- `gps/lib/rinex/obs.go` — `SignalObservation`, `SignalValues`, `Metadata`, `Time`,
  `Sink`, `SatelliteID`/`SignalID`, `MergeMetadata`. The model to port.
- `gps/lib/opt/opt.go` — `opt.Val[T]` (= Rust `Option`, omitted-when-unset in JSON).
- `gps/lib/rinex/write.go` — RINEX + obsj writers + Sink impls (we port obsj; RINEX
  goes via the crate, but read this for exact RINEX semantics & header rules).
- `gps/lib/rinex/read.go` — RINEX + obsj readers (arc-from-LLI, CN0-from-SSI,
  time-system→GPS conversion).
- `gps/lib/rinex/decimate.go`, `requirecp.go` — the two stream filters.
- `gps/lib/rinex/freq.go` — signal→frequency table (incl. GLONASS FDMA math).
- `gps/lib/rinex/diff.go`, `diffobs/main.go`, `diffobs/main_test.go` — the comparator.

**RTCM (Stage 1b)**
- `gps/lib/rnxrtcm/rtcm.go` — the converter (port this; ~350 lines of real algorithm).
- `gps/lib/rtcmbin/rinex.go` — RTCM→RINEX sat/signal mapping tables (port verbatim).
- `gps/lib/rtcmbin/mt.go`, `msmconv.go` — MSM field layout the converter consumes
  (for understanding `rtcm-rs`'s equivalent fields).
- `gps/app/gpsio/log.go` — `PacketLogEntry` JSONL schema.
- `gps/gpsreg/reg.go`, `gps/scan/scan.go`, `gps/gpsprot/` — packet tags, framing,
  signal/GNSS types (only needed for `-r raw` stream demux).

**UBX (Stage 2)**
- `gps/lib/rnxubx/ubx.go` — converter; `gps/lib/ubxbin/rxm.go` — RxmRawx fields;
  `gps/lib/ubxbin/rinex.go` — UBX→RINEX mapping tables.

## Crate reference (NOT in Go source — verify versions at impl time)

- **`rtcm-rs` 0.11** (MIT/Apache, `no_std`, **dormant** — last release 2024-04, vendor
  if a gap appears). MSM7 (1077/1087/…): exposes **DF407 lock-time (`u16`) and DF420
  half-cycle (`u8`) RAW** — the critical fields. DF397 rough range int (`u8`), DF399
  rough phase-rate (`i16`), DF419 GLONASS channel (`i8`, bias −7) raw. **Caveat:** the
  *fine* per-cell fields DF405/406/404 and DF398/DF408 come back **pre-scaled to
  `f64`** (×2⁻²⁹ etc.), not raw ints — recover the raw int (multiply back, exact
  power-of-two) to replicate Go's exact arithmetic order for bit-identical obsj.
  Masks aren't exposed raw; instead you get decoded `satellite_id` per sat and
  `(satellite_id, signal_id: SigId)` per cell — map `SigId` → RINEX 2-char code to
  match Go's `rtcmbin/rinex.go` table. Metadata msgs 1005/1006/1007/1008/1013/1033/1230
  decoded. Framing + CRC-24Q via `MessageFrame` (`crc24lte_a` = RTCM poly).
- **`rinex` 0.22** (MPL-2.0, std-only; keep features scoped — `nav` pulls nalgebra/anise,
  avoid it). **Read** preserves LLI + SSI: `SignalObservation { sv, observable, value,
  lli: Option<LliFlags>, snr: Option<SNR> }`; record =
  `BTreeMap<ObsKey{epoch,flag}, Observations{clock, signals: Vec<…>}>`. **Write** is
  crate-controlled (`{:14.3}`, re-sorts SVs, omits `SYS / PHASE SHIFT`, possible LLI
  formatting quirk) — **not byte-exact**, which is why we validate RINEX semantically.
  *Verify early (Stage 1a gate):* all header metadata convobs needs is settable, and
  output LLI/SSI are semantically correct.
- **`ublox` 0.10** (MIT, `no_std`) for Stage 2: `cp_stdev_raw()` (`& 0x0F` for the slip
  nibble), `trk_stat_raw()` + `TrkStatFlags` (PR_VALID/CP_VALID/HALF_CYCLE/
  SUB_HALF_CYCLE all reachable), `lock_time()`/`cno()`/`freq_id()`, framing via
  `Parser::consume_ubx`. **Caveat:** `sigId` is reachable but mislabeled
  `reserved2()` (offset-22 byte) — verify against the message version.
- **Unicore:** no Rust crate exists (confirmed). Deferred; if ever needed, port from
  RTKLIB `src/rcv/novatel.c` (OEM7 framing `AA 44 12`, adaptable to Unicore OBSVM).

## Load-bearing details / gotchas (don't miss)

- **Time:** `i64` ticks of 100 ns since GPS epoch 1980-01-06, **no leap adjustment**.
  obsj `t` string = `YYYY-MM-DDTHH:MM:SS.fffffff` (exactly 7 frac digits, no TZ) —
  lossless ⇒ time compares exactly. RINEX read converts file time-system→GPS
  (GLO/UTC add leap seconds, BDT +14 s).
- **obsj wire format** (field order, all `omitzero`): observation =
  `t, sat, sig, frq, pr, cp, do, cn0, arc, hc, bt`; metadata record = no `t`, nested
  objects (`run`, `marker`, `receiver`, `antenna`). NaN/Inf must error (Go json does).
  Reader rejects legacy keys `ssi`/`lli`/`ll`.
- **arc ↔ LLI:** RINEX LLI **bit0** (loss of lock) ⇔ `arc` changed vs the previous
  *kept* epoch for that (sat,sig); **bit1** ⇔ `hc`; **bit2** ⇔ `bt`. On read, each bit0
  increments the per-(sat,sig) arc counter. **CN0-from-SSI** on read when no `S` obs:
  `ssi≤1→6, ssi≥9→57, else ssi*6+3`.
- **Decimation:** interval must be ≥1 s, a multiple of 100 ns, and divide 24 h exactly.
  Keep rule: round `t` to nearest **100 ms** grid, keep iff `rounded % interval == 0`.
  Passes metadata/flush through. (arc makes this correct across dropped gaps.)
- **requireCP (`--ppp-ar`):** drop observations with no `cp`.
- **RTCM converter:** constants `rangeMS=c*0.001`, `p2_10/29/31`; doppler
  `prr = SatPhaseRate + SigPhaseRate*0.0001`, sign-flipped iff `--rtcm-strict-prr`,
  `d = (f32)(prr*freq/c)` — **replicate the `f32` narrowing**; `cn0 = CNR*0.0625`;
  `--rtcm-omit-zero-do` drops numeric-zero Doppler. GLONASS channel = `ExtInfo−7`.
  Arc/slip from DF407 lock-time (decrease ⇒ slip; defer slip flag if no phase this
  epoch). **Week resolution** (the trickiest part): resolve GPS week from a
  `TimeInterval` constraint or epoch continuity; GLONASS epoch day==7 ⇒ 7 candidate
  week offsets; BeiDou +14000 ms. See `resolveWeek`/`epochWeekOffsets` in `rtcm.go`.
- **Raw-mode metadata buffering:** in `-r raw`, RTCM metadata (1005 etc.) is buffered
  until an MSM7 commits the RTCM family; dropped if a non-RTCM family (UBX/Unicore)
  is selected first. Mixed observation families ⇒ warn, keep first family.
- **Doppler default sign is receiver-polarity, not spec** (`--rtcm-strict-prr` off by
  default). Unicore CP = `−ADR` (Stage 3).
- **Edge cases** (briefings in `tmp/rinex/*.md`): BDS B2b ⇒ label **`7D`** (not `7P`);
  BDS B3I `C6I` is emitted (RTKLIB drops it — golden whitelist); QZSS L2C-M ⇒ `2S`
  (a gap to *implement*, see `um980-cross-test-findings.md`); UBX `subHalfCyc`/
  `halfCyc` arc/HC logic (`ubx-halfcycle-briefing.md`); Galileo E1C/E5Q CP fractional
  consistency UBX-vs-RTCM (`gal-iar-briefing.md`).

## `diffobs` spec (the comparator to port)

- Reads `.obs[.gz]` (rinex crate) and `.obsj` (our reader) into the midpoint.
- Align epochs by **exact `Time`** equality; align keys by `(sat, sig)`.
- Per-field tolerances: PR/CP/Do/CN0 default **5e-4** for RINEX; **0 (exact `f64`)**
  for obsj. ApproxPos/AntennaDelta 5e-5. `frq`/`hc`/`bt` exact bool/int.
- **`arc` is compared as a *relative transition*, never as an absolute number** (per
  side, did arc change at this epoch?); emit `ll:true` only when the two sides
  disagree about the transition. Metadata diff ignores `Run` and `Comment` (and
  Marker name/number for RTCM goldens).
- Output: one JSON object per difference; exit 0=identical, 1=differences, 2=error.

## Tests, fixtures & oracles

- **Committed small fixtures** (copy from `../satpulse/internal/convobscmd/testdata/`,
  ≤ ~4 MB ceiling): `packet-rtcm-20260519.{rtcm,obs.gz}`,
  `um980-rtcm-20260527.{rtcm,obs.gz}` (Stage 1b); `m8t/f9t-20251217.{ubx,obs.gz}`
  (Stage 2). Golden = RTKLIB-EX `convbin` (commit `89a735ba…`, "CONVBIN EX 2.5.0").
  Exact convbin flags per case are in `testdata/Makefile`; SatPulse-side flags and the
  signal whitelist (`C6I`, `C7D`, `C7P` for the um980 uncb case) are in
  `convobs_test.go` (`TestGoldenFiles`).
- **Large fixtures** (this repo's `tmp/`, gitignored): maasdam/serpa/x20p/ttyAMA0
  packet logs + their `.obs.gz`/`.obsj`, `packet-rtcm-20260519-3h.{rtcm,obs}` — for
  big-scale validation and the perf gate. `tmp/rinex/rinex.toml` is an example
  `--header-file`.
- **Side-by-side with Go:** build the reference with `cd ../satpulse && go build ...`
  (the `satpulsetool convobs` subcommand) to generate Go output for any input; that is
  the primary "identical-to-Go" oracle (exact obsj, semantic RINEX).

## CLI parity

Reproduce flags from the man page and the exact error strings in `convobs.go`
(matched by substring in tests), e.g. `"expected at least one input file"`,
`"--packet-log is valid only with packet input formats"`,
`"no RTCM MSM7 messages found"`,
`"RTCM input is older than one week; provide --date, --recent, or --date-from-filename"`,
the decimation-interval errors, and the per-option "valid only with …" messages.

## Protocol specs (local)

`../gps-protocol-docs/`: `igs/rinex_4.02.md`, `rtcm/RTCM_SC-104_v3.2.md` (3.2, slightly
behind 3.4), `u-blox/F9-HPG-1.51.md` (+M8/X20), `unicore/unicore1.13.md`.

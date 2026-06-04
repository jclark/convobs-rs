# convobs-rs — implementation plan

A Rust port of satpulse's `convobs` (`../satpulse/internal/convobscmd`). Goal: same
observation conversions, **semantically identical** output to the Go tool, and
substantially faster on large inputs (packet logs can be hundreds of MB).

Reference sources: `../satpulse` (Go), `../gps-protocol-docs` (RINEX 4.02, RTCM 3.2,
UBX, Unicore specs). **See `PORTING-NOTES.md`** for the Go-source→port map, crate
caveats, load-bearing algorithm details, the `diffobs` spec, fixtures/oracles, and
CLI parity — read it alongside this plan before implementing.

## Scope & staging

- **Stage 1a — I/O spine.** Midpoint model, Sink pipeline, obsj read/write, RINEX
  read/write via the `rinex` crate (adapters), metadata + `--header-file` (TOML),
  decimation, `--ppp-ar`, and the ported `diffobs` semantic comparator. End-to-end
  `obsj ↔ rinex`. Already a useful tool.
- **Stage 1b — RTCM.** `rtcm` input (packet stream + `--packet-log`), week inference,
  ported `rnxrtcm` converter + signal-map tables. This is the MVP:
  `rtcm/rinex/obsj` in, `rinex/obsj` out.
- **Stage 2 — UBX.** `ublox` crate + ported `rnxubx` converter.
- **Stage 3 — Unicore (deferred).** No Rust crate exists; would port from RTKLIB
  `novatel.c`. Out of scope unless requested.
- **Stage 4 — performance pass.** Profile-guided; float formatting, JSONL parsing,
  allocation. (Performance is a gate throughout, not only here.)

## Midpoint format (decided)

Go-faithful, per-signal, **carrying `arc`** (monotonic carrier-phase arc counter),
with compact `Copy` types and no per-record heap allocation:

```rust
struct SignalObservation { t: GpsTime, sat: SatId /*[u8;3]*/, sig: SigId /*[u8;2]*/, v: SignalValues }
struct SignalValues { // Copy
    frq: Option<i8>, pr: Option<f64>, cp: Option<f64>, do_: Option<f64>,
    cn0: Option<f32>, arc: u32, hc: bool, bt: bool,
}
trait Sink { fn metadata(&mut self, m: &Metadata); fn observation(&mut self, o: &SignalObservation); fn flush(&mut self); }
```

Rationale: `arc` is the better intermediate — it makes decimation both simple *and
correct* (a loss-of-lock inside a dropped gap still surfaces on the next kept epoch,
since `LLI = arc != previous_kept_arc`), and the converters compute it for free.
`arc ↔ LLI` is trivial (one compare + small-map lookup per obs) and happens once, at
the RINEX boundary only. obsj keeps `arc` directly.

## Pipeline (single pass into the sink)

Converter → inline `RequireCpFilter` → inline `DecimationSink` → output sink.
- **obsj sink:** streams one line per record, O(1) memory, no `arc→LLI`.
- **RINEX sink:** builds the `rinex` crate's epoch-keyed model directly, deriving LLI
  from arc transitions as it goes; formats on flush. No separate midpoint buffer
  beyond what RINEX inherently requires.
- Hot path statically dispatched (monomorphized filter→sink chain) to avoid
  per-observation virtual-call overhead.

## Code shape

Single binary crate (lib + thin `main`):
- `obs` — midpoint types, `GpsTime` (i64 ticks, ported to match Go), ported `freq` table.
- `sink` — `Sink` trait + obsj/RINEX sinks, decimation, requireCP, metadata buffer.
- `obsj` — serde read/write (our format).
- `rinexio` — read adapter (crate per-observable records → our bundles; arc from LLI
  transitions; CN0 from SSI) and write adapter (bundles → crate model; LLI from arc).
- `rtcm` — adapter over `rtcm-rs` + ported `rnxrtcm` (cell math, arc/slip, GLONASS
  week/day-7 resolution, metadata extraction, signal-map tables).
- `packetlog` — `PacketLogEntry` JSONL (hex `bin`/`ascii`, `t`, `tag`, `out`).
- `diffobs` — ported comparator over the midpoint; reads both `.obs[.gz]` and
  `.obsj`. Per-field configurable tolerances. Our comparison oracle for **both**
  formats: **RINEX** uses 5e-4 (forced by RINEX's 3-decimal text precision); **obsj**
  uses **exact `f64` equality** (default tolerance 0), since obsj I/O is lossless.
- `cli` — clap; reproduce flags and exact error strings (substring-compatible w/ Go).
- `ubx` (Stage 2); minimal `raw` demux for `-r raw` stream mode.

## Crates

`rtcm-rs` 0.11 (RTCM parse; exposes raw DF407/DF420/DF419, metadata msgs, CRC-24Q;
fine fields pre-scaled but losslessly recoverable; dormant — vendor if needed),
`rinex` 0.22 (RINEX I/O; reader preserves LLI/SSI), `ublox` 0.10 (Stage 2),
`serde`/`serde_json`, `toml`, `flate2`, `clap`, `thiserror`/`anyhow`.
GpsTime, diffobs, and all converters are ported, not crates.

## Testing & fixtures

Oracles: (1) ported `diffobs` (arc compared as *relative* transition; tolerance is
per-format — RINEX 5e-4, **obsj exact `f64`**); (2) convbin goldens as independent
reference; (3) the Go `convobs` binary run side-by-side for direct "identical-to-Go"
checks. RINEX validation is semantic (5e-4); **obsj validation is exact `f64`** —
the Rust converters must reproduce Go's `f64` bit-for-bit, making obsj the
high-precision regression oracle.

- **Committed small fixtures (≤ ~4 MB):** `packet-rtcm-20260519.{rtcm,obs.gz}`,
  `um980-rtcm-20260527.{rtcm,obs.gz}` (Stage 1b); `m8t/f9t-20251217.{ubx,obs.gz}`
  (Stage 2). Reuse satpulse's Makefile convbin flags + the documented BDS signal
  whitelist (`C6I`, `C7D/7P`).
- **Large uncommitted fixtures (`tmp/`, gitignored):** maasdam/serpa/x20p/ttyAMA0
  packet logs + their `.obs.gz`/`.obsj`, and `packet-rtcm-20260519-3h` — for
  big-scale validation and benchmarking.
- **Tests:** obsj↔rinex round-trip stability; RTCM→{rinex,obsj} vs Go and vs convbin;
  flag/error-parity unit tests; targeted edge cases (BDS B2b=`7D`, BDS B3I emitted,
  RTCM Doppler sign default vs `--rtcm-strict-prr`, GLONASS week resolution).

## Success criteria (correctness first, performance second)

- **Correctness:** **exact-`f64`** obsj match vs Go on every fixture (zero tolerance);
  zero `diffobs` differences (5e-4) on RINEX vs Go (modulo the documented whitelist);
  within-whitelist match to convbin goldens; error-message parity; stable
  obsj↔rinex round-trip. Converters reproduce Go's arithmetic bit-for-bit (operation
  order, constants, `f32` narrowings).
- **Performance:** beat Go `convobs` wall-clock on the large RTCM packet logs
  (initial target ≥2–3×); obsj output path streaming (O(1) memory). Measured on
  serpa/x20p/maasdam.

## Risks / gates

1. **`rinex` crate write throughput** — its buffered `BTreeMap` model + float
   formatting governs RTCM→RINEX speed. **Perf gate in Stage 1b:** convert a large
   RTCM log to RINEX via the crate, time vs Go; if it can't deliver the speedup, stop
   and discuss (do *not* silently port the writer).
2. **`rinex` crate metadata coverage** — verify early (Stage 1a) that all header
   fields convobs supports are settable and that output LLI/SSI are semantically
   correct; patch/vendor if not.
3. **`rtcm-rs` dormancy / pre-scaled fine fields** — recover raw ints for exact math
   where needed; vendor the crate if a gap appears.

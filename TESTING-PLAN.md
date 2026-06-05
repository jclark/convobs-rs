# convobs-rs — testing improvement plan

Goal: make `cargo test` (and CI, with no Go checkout and no external tools) the
authoritative validation of the conversion, diff, and week-resolution logic —
the way `go test` already is in `../satpulse`. Today the Rust port has only a
handful of unit tests and **no committed fixtures**; all end-to-end checks live
in gitignored `tmp/t/*.sh` scripts that shell out to the Go oracle and convbin.

## The pattern we're adopting (from satpulse)

`go test` there covers two layers with **zero external tools at test time**:

1. **In-process golden test** — `internal/convobscmd/convobs_test.go:TestGoldenFiles`
   runs the converter in-process to a `bytes.Buffer` (`convJob{out: &got}`), reads
   a **committed, gzipped** golden, and compares with the library's own semantic
   diff (`rinex.DiffObservations` / `DiffMetadata`, tol 5e-4). convbin is *not*
   invoked — `testdata/Makefile` regenerates the goldens out-of-band; the committed
   `.obs.gz` is the source of truth.
2. **Per-package unit tests** — e.g. `gps/lib/rnxrtcm/rtcm_test.go` (21 table-driven
   cases: week resolution, ambiguity, GLONASS FDMA channel, BeiDou +14 s offset,
   blank-phase LLI, zero-lock slip), `gps/lib/rinex/diff_test.go`,
   `decimate_test.go`, `write_test.go`.

We mirror both layers.

## Current Rust state (baseline)

- Unit tests exist only in `obsj/src/obs.rs` (time/sat), `obsj/src/arc.rs` (arc
  machinery), `obsj/src/rinexobs.rs` (blank-phase round-trip).
- **Untested in-crate:** `rtcm.rs` week resolution & cell math, `ubx.rs` mapping &
  slip logic, `diff.rs` (the comparator that *is* the oracle), `sink.rs` decimation
  grid, `freq.rs` frequency table, `json.rs` legacy-key rejection.
- No `testdata/`, no committed goldens, no `tests/` integration dirs.
- `STATUS.md` / `PORTING-NOTES.md` claim "committed small fixtures (`testdata/…`)" —
  that describes the *Go* repo and is currently false here; fix the docs once Step 2
  lands.

The pieces needed already exist and are just unused by tests:
`obsj::diff::{diff_observations, diff_metadata}` (in-process comparator, with
tolerances + `ignore_marker`), `convobs::read_obs_file` (loads RINEX/obsj→model,
auto-gunzips from content), `obsj::rinexobs::read_observation_file` /
`obsj::json::read_obsj` (read from any `BufRead`).

---

## Step 1 — convobs refactor: injectable output writer + clock (do this first)

**Why first:** every in-process test needs to run a conversion into a buffer with a
fixed clock, exactly like Go's `convJob{out: &got}.run(now)`. Today `convobs::run`
resolves its own output (`open_writer`) and reads the wall clock (`now_instant`)
internally, so it can only be driven as a subprocess against the real filesystem
and a nondeterministic time.

**Change (`cli/src/lib.rs`):** separate *parse* from *execute*, and thread the
writer and `now` in as parameters.

```rust
pub fn run(args: &[String]) -> Result<(), Error> {
    let Some(cfg) = parse_args(args).map_err(Error::Usage)? else { return Ok(()) };
    let writer = open_writer(cfg.output_path.as_deref())?;
    execute(&cfg, writer, now_instant())
}

/// Test/embedding seam: run a parsed conversion against a caller-supplied writer
/// and clock — mirrors Go's `convJob{out}.run(now)`. `--output` is ignored here;
/// the provided writer wins.
pub fn run_to_writer(args: &[String], out: Box<dyn Write>, now: Instant) -> Result<(), Error> {
    let Some(cfg) = parse_args(args).map_err(Error::Usage)? else { return Ok(()) };
    execute(&cfg, out, now)
}

fn execute(cfg: &Config, writer: Box<dyn Write>, now: Instant) -> Result<(), Error> {
    match cfg.from {
        InputFormat::Rinex | InputFormat::ObsJson => convert_observation_inputs(cfg, writer),
        _ => convert_packet_inputs(cfg, writer, now),
    }
}
```

- `convert_observation_inputs` and `convert_packet_inputs` take `writer: Box<dyn Write>`
  and **drop** their internal `open_writer(...)` calls.
- `Config` and `parse_args` stay private; the public seam takes `&[String]` args, so
  no internals leak. `Instant` is already `pub` (`obsj::obs::Instant`).

**Acceptance:**
- `convobs::run` behaviour unchanged (still writes `--output`/stdout); existing
  scripts pass.
- New smoke test: `run_to_writer(&["--from","obsj","--to","obsj", path], Box::new(Cursor), now)`
  converts a tiny in-memory input into a buffer and produces the expected bytes.
- `cargo clippy` clean.

---

## Step 2 — commit curated fixtures + goldens under `testdata/`

Port the relevant slice of `../satpulse/internal/convobscmd/testdata/`:

| case | input | golden | flags |
|---|---|---|---|
| `m8t` | `m8t-20251217.ubx` | `…obs.gz` | `--ubx-bds-geo-half-cycle` |
| `f9t` | `f9t-20251217.ubx` | `…obs.gz` | `--ubx-bds-geo-half-cycle` |
| `rtcm` | `packet-rtcm-20260519.rtcm` | `…obs.gz` | `--from rtcm --date-from-filename --rtcm-omit-zero-do` |
| `um980_rtcm` | `um980-rtcm-20260527.rtcm` | `…obs.gz` | `--from rtcm --date-from-filename --rtcm-omit-zero-do` |

- **Out of scope:** `um980-uncb` (Unicore) and its `C6I/C7D/C7P` ignored-signal
  whitelist — not a supported format here, and it's the largest pair.
- Port `testdata/README.md` + the convbin-flag `Makefile` as provenance (the
  goldens are regenerated out-of-band; tests never run convbin).
- **Open decision — repo size:** the four in-scope pairs are ~11 MB; a minimal
  `m8t` + one RTCM pair is ~4 MB (matches the "≤4 MB" note in PORTING-NOTES).
  Alternatively keep inputs out of git (LFS / fetch script) and skip when absent.
- Update `STATUS.md` / `PORTING-NOTES.md` so the "committed fixtures" claim becomes
  true.

---

## Step 3 — in-process golden test (`cli/tests/golden.rs`)

Mirror `TestGoldenFiles`, built on the Step 1 seam:

```text
for each case (name, args, golden_path, ignore_marker):
    buf = Vec::new()
    run_to_writer(args + [input_path], Box::new(&mut buf), fixed_now)   // RINEX into a buffer
    (got_meta, got_obs)  = obsj::rinexobs::read_observation_file(Cursor::new(buf))
    (want_meta, want_obs)= convobs::read_obs_file(golden_path, Rinex, None)  // auto-gunzips
    assert diff_metadata(got, want, mtol, ignore_marker) is empty
    assert diff_observations(got, want, tol{5e-4}, ignore_blank_phase=false) is empty
    on failure: write the JSONL diff to target/ and fail with its path
```

- Tolerances: `pr/cp/do/cn0 = 5e-4`, `approxPos/antennaDelta = 5e-5` (match
  `goldenTolerances()`).
- `diff_metadata` already ignores `Run`/`Comment`; pass `ignore_marker = true` for
  the two RTCM cases (matches Go's `cleanRTCM`). No extra metadata cleaning needed.
- Fixed `now` so the run is deterministic (the in-scope cases derive week from the
  filename, but inject it anyway for reproducibility and future `--recent`/auto
  cases).
- Add a self-contained obsj round-trip case (convert a committed RTCM/UBX fixture to
  obsj, read it back, convert again, assert stable) to cover the obsj path without a
  convbin golden.

**Acceptance:** `cargo test -p convobs-cli` exercises all four goldens with no Go
binary, no convbin, no `tmp/`. Also run under `--features convobs-cli/rinex-crate`
to cover the crate backend (with `ignore_blank_phase` where the crate can't emit
blank phase).

---

## Step 4 — library unit tests in `obsj` (mirror the Go package tests)

Highest-risk algorithmic core, testable as **pure functions** (no frame fixtures
needed — `rtcm-rs`/`ublox` own framing, so test the converter helpers directly):

- **`rtcm.rs`** ← `rnxrtcm/rtcm_test.go`: `resolve_week` (single match, ambiguous →
  error, no-match → error), `glonass_epoch_week_offsets` (day≠7 single, day==7 → 7
  candidates), `epoch_week_offsets` BeiDou `+14000 ms` & range checks,
  `resolve_continuity`/`continuity_candidate` half-week wrap, `rinex_sat_num`
  ranges, the `f32`-narrowing in `doppler`, `cn0` raw-int recovery.
- **`diff.rs`** ← `diff_test.go`: `diff_signal` tolerance behaviour per field,
  missing-side reporting, the arc→LL transition (`ArcToLl::transition`), `diff_metadata`.
- **`sink.rs`** ← `decimate_test.go`: `decimation_interval_ticks` rejects bad
  intervals; `DecimationSink` keeps only rounded-grid epochs; `RequireCpFilter`.
- **`freq.rs`**: known carrier frequencies incl. GLONASS FDMA (`1602 + k·0.5625`).
- **`json.rs`**: legacy-key (`ssi`/`lli`/`ll`) rejection; metadata vs observation
  dispatch; exact-f64 float round-trip via `RawValue`.
- **`ubx.rs`** ← `rnxubx/ubx_test.go`: `rinex_sig` mapping tables, `slip_hc`
  sub-half-cycle / lock-time / cp-stdev logic, `half_cycle_unresolved`.

**Porting nuance:** Go's converter tests synthesise MSM/RXM bit payloads; here the
crates decode framing, so prefer testing the pure helpers above. Where a full decode
path is wanted, add a tiny committed single-frame byte fixture rather than
hand-encoding bits.

---

## Step 5 — CI wiring (last)

Now that goldens are committed and tests are hermetic:

- CI job: `cargo test --workspace`, then `cargo test --workspace --features convobs-cli/rinex-crate`,
  then `cargo clippy --workspace --all-targets -- -D warnings`.
- Keep `tmp/t/*.sh` as an *additional* cross-check against the live Go oracle for
  local/manual runs, but it is no longer the only gate.

---

## Go → Rust test mapping (where each lands)

| Go | Rust home |
|---|---|
| `convobscmd/convobs_test.go::TestGoldenFiles` | `cli/tests/golden.rs` |
| `convobscmd/convobs_test.go` flag/parse tests | `cli/tests/` or `#[cfg(test)]` in `cli/src/lib.rs` |
| `rnxrtcm/rtcm_test.go` | `#[cfg(test)]` in `obsj/src/rtcm.rs` |
| `rnxubx/ubx_test.go` | `#[cfg(test)]` in `obsj/src/ubx.rs` |
| `rinex/diff_test.go` | `#[cfg(test)]` in `obsj/src/diff.rs` |
| `rinex/decimate_test.go` | `#[cfg(test)]` in `obsj/src/sink.rs` |
| `rinex/write_test.go` / `read_test.go` | `#[cfg(test)]` in `obsj/src/rinexobs.rs` (extend) |

## Sequencing & open decisions

1. **Step 1** (refactor) — unblocks everything, no fixtures required.
2. **Step 2** (fixtures) — choose size budget *(open decision)*.
3. **Step 3** (golden harness) — first real end-to-end coverage in `cargo test`.
4. **Step 4** (unit tests) — can proceed in parallel with 2–3.
5. **Step 5** (CI).

Open decisions to settle before/while doing Step 2–3:
- Fixture size budget: minimal (~4 MB) vs all four pairs (~11 MB) vs out-of-git.
- Keep or retire the `tmp/t/*.sh` oracle scripts once Steps 3–4 land.

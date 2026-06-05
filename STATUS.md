# convobs-rs — status

Snapshot as of 2026-06-05, after the staged migration in `PLAN.md`. The repo is
now the **cargo workspace** the plan describes — `obsj` (the leaf library),
`rinex-obsj` (the `rinex`-crate bridge), and a CLI package (`convobs` +
`diffobs`) — not the single-package prototype. Everything below is measured
against the Go oracle (`tmp/oracle/satpulsetool`) and the convbin goldens, with
our `diffobs`.

`cargo test` is now the hermetic, authoritative gate (no Go checkout, no convbin,
no `tmp/` at test time): `cli/tests/golden.rs` runs the four committed
`testdata/` fixtures in-process through `convobs::run_to_writer` and compares
each against its convbin golden with the library's own `diff_observations` /
`diff_metadata` (5e-4), covering both the DIY backend and — under
`--features convobs-cli/rinex-crate` — the crate backend. `obsj` carries
per-module unit tests for the algorithmic core (week resolution, slip/LLI, the
diff comparator, decimation, the frequency table, obsj parsing). The `tmp/t/*.sh`
oracle scripts remain as an extra cross-check against the live Go oracle.

## Migration: stages 1–8 complete

| stage | result |
|---|---|
| 1. workspace scaffold | `obsj` / `rinex-obsj` / `cli`, builds green |
| 2. centralized `ll ↔ arc` | converters emit `ll`; `LossOfLockSink` accumulates `arc`; `ArcToLl` is the inverse; `diff` uses it |
| 3. `rinexobs` DIY backend | golden → obsj → RINEX → re-read round-trips; blank phase = `cp=None` |
| 4. `rtcm` on `rtcm-rs` | RTCM → obsj = **0 diffs at exact f64** vs Go (raw ints recovered from rtcm-rs's pre-scaled f64) |
| 5. `ubx` on `ublox` | UBX → obsj = **0 diffs at exact f64** vs Go |
| 6. `rinex-obsj` bridge | `RinexObsj` extension trait (upstreamable); RTCM/UBX → RINEX matches convbin golden with `--ignore-blank-phase` |
| 7. CLI package | real error type; compile-time `rinex-crate` feature; runtime backend selection; gzip-from-content; `diffobs` formats by explicit option (never filename) |
| 8. raw packet streams | `-r raw` auto-detects a single family (UBX *or* RTCM) and matches Go |

Stage 9 (performance) is the only remaining work: profiling and easy hotspot
fixes — see *Performance*.

## What works (validated against the Go oracle / convbin goldens)

obsj is the exact-f64 path — every obsj conversion matches Go with **0 diffobs
differences**:

| path | result |
|---|---|
| RTCM stream → obsj | 0 diffs, 114 082 records |
| RTCM stream (3 h) → obsj | 0 diffs, 1 255 009 records |
| RTCM packet-log (JSONL) → obsj | 0 diffs |
| UBX stream → obsj (m8t / f9t) | 0 diffs (14 217 / 58 455) |
| RTCM `--interval 30` → obsj | 0 diffs (decimation-robust via centralized `arc`) |
| `-r raw` RTCM stream → obsj | 0 diffs |
| `-r raw` UBX stream → obsj | 0 diffs (pack-generated single-family) |
| obsj → obsj round-trip | self-stable |

RINEX, both backends (semantic diff: 5e-4):

| path | result |
|---|---|
| RTCM/UBX → RINEX (**DIY**, default) vs convbin golden | **0 diffs**, faithful blank phase (RTCM also `--ignore-marker`) |
| RTCM/UBX → RINEX (**crate bridge**) vs convbin golden | 0 diffs with `--ignore-blank-phase` (the crate cannot emit blank phase) |
| DIY golden round-trip (113 180 obs) | stable |

The decoders are now `rtcm-rs` (framing/CRC/MSM) and `ublox` (framing/RXM-RAWX);
the hand-rolled bit decoders and `crc24q` are gone. Exact-f64 is preserved by
recovering the raw integers from the crates' pre-scaled fields.

## Backends and features

- **DIY RINEX** (`obsj` `rinexobs` feature) is the default: self-contained,
  faithful blank-phase, no heavy deps. The lean binary is DIY-only.
- **Crate bridge** (`rinex-obsj`, behind the CLI's `rinex-crate` feature) adds
  CRINEX/Hatanaka via the `rinex` crate; engaged with `--rinex-backend crate`
  or auto for CRINEX input. In the lean build, asking for it errors cleanly.
- Compression is detected from content (gzip magic), never the filename.
- `diffobs` takes each input's format by `--format`/`--a-format`/`--b-format`
  and has `--ignore-blank-phase` and `--ignore-marker`.

## Not yet done

- Mixed UBX/RTCM **interleaved** raw streams are intentionally out of scope for
  now — `-r raw` locks to the first family seen.
- Unicore (uncb/unca) — out of scope.
- Deeper performance work (noted in `PERFORMANCE.md`) — not needed at current
  speeds.

## Performance

Stage 9 done: profiled the hot path with **samply** and took the easy wins —
`faster-hex` for the hex decoder (was 18% self-time), `rustc-hash` `FxHashMap`
for the `(sat,sig)` maps (SipHash was ~14%), skip non-RXM-RAWX UBX before
decoding, a hand-written obsj serializer instead of `#[serde(flatten)]`, and one
CRC/parse per packet-log RTCM frame. On `--interval 30 → obsj` packet logs the
workspace build is **≈12–40× faster than Go with ≈5× less memory** (peak RSS
< 4 MB on a 1.3 GB log), output bit-identical at exact f64.

The obsj **input** path was profiled too: it now parses each line in a single
pass (no `serde_json::Value` intermediate; observation floats captured as raw
JSON tokens and rounded with std `f64::from_str`, so `arbitrary_precision` is
gone) and **streams** records straight into the sink. obsj→obsj dropped from
6.6 s / 498 MB to **2.9 s / 3.2 MB** (O(1) memory). RINEX output still buffers
(inherent — the header needs every epoch). What remains in the profile is
rtcm-rs's MSM decode and serde_json — library/inherent. Full methodology, the
profile, and per-file numbers are in **`PERFORMANCE.md`**.

## How to reproduce

```sh
cd ../satpulse && go build -o ../convobs-rs/tmp/oracle/satpulsetool ./cmd/satpulsetool
cd ../convobs-rs && cargo build --release            # lean (DIY) binary
cargo build --release --features convobs-cli/rinex-crate   # with the crate backend

./tmp/t/regress.sh     # obsj exact-f64 gates vs Go
./tmp/t/rinexgate.sh   # RINEX gates, both backends, vs convbin goldens
./tmp/t/rawgate.sh     # -r raw single-family streams vs Go
```

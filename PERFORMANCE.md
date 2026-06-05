# convobs-rs — performance report

Stage 9 of the `PLAN.md` migration: profile the hot path, take the easy wins,
report. The hot path is large JSONL packet logs converted to obsj with
`--interval 30` (typical PPP usage).

## Method

Profiled with **samply** (a sampling profiler) at 2 kHz, symbolicated against a
debug-info release build (`addr2line`). The box caps locked memory at 8 MB, so
samply is pinned to 4 CPUs (`taskset -c 0-3`) to fit the per-CPU perf ring
buffers. Wall-clock and peak RSS are from `/usr/bin/time -v` on a warm page
cache. Every result was checked identical to the Go oracle with `diffobs`
(exact f64) — no optimization changed the output.

A differential pass first located the stages (serpa, 1.6 GB RTCM): JSON line
parsing ≈ 0.5 s (small), RTCM decode ≈ 4 s (dominates `--interval 30`), obsj
output ≈ 5 s (dominates full output). The sample profile then named the
functions.

## Profile (RTCM `--interval 30`), before → after

Top self-time before the stage-9 changes:

| % | function |
|---|---|
| 18.1 | `convobs::packetlog::hex_decode` (hand-rolled) |
| 8.8 + 5.2 | `DefaultHasher::write` + `hash_one` — SipHash on `(sat,sig)` keys |
| 9.5 | `rtcm_rs … Parser::parse` |
| 7.5 | `obsj::rtcm … convert_cell` |
| 7.0 | `crc_any … CRC::digest` |

After: `hex_decode` and SipHash are gone from the profile; hashing is ~2%. What
remains is rtcm-rs's own MSM decode (`Parser::parse` 11%, `CRC::digest` 9%, the
`df*::decode` field unpackers) plus serde_json and the buffered writer — library
and inherent costs, not worth chasing.

## Optimizations (each measured, output unchanged)

1. **Skip non-RXM-RAWX UBX before decoding.** Most UBX in a packet log is NAV,
   not RXM-RAWX (~10% on maasdam). The frame header (`b5620215`) is recognised
   from the `bin` hex *before* hex-decoding. ~90% of UBX lines are dropped cheap.
2. **`faster-hex`** (SIMD) replaces the hand-rolled hex decoder — the #1 hotspot.
3. **`rustc-hash` `FxHashMap`** for the per-`(sat,sig)` maps in the converters
   and the arc accumulator — SipHash is overkill for 5-byte keys.
4. **Hand-written obsj `Serialize`** instead of `#[serde(flatten)]`, which on the
   write side buffers every record through serde_json's `Content` map. (Reading
   keeps `flatten`.) Byte-identical output.
5. **One CRC/parse per packet-log RTCM frame** — the payload is a single framed
   message, so it is decoded directly instead of validated by `frames()` and
   again by `convert_frame`.

Effect (vs the pre-stage-9 build): serpa RTCM `--interval 30` 4.64 s → 2.85 s;
maasdam UBX `--interval 30` 1.31 s → 0.51 s; x20p UBX `--interval 30`
2.2 s → 1.05 s. Items 2 and 3 (the profile-driven hex + hash swaps) were the
bulk of it.

## Results — sample files → obsj (wall-clock; peak RSS < 4 MB throughout)

| file | size | mode | Rust | Go |
|---|---|---|---|---|
| packet-rtcm `.rtcm` | 1.5 MB | RTCM stream, full | 0.07 s | — |
| maasdam packet log | 590 MB | UBX, `--interval 30` | **0.51 s** | 20.3 s |
| maasdam packet log | 590 MB | RTCM, `--interval 30` | **0.93 s** | 16.1 s |
| x20p packet log | 1.3 GB | UBX, `--interval 30` | **1.05 s** | 40.6 s |
| x20p packet log | 1.3 GB | RTCM, `--interval 30` | **1.69 s** | 27.3 s |
| serpa packet log | 1.5 GB | RTCM, `--interval 30` | **2.85 s** | ~27 s |
| serpa packet log | 1.5 GB | RTCM, full | 7.72 s | — |

≈ 12–40× faster than Go on the `--interval 30` hot path, ≈ 5× less memory,
output bit-identical at exact f64. Memory is O(1) in input size (obsj is
streamed), so a 1.3 GB log peaks under 4 MB.

## obsj as input (`-r obsj`)

The obsj *reader* was the second target. The profile showed it spread across
serde_json with a lot of allocation: the reader parsed each line to a
`serde_json::Value` and then `from_value` into the record — two passes, forced
by `#[serde(flatten)]` on the wire type — and the whole file was buffered in a
`Vec`. Two fixes:

- **One-pass parse.** Read each line directly into a flat record struct (no
  `Value`, no `flatten`). Observation floats are captured as raw JSON tokens
  (serde_json `RawValue`) and rounded with std `f64::from_str`, which is
  correctly rounded — so `arbitrary_precision` (whose correct rounding only
  applies via `Value`, and which is slow) was dropped entirely. Round-trip stays
  bit-exact.
- **Streaming.** obsj records are pushed straight into the sink as they are
  parsed instead of collected, so obsj→obsj is O(1) memory.

On a 398 MB / 2.88 M-record obsj file:

| path | before | after |
|---|---|---|
| `-r obsj --to obsj` | 6.61 s, 498 MB | **2.90 s, 3.2 MB** |
| `-r obsj --to rinex` | 8.06 s, 1.66 GB | **4.26 s, 1.41 GB** |

obsj→obsj is now fully streamed (3 MB regardless of size). obsj→RINEX still
buffers — the RINEX writer needs every epoch to emit the header and sorted body
— but no longer double-buffers the input.

## Left for later (not "easy")

- The remaining `--interval 30` cost is rtcm-rs's MSM decode itself plus the
  converter's per-message `Vec<SatData>`/`Vec<SigData>` normalization. Removing
  those allocations means iterating the crate's data segments without the
  unifying macro — a real refactor; the profile shows it would only chip at the
  ~2% the allocations cost, so it was not done.
- Full (non-decimated) output is bounded by ryu float formatting of ~5 fields ×
  millions of records — close to the floor for JSON text.

To reproduce a profile: `taskset -c 0-3 samply record -- ./target/release/convobs …`
(needs `perf_event_paranoid ≤ 1`).

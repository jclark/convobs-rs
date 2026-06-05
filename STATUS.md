# convobs-rs — status

Snapshot of the prototype as of 2026-06-05. It is a **single-package prototype**
(under `src/`), not yet the crate split described in `PLAN.md`. Everything below
is measured against the Go oracle (`tmp/oracle/satpulsetool`) and the convbin
goldens, compared with our `diffobs`.

## What works (validated)

obsj is the strong, fully-validated path — every obsj conversion below matches
the Go oracle at **exact f64 (0 diffobs differences)**:

| path | result |
|---|---|
| RTCM stream (`.rtcm`) → obsj | **0 diffs**, 114 082 records |
| RTCM packet-log (JSONL) → obsj | **0 diffs**, 136 865 records |
| UBX stream → obsj (m8t / f9t) | **0 diffs** (14 217 / 58 455 records) |
| RTCM `--interval 30` → obsj (decimation) | **0 diffs**, 4 673 records |
| `-r raw` auto-detect (RTCM) → obsj | **0 diffs**, 114 082 records |
| obsj → obsj round-trip | self-stable (0 diffs) |

So: **RTCM and UBX → obsj, as streams and as packet logs, with decimation and
raw auto-detect, all reproduce Go bit-for-bit at f64.** This is the headline
capability and it works.

The **DIY RINEX reader/writer** (`src/rinexobs.rs`) is validated *standalone*:
it reads a convbin golden's blank carrier-phase fields faithfully (`cp=None`
with `arc` incremented — where the `rinex` crate invents `cp:1.0`) and
round-trips 113 180 observations stably. It is **not yet wired into the CLI**.

## Partial / known limitations

- **RINEX I/O in the CLI currently uses the `rinex` crate** (`src/rinexio.rs`),
  which cannot represent a missing carrier phase carrying only a loss-of-lock
  flag:
  - RTCM → RINEX vs the convbin golden: **3 781 diffs**, *all* the blank-phase
    category (3 753 `cp,ll` + 27 `cp` + 1 metadata). The other observation
    values are correct; this is purely the blank-phase limitation, and would be
    ~0 under the planned `diffobs --ignore-blank-phase`.
  - `-r rinex` input → obsj: the crate reader produces **3 780 bogus `cp:1.0`**
    values from blank-phase fields.
  - The faithful path (the DIY `rinexobs` module) exists and is validated but is
    not selected by the CLI yet (no backend selection wired).
- `diffobs` itself reads `.obs` via the `rinex` crate, so diffing RINEX files
  inherits the same blank-phase misread.
- **`diffobs` (and any CLI) must not infer format from the filename.** The
  prototype's `diffobs` currently picks its reader (obsj vs RINEX) *and* its
  default tolerance from the extension — so an obsj file named otherwise is
  mis-read as RINEX (this is exactly how a `.go`/`.rs`-named obsj file produced a
  spurious "error" during perf testing). Per the Go-reference CLI design
  (PLAN.md), each input's format must come from an explicit option; only
  compression may be detected from content.
- **Mixed UBX/RTCM raw streams are not handled.** `-r raw` works on a
  single-family byte stream (tested: raw RTCM → obsj = 0 diffs), but the
  prototype's raw *stream* path scans only one family over the whole buffer, so
  an interleaved UBX+RTCM stream is mis-framed. The fix is the thin demux over
  the crate framers (PLAN.md stage 8) — *not* porting the Go scanner. The
  raw-mode metadata family-lock/buffering is also simplified vs the Go behaviour.
  - **How to create test streams:** `satpulsetool pack` turns a JSONL packet log
    into a packet byte stream. `pack log.jsonl > mixed.bin` emits all tags
    interleaved (the mixed case to fix); `pack --tag RTCM` / `--tag UBX` emit a
    single family. So the `tmp/` packet logs can be replayed as raw streams to
    exercise `-r raw` both ways:
    ```sh
    tmp/oracle/satpulsetool pack tmp/rinex/x20p-20260531.packet.jsonl > tmp/t/mixed.bin
    ./target/release/convobs -r raw --recent --to obsj tmp/t/mixed.bin   # must match Go
    ```

## Not yet done (per the PLAN.md migration)

- Crate split (`obsj` / `rinex-obsj` / CLI) — still one package.
- Centralized `ll ↔ arc` — converters still keep arc state internally.
- Decode via `rtcm-rs` / `ublox` — currently hand-rolled bit decoders (these are
  what give the exact-f64 obsj match today).
- Compile-time `rinex-crate` feature + runtime backend selection.
- `diffobs --ignore-blank-phase`.
- Idiomatic error types (currently `Result<_, String>`).
- Raw-mode metadata buffering / family-lock edge cases (simplified).
- Large-scale perf validation (only ~20 MB tested so far).
- Unicore — out of scope.

## Performance

Measured against the Go oracle on real-scale RTCM packet logs; **every output is
bit-identical at exact f64** (diffobs = 0). Wall-clock time / peak RSS:

| input | mode | Go | Rust | speedup |
|---|---|---|---|---|
| serpa-tail, 20 MB | `--interval 30` → obsj | 0.58 s / 17.8 MB | 0.04 s / 4.0 MB | ~14× |
| maasdam, 591 MB | `--interval 30` → obsj | 16.1 s / 18.7 MB | 1.04 s / 3.9 MB | **~15×** |
| maasdam, 591 MB | full → obsj (2.5 M rec, 346 MB out) | 19.9 s / 18 MB | 1.94 s / 4.1 MB | ~10× |
| x20p, 1.3 GB | `--interval 30` → obsj | 26.7 s / 18.6 MB | 2.01 s / 3.8 MB | ~13× |

≈ 10–15× faster, ≈ 5× less memory — and the Rust side is **not yet perf-tuned**
(stage 8). Caveats for honesty: this is a JSON-emit-heavy workload, so much of the
gap is Go's reflection-based `encoding/json` (and the Go writer was already
hand-tuned — see the `linebuf`/`appendf` pprof artifacts in `tmp/`). Go's
`encoding/json/v2` would narrow the encoding part; the GC/allocation/no-reflection
advantage stays on the Rust side.

## How to reproduce

```sh
# build the Go oracle (once):
cd ../satpulse && go build -o ../convobs-rs/tmp/oracle/satpulsetool ./cmd/satpulsetool
cd ../convobs-rs && cargo build --release

TD=../satpulse/internal/convobscmd/testdata
O=./tmp/oracle/satpulsetool; R=./target/release/convobs; D=./target/release/diffobs

# obsj exact-f64 check (expect diffobs exit 0):
$O convobs -r rtcm --date-from-filename --rtcm-omit-zero-do --to obsj $TD/packet-rtcm-20260519.rtcm >go.obsj
$R          -r rtcm --date-from-filename --rtcm-omit-zero-do --to obsj $TD/packet-rtcm-20260519.rtcm >rs.obsj
$D go.obsj rs.obsj   # exit 0 = identical
```

# convobs-rs — plan

A small, efficient Rust implementation of the observation conversions that
satpulse's `convobs` performs (RTCM MSM7 / UBX RXM-RAWX / RINEX / obsj →
RINEX / obsj), with the **obsj format as a first-class, standalone Rust crate
usable outside satpulsetool**.

This is **not** a line-by-line port of the Go tool. The Go source
(`../satpulse`) is a reference for the algorithms and the **man page**
(`docs/convobs.1.md`) is the behaviour spec, but the code is **idiomatic Rust**
and must not read like transliterated Go. We leverage the Go implementation a
different way: as a **test oracle**, run side-by-side and compared with
`diffobs` (see below).

Non-goals: byte-identical output to Go; reproducing Go's exact error strings or
every edge case; Unicore (uncb/unca) input.

## Guiding principles

- **Idiomatic Rust, not a port.** Traits, iterators, `?`, serde, features,
  ecosystem crates — structure the code the way a Rust library would be
  structured, not the way the Go package is. No "ported from `*.go`" framing.
- **CLI behaviour follows the Go reference design.** The man page
  (`docs/convobs.1.md`, which derives from `convobs.go`) is the behaviour
  contract: the same options, the same semantics and validations. In particular,
  **input/output format is selected only by explicit options** (`-r`/`--from`,
  `--to`) — *never* inferred from a filename extension or sniffed from content.
  The same rule applies to `diffobs` (the format of each input is given by an
  option) and to anything we add beyond the Go tool: the RINEX backend selection
  chooses a *backend*, never the *format*; compression may be detected from
  content (gzip magic bytes), but never the format. (This is about *behaviour and
  option design* — the implementation and the exact error wording stay idiomatic
  Rust; matching Go's error strings is not a goal.)
- **diffobs is the oracle; byte-identity is a non-goal.** Output is validated
  *semantically*, not byte-for-byte. This is what frees us to use serde for
  obsj and the `rinex` crate for RINEX even though their bytes differ from Go's.
- **obsj is the centerpiece** and the one genuinely reusable artifact: a clean,
  light, standalone crate anyone can depend on.
- **Minimal code, lean dependencies, efficient on large inputs.**

## Validation: diffobs

`diffobs` is a semantic comparator over the obsj model (align by epoch and
`(sat,sig)`, compare per field within tolerance):

- **obsj — exact f64 (tolerance 0).** serde + `ryu` round-trips floats
  bit-exactly; the reader enables serde_json `arbitrary_precision` so parsing is
  correctly rounded (serde_json's default float parse is not).
- **RINEX — 5e-4** (its three-decimal text precision).
- Independent references: convbin (RTKLIB-EX) goldens, and the Go `convobs`
  binary run side-by-side.
- **`--ignore-blank-phase`** skips the `cp`/`ll` comparison when one side lacks
  a carrier phase, so the rinex-crate RINEX path still validates (see below).

## Crate decomposition (one workspace / repo)

- **`obsj`** (library — the centerpiece). The obsj format and model: `GpsTime`,
  `SatId`/`SigId`, `SignalValues`, `Metadata`; serde (de)serialization; the
  **`arc ↔ ll` loss-of-lock machinery**; and the **diff** logic. **Depends on
  nobody** (a DAG leaf), licensed permissively (MIT/Apache) so heavier crates
  can depend on it. Optional features:
  - `rinexobs` — the self-contained DIY RINEX reader/writer.
  - `rtcm` — RTCM MSM7 → obsj via `rtcm-rs`.
  - `ubx` — UBX RXM-RAWX → obsj via `ublox` (same shape as `rtcm`).
- **`rinex-obsj`** (library — the bridge). Maps both ways between the obsj model
  and the `rinex` crate's model. Depends on `obsj` + `rinex`. Written against
  **public APIs only** and shaped as free functions / an extension trait, so it
  can be **contributed upstream** as the `rinex` crate's own `obsj` feature with
  near-mechanical change.
- **CLI** (separate package — binaries `convobs` + `diffobs`). Packet-log (JSONL)
  parsing, RTCM week inference, decimation / `--ppp-ar`, format orchestration —
  everything tangential to the obsj format lives here so `obsj` consumers don't
  pay for it. Depends on `obsj`; optional dep on `rinex-obsj` behind a
  `rinex-crate` feature.

Dependency rules: Cargo forbids crate cycles, so the graph is a DAG. `obsj` is a
leaf; `rinex-obsj` and the CLI point *into* it; nothing points back. The CLI is
a separate package specifically to avoid an `obsj ⇄ rinex-obsj` cycle and to
keep `obsj`'s dependencies minimal. **No fork of the `rinex` crate is needed** —
it is used as published.

## Centralized arc/ll handling (improvement over the Go architecture)

In Go, every converter (`rnxrtcm`, `rnxubx`, …) maintains its own per-signal
lock/slip state and computes the `arc` counter itself. Here that bookkeeping is
**centralized in the `obsj` crate**: converters only *detect a slip* and emit a
per-observation loss-of-lock boolean (`ll`); the `obsj` crate's streaming
`ll → arc` accumulator turns that into the monotonic `arc` per `(sat,sig)`. The
inverse `arc → ll` transform feeds RINEX LLI and the diff comparator.

Consequences:
- Converters get simpler — no arc maps, no per-converter slip bookkeeping.
- One tested implementation of slip→arc, reused by every converter, by both
  RINEX backends, and by diffobs.
- The diff comparator compares an `ll` field directly instead of reconstructing
  transitions inline (as the Go comparator does).

## obsj format notes

- `t` (and metadata dates) are **ISO 8601 date-times without a time-zone
  designator** (`t` is GPST). *Not* RFC3339. (Packet-log timestamps — a CLI
  input, not obsj — are RFC3339; keep them distinct.)
- On the wire, loss of lock is **`arc`** (monotonic per-`(sat,sig)` counter):
  the canonical, decimation-robust form — a slip inside a dropped gap still
  surfaces on the next kept epoch. `hc`/`bt` are RINEX LLI bits 1/2. `cn0` in
  dB-Hz (derived from SSI on RINEX read when no `S` observable).
- The full field set and semantics are the man page (`docs/convobs.1.md`).

## Two RINEX backends

The `rinex` crate's observation model stores a mandatory `value: f64`, so it
**cannot represent a missing carrier phase that carries only a loss-of-lock
indicator** (a blank phase field on a pseudorange-only signal). Its reader
invents `cp = 1.0`; its writer cannot emit the blank. That case is not needed
for the product — only for diffing against convbin/Go — so we offer two paths:

- **`rinexobs` (DIY)** — feature of the `obsj` crate. Fixed-column reader/writer
  with an **optional** observation value, so blank-phase-with-LLI round-trips
  faithfully (matches convbin/Go). Light (no heavy deps), fast, ours; narrow
  scope (RINEX 3.x observation files; no CRINEX/nav/meteo).
- **`rinex-obsj` (crate bridge)** — full `rinex`-crate ecosystem: CRINEX/Hatanaka,
  gzip, broad version coverage, QC, interop, upstreamable. Heavier; inherits the
  blank-phase limitation (handled in tests by `diffobs --ignore-blank-phase`).

**Backend selection:**
- **Compile time** — a `rinex-crate` feature gates whether the `rinex` crate is
  linked at all. Off by default → lean, DIY-only binary.
- **Runtime** (only when compiled in) — the crate backend is used when
  explicitly requested (a flag) or auto-engaged when the job needs a capability
  DIY lacks (e.g. CRINEX in/out); otherwise DIY handles it. In the lean build,
  requesting CRINEX / the crate flag errors cleanly.

## Decode

- **RTCM** via `rtcm-rs` (framing/CRC/MSM bit-unpacking), recovering raw ints
  from its pre-scaled fields where exact arithmetic matters, feeding the
  converter (cell math, slip from DF407 lock time, GLONASS week resolution,
  metadata messages 1005/1006/1007/1008/1013/1033/1230).
- **UBX** via `ublox`, same pattern. (`sigId` is reachable as `reserved2()` in
  ublox 0.10.)
- **raw** (`-r raw`, mixed UBX/RTCM): a thin demux over the crates' own framers —
  do **not** port the Go scanner.

## Performance

The hot path is large packet logs (hundreds of MB) converted with
**`--interval 30`** (typical usage): JSONL line → hex-decode → decode
(rtcm-rs/ublox) → convert → decimate → stream to obsj/RINEX. Keep it streaming
and allocation-light. **Benchmark with `--interval 30` on the large fixtures**
(`tmp/`: serpa/x20p/maasdam/ttyAMA0), exercised both as packet logs and as
`satpulsetool pack`-generated raw streams. obsj output is O(1) memory; RINEX
buffers, but decimation keeps the buffer small.

## Fixtures & oracles

- **Committed small fixtures** (`testdata/`, ≤ ~4 MB): `packet-rtcm-20260519`,
  um980 RTCM, m8t/f9t UBX — convbin (RTKLIB-EX) goldens, with the documented
  signal whitelist (`C6I`, `C7D/7P`).
- **Large uncommitted fixtures** (`tmp/`, gitignored): serpa/x20p/maasdam/ttyAMA0
  packet logs + `.obs.gz`/`.obsj`, `packet-rtcm-20260519-3h` — big-scale
  validation and the perf gate.
- Oracles: diffobs (exact-f64 obsj, 5e-4 RINEX), convbin goldens, and the Go
  `convobs` binary side-by-side.

## Repository & environment (for a fresh start)

- **Toolchain:** Rust 1.96, edition 2021. The repo is a git repo with a single
  commit (the original plan); **none of the prototype code is committed yet**.
- **Go reference & oracle:** `../satpulse`. The oracle binaries are prebuilt under
  `tmp/oracle/` (gitignored); rebuild with
  `cd ../satpulse && go build -o <repo>/tmp/oracle/satpulsetool ./cmd/satpulsetool`
  and `go build -o <repo>/tmp/oracle/diffobs ./gps/lib/rinex/diffobs`. Run
  `tmp/oracle/satpulsetool convobs …` to produce reference output for any input.
- **Man page (behaviour spec):** `docs/convobs.1.md` (this repo, Unicore removed);
  original at `../satpulse/docs/man/satpulsetool-convobs.1.md`.
- **Fixtures:** committed goldens live in
  `../satpulse/internal/convobscmd/testdata/` — copy the in-scope ones into this
  repo's `testdata/`. Large fixtures are in `tmp/` (gitignored). See *Fixtures &
  oracles*. (The prototype currently reads fixtures from the satpulse path
  directly; a real test suite should copy them in.)
- **Protocol docs:** `../gps-protocol-docs/`.
- **`PORTING-NOTES.md`** holds the load-bearing algorithm details, constants, and
  crate caveats — read it alongside this plan.

## Code inventory (prototype → target)

The prototype is a single package under `src/`. **Reuse the validated cores;
restructure, don't rewrite.**

| prototype file | target | action |
|---|---|---|
| `obs.rs` (model, `GpsTime`, serde) | `obsj` core | reuse ~as-is |
| `obsj.rs` (serde read/write) | `obsj` core | reuse — exact-f64 proven |
| `diff.rs` (comparator) | `obsj` core | reuse; refactor onto centralized `ll` |
| `sink.rs` (decimation, requireCP) | `obsj` / CLI | reuse; add the `ll→arc` stage |
| `freq.rs` (frequency table) | `obsj` (converter support) | reuse |
| `rinexobs.rs` (DIY RINEX) | `obsj` `rinexobs` feature | reuse — blank-phase proven |
| `rtcm.rs` / `ubx.rs` converter *logic* | `obsj` `rtcm`/`ubx` features | reuse algorithm; **swap bit-decode → rtcm-rs/ublox** |
| `rinexio.rs` (rinex-crate adapter) | `rinex-obsj` bridge | reuse as the bridge |
| `crc24q.rs` | — | drop (rtcm-rs owns CRC/framing) |
| `cli.rs`, `packetlog.rs` | CLI package | reuse logic; idiomatize errors; add backend selection |

Validated so far: **RTCM packet → obsj matches the Go oracle at exact f64 (0
diffobs differences)**; the DIY RINEX reader reads blank-phase faithfully
(`cp=None`, `arc` incremented) and round-trips stably (113,180 obs).

## Staged migration plan

Each stage keeps the build green and ends with a **diffobs gate**. The Go oracle
and convbin goldens are the references. Throughout: idiomatic Rust (traits, `?`,
iterators, real error types), not a transliteration.

1. **Workspace scaffold.** Convert the single package into a cargo workspace:
   members `obsj` (lib), `rinex-obsj` (lib), and a CLI package (bins `convobs`,
   `diffobs`). *Gate:* `cargo build` of the workspace.

2. **`obsj` core + centralized `ll ↔ arc`.** Move `obs`/`obsj`/`diff`/`sink`/`freq`
   into `obsj`. Add the public `arc → ll` transform and a streaming `ll → arc`
   accumulator as a pipeline stage **upstream of decimation** (so `arc` stays
   decimation-robust). Refactor `diff` to compare the `ll` transition via the
   transform. obsj depends only on serde/serde_json. *Gate:* obsj unit tests;
   obsj→obsj round-trip stable.

3. **`rinexobs` feature.** Move the DIY RINEX reader/writer into `obsj` behind a
   `rinexobs` feature, wired through the `ll ↔ arc` transform. *Gate:* convbin
   golden → obsj → RINEX → re-read round-trips; blank-phase reads as `cp=None`.

4. **`rtcm` feature on rtcm-rs.** Move the RTCM converter logic into `obsj` behind
   a `rtcm` feature; replace the hand-rolled bit-decode with `rtcm-rs` (recover
   raw ints from its pre-scaled fields). The converter emits `ll`; the accumulator
   assigns `arc`. *Gate:* RTCM packet → obsj vs the Go oracle = **0 diffs at exact
   f64** (the prototype already hits this with its own decoder).

5. **`ubx` feature on ublox.** Same shape with the `ublox` crate. *Gate:* UBX →
   obsj vs Go oracle = 0 diffs at exact f64 (committed m8t/f9t fixtures).

6. **`rinex-obsj` bridge.** Move the rinex-crate adapter into the `rinex-obsj`
   crate (deps `obsj` + `rinex`), shaped as free functions / an extension trait so
   it is upstreamable. *Gate:* RTCM → RINEX via the bridge vs convbin golden with
   `diffobs --ignore-blank-phase` = 0 (within the documented whitelist).

7. **CLI package.** Move orchestration, packet-log parsing, week inference, and
   flags into the CLI bins with a real error type. Implement the compile-time
   `rinex-crate` feature and runtime backend selection (DIY default; crate when
   requested or when a capability such as CRINEX needs it). `diffobs` is a thin
   wrapper with `--ignore-blank-phase`. *Gate:* the golden test set passes; flag
   validation matches the man page.

8. **Raw mode + performance.** Thin `-r raw` demux (mixed UBX/RTCM) over the crate
   framers — *not* the Go scanner. Profile and tune the packet-log `--interval 30`
   streaming path. *Gate:* large fixtures (serpa/x20p/maasdam) convert correctly
   (diffobs vs Go) and beat Go's wall-clock, exercised both as packet logs and as
   `satpulsetool pack` raw streams.

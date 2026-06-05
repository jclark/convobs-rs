# convobs

**convobs** converts raw GNSS observation data emitted by a receiver into a
RINEX observation file. The resulting file can be sent to a PPP post-processing
service such as CSRS-PPP to determine the precise position of the receiver. The
u-blox UBX-RXM-RAWX and RTCM MSM7 raw formats are supported, and the input
format can be auto-detected.

**convobs** also introduces `obsj`, a convenient [JSON Lines](https://jsonlines.org/)
representation of observation data with RINEX-adjacent semantics, designed for
processing with modern tools such as **jq**. It is supported for both input and
output. Each line is one JSON object: a line with a `t` field is an observation,
and a line without one carries header metadata. The fields mirror RINEX
concepts — for example, `sat` and `sig` are RINEX satellite and signal
identifiers, and `arc` and `hc` correspond to RINEX loss-of-lock indicator bits.
For example:

```
{"t":"2025-12-17T08:14:06.0080000","sat":"G07","sig":"1C","pr":23956830.529584773,"cp":125893980.17237933,"do":2059.716796875,"cn0":34}
```

See the [**convobs**(1) man page](docs/convobs.1.md) for the full set of options,
the `obsj` field definitions, and the RINEX header metadata format.

A companion command, **diffobs**, compares two observation files (`obsj` or
RINEX) semantically; run `diffobs --help` for details.

There is also an implementation in Go that is part of
[SatPulse](https://satpulse.net/), exposed as the `satpulsetool convobs` command
and documented in its
[man page](https://satpulse.net/man/satpulsetool-convobs.1.html). This Rust
implementation runs about 5× faster.

## Building

With a [Rust toolchain](https://rustup.rs/) installed, run `make release`; the
`convobs` and `diffobs` binaries are written to `target/release`. Run `make` on
its own to list all targets, or `make install` to install the binaries into
`~/.cargo/bin`.

The project is a Cargo workspace of three crates:

- **`obsj`** — the core library: the `obsj` observation model and format, the
  carrier-phase arc / loss-of-lock logic, a semantic diff, and a self-contained
  RINEX 3.x reader/writer (the *internal* backend). Optional `rtcm` and `ubx`
  features add the raw-stream converters.
- **`rinex-obsj`** — a bridge to the third-party
  [`rinex`](https://crates.io/crates/rinex) crate (the *external* backend),
  which adds CRINEX (Hatanaka-compressed) input.
- **`convobs-cli`** — the `convobs` and `diffobs` binaries.

The external backend is optional, gated behind the CLI's `rinex-crate` feature.
The default build (`make release`) omits it for a lean, fully MIT-licensed
binary; `make release-full` (or `make install-full`) enables it, linking the
`rinex` crate and its MPL-2.0 dependencies and adding CRINEX support.

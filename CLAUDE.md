# CLAUDE.md

Guidance for Claude Code when working in this repository.

## Commit messages

- Do **not** add a `Co-Authored-By: Claude ...` trailer — or any AI co-author
  attribution — to commit messages.
- This repo sets `includeCoAuthoredBy: false` in `.claude/settings.json`. That
  option only suppresses the trailer Claude Code would append *automatically*;
  it does not stop a trailer that is hand-written into a `-m`/`-F` message. So
  never write the trailer into the message text yourself either.

## Interaction

- Do not present multiple-choice questionnaires (e.g. the AskUserQuestion
  picker). When you need a decision, ask in plain prose with a brief
  recommendation, and let the user answer in their own words.

## Working in this repository

Cargo workspace, three crates: `obsj` (core library — the obsj model/format,
arc/loss-of-lock logic, a semantic diff, and the self-contained "internal" RINEX
reader/writer), `rinex-obsj` (bridge to the external `rinex` crate), and
`convobs-cli` (the `convobs` and `diffobs` binaries).

Use cargo: `cargo build --release`, `cargo test --workspace`. Features:

- `obsj`: `rinexobs` (internal RINEX backend), `rtcm`, `ubx` (raw converters);
  the CLI enables all three.
- `convobs-cli/rinex-crate` (off by default) links `rinex-obsj` for the external
  RINEX backend with CRINEX (Hatanaka) input. CI tests with and without it and
  gates on `clippy -D warnings`.

`docs/convobs.1.md` is the behaviour spec; keep it in sync with CLI changes.

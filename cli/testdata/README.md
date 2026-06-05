# convobs golden test fixtures

These files back `TestGoldenFiles` in `../tests/golden.rs`. Each test case
converts a raw observation input with `convobs` (in-process, via
`convobs::run_to_writer`) and compares the result, semantically, against a
golden RINEX observation file produced by RTKLIB Explorer's `convbin`. The
comparison ignores metadata that depends on run time.

The golden files are therefore an independent reference (RTKLIB Explorer), not
convobs self-output. Regenerate them with `make` (see `Makefile`, which holds
the convbin flags for every case) only when the reference itself changes, and
review any resulting observation differences before committing. `cargo test`
never runs `convbin`; it only reads the committed `.obs.gz` goldens.

These fixtures were ported from the SatPulse repo
(`satpulse/internal/convobscmd/testdata`). The Unicore `um980-uncb` pair is out
of scope here — convobs-rs does not support the Unicore input format — so only
the UBX and RTCM pairs are committed.

## Tools

- `convbin` is the RTKLIB Explorer build, not the system `convbin`. Build it
  from `~/rtklib-ex` so it matches the `~/rtklib-ex/src` code the converters
  are checked against, and point `make` at it with
  `make CONVBIN=~/rtklib-ex/bin/convbin`. The committed golden files were
  generated with RTKLIB Explorer commit
  `89a735ba8ff5038b2b556b267913a617d7210dd4` (`CONVBIN EX 2.5.0`); regenerate
  from the same commit unless intentionally updating the reference.
- The raw input streams were extracted from packet logs (not committed here);
  each is a 15-minute slice, long enough to cover all signals and exercise the
  carrier-phase arc logic.

## Fixtures

Each input file is paired with a `.obs.gz` golden of the same base name.

### `m8t-20251217.ubx`, `f9t-20251217.ubx`

Raw UBX-RXM-RAWX captures from u-blox M8T and F9T receivers. The golden test
passes `--ubx-bds-geo-half-cycle` to match RTKLIB Explorer's BDS GEO half-cycle
phase correction.

### `packet-rtcm-20260519.rtcm`

RTCM 3 MSM7 stream. The golden test passes `--from rtcm --date-from-filename
--rtcm-omit-zero-do`.

### `um980-rtcm-20260527.rtcm`

RTCM 3 MSM7 stream from a Unicore UM980, with the same flags. Unlike the
2026-05-19 stream, this one carries an RTCM 1005 station-ID message, so it
exercises station-id metadata extraction (compared with `ignore_marker`).

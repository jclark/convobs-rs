//! Regression test for `raw` family auto-detection. A non-observation UBX frame
//! that precedes RTCM MSM7 observations must not capture the stream as the UBX
//! family and silently drop all the RTCM data. See `detect_raw_family` in
//! cli/src/lib.rs: only an observation frame (UBX RXM-RAWX or RTCM MSM7) may
//! select a family, matching SatPulse.

mod common;

use common::fixed_now;
use std::path::PathBuf;

fn testdata(name: &str) -> String {
    let mut p = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    p.push("testdata");
    p.push(name);
    p.to_string_lossy().into_owned()
}

#[test]
fn raw_skips_non_observation_ubx_before_rtcm() {
    // A complete, checksum-valid UBX NAV-PVT frame (class 0x01, id 0x07), empty
    // payload — a valid UBX frame, but not an observation. Its Fletcher-8
    // checksum over `01 07 00 00` is `08 19`.
    const UBX_NAV_PVT: &[u8] = &[0xB5, 0x62, 0x01, 0x07, 0x00, 0x00, 0x08, 0x19];

    // Prepend it to a real RTCM MSM7 stream (committed fixture, real CRCs).
    let rtcm = std::fs::read(testdata("um980-rtcm-20260527.rtcm")).unwrap();
    let mut mixed = UBX_NAV_PVT.to_vec();
    mixed.extend_from_slice(&rtcm);
    let mut mixed_path = PathBuf::from(env!("CARGO_TARGET_TMPDIR"));
    mixed_path.push("raw-detect-mixed.raw");
    std::fs::write(&mixed_path, &mixed).unwrap();

    // `--date` pins the RTCM week explicitly, so the test is independent of the
    // injected clock and emits no inferred-week warning.
    let args = [
        "--from".to_string(),
        "raw".to_string(),
        "--date".to_string(),
        "20260527".to_string(),
        "--to".to_string(),
        "obsj".to_string(),
        mixed_path.to_string_lossy().into_owned(),
    ];
    let mut out = Vec::new();
    convobs::run_to_writer(&args, &mut out, fixed_now())
        .expect("raw auto-detect should select RTCM, not the leading UBX frame");

    let (_meta, obs) = obsj::json::read_obsj(std::io::Cursor::new(out)).unwrap();
    assert!(
        !obs.is_empty(),
        "leading non-RAWX UBX captured the stream and dropped the RTCM observations"
    );
}

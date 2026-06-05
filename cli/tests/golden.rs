//! In-process golden test. Each case converts a committed raw fixture with
//! `convobs::run_to_writer` into an in-memory buffer, reads the resulting RINEX
//! back through the obsj model, and compares it — semantically, not byte-wise —
//! against a committed golden RINEX file produced by RTKLIB Explorer's `convbin`
//! (see `testdata/Makefile`). No subprocess, no `convbin`, no `tmp/` at test
//! time.
//!
//! The golden tolerances are pr/cp/do/cn0 at 5e-4 (RINEX's three-decimal text
//! precision), approxPos/antennaDelta at 5e-5. The two RTCM cases pass
//! `ignore_marker` because convbin records the RTCM station id as the marker
//! *name* while convobs records it as the *number*.
//!
//! Built with `--features convobs-cli/rinex-crate`, the suite drives the crate
//! RINEX backend instead of the default DIY backend, and ignores blank phase
//! (the crate cannot emit a phase field that exists only to carry a
//! loss-of-lock flag).

mod common;

use common::fixed_now;
use convobs::{ObsFormat, RinexBackend};
use obsj::diff::{diff_metadata, diff_observations, DiffRecord, MetadataTolerances, ObsTolerances};
use obsj::obs::Metadata;
use std::io::{Cursor, Write};
use std::path::PathBuf;

const OBS_TOL: ObsTolerances = ObsTolerances {
    pr: 5e-4,
    cp: 5e-4,
    dop: 5e-4,
    cn0: 5e-4,
};
const META_TOL: MetadataTolerances = MetadataTolerances {
    approx_pos: 5e-5,
    antenna_delta: 5e-5,
};

/// Whether this build links the crate RINEX backend. When it does, the golden
/// suite exercises that backend (and ignores blank phase, which it cannot
/// emit); otherwise it exercises the self-contained DIY backend at full
/// fidelity.
const USE_CRATE_BACKEND: bool = cfg!(feature = "rinex-crate");

struct Case {
    name: &'static str,
    /// convobs-side flags, before the input path. The golden's convbin flags
    /// live in `testdata/Makefile`.
    flags: &'static [&'static str],
    input: &'static str,
    golden: &'static str,
    ignore_marker: bool,
}

const CASES: &[Case] = &[
    Case {
        name: "m8t_20251217",
        flags: &["--ubx-bds-geo-half-cycle"],
        input: "m8t-20251217.ubx",
        golden: "m8t-20251217.obs.gz",
        ignore_marker: false,
    },
    Case {
        name: "f9t_20251217",
        flags: &["--ubx-bds-geo-half-cycle"],
        input: "f9t-20251217.ubx",
        golden: "f9t-20251217.obs.gz",
        ignore_marker: false,
    },
    Case {
        name: "rtcm_20260519",
        flags: &[
            "--from",
            "rtcm",
            "--date-from-filename",
            "--rtcm-omit-zero-do",
        ],
        input: "packet-rtcm-20260519.rtcm",
        golden: "packet-rtcm-20260519.obs.gz",
        ignore_marker: true,
    },
    Case {
        name: "um980_rtcm_20260527",
        flags: &[
            "--from",
            "rtcm",
            "--date-from-filename",
            "--rtcm-omit-zero-do",
        ],
        input: "um980-rtcm-20260527.rtcm",
        golden: "um980-rtcm-20260527.obs.gz",
        ignore_marker: true,
    },
];

fn testdata(name: &str) -> String {
    let mut p = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    p.push("testdata");
    p.push(name);
    p.to_string_lossy().into_owned()
}

/// The backend used to read both the produced RINEX and the golden. Both sides
/// must use the *same* backend the conversion wrote with: the crate writer and
/// crate reader are a matched pair (the DIY reader drops the GLONASS frequency
/// channel and antenna delta the crate writer encodes its own way, and can fail
/// to parse the crate's RTCM header). `None` is auto, which resolves to DIY for
/// the plain RINEX these tests use.
fn read_backend() -> Option<RinexBackend> {
    USE_CRATE_BACKEND.then_some(RinexBackend::Crate)
}

fn run_case(c: &Case) {
    // Convert the raw fixture to RINEX, in process, into a buffer.
    let mut args: Vec<String> = c.flags.iter().map(|s| s.to_string()).collect();
    if USE_CRATE_BACKEND {
        args.push("--rinex-backend".to_string());
        args.push("crate".to_string());
    }
    args.push(testdata(c.input));
    let mut out = Vec::new();
    convobs::run_to_writer(&args, &mut out, fixed_now())
        .unwrap_or_else(|e| panic!("{}: run_to_writer: {e}", c.name));

    // Read the produced RINEX (via a temp file) and the committed golden (which
    // auto-gunzips from content) back through the obsj model, both with the
    // backend that wrote the output.
    let mut got_path = PathBuf::from(env!("CARGO_TARGET_TMPDIR"));
    got_path.push(format!("{}-got.obs", c.name));
    std::fs::write(&got_path, out).unwrap();
    let backend = read_backend();
    let (got_meta, got_obs) =
        convobs::read_obs_file(got_path.to_str().unwrap(), ObsFormat::Rinex, backend)
            .unwrap_or_else(|e| panic!("{}: read produced RINEX: {e}", c.name));
    let (want_meta, want_obs) =
        convobs::read_obs_file(&testdata(c.golden), ObsFormat::Rinex, backend)
            .unwrap_or_else(|e| panic!("{}: read golden {}: {e}", c.name, c.golden));

    // Compare semantically. The crate backend drops blank phase, so ignore it
    // there.
    let (ma, mb) = diff_metadata(&got_meta, &want_meta, META_TOL, c.ignore_marker);
    let obs_diffs = diff_observations(&got_obs, &want_obs, OBS_TOL, USE_CRATE_BACKEND);

    let meta_differs = !ma.is_zero() || !mb.is_zero();
    if meta_differs || !obs_diffs.is_empty() {
        let path = write_diff(c.name, &ma, &mb, &obs_diffs);
        panic!(
            "{}: semantic diff: metadata_differs={} observation_records={}; full diff written to {}",
            c.name,
            meta_differs,
            obs_diffs.len(),
            path
        );
    }
}

/// Writes the differences to a JSONL artifact under the test temp dir and
/// returns its path, so a failing run points at the full diff.
fn write_diff(name: &str, ma: &Metadata, mb: &Metadata, diffs: &[DiffRecord]) -> String {
    let mut p = PathBuf::from(env!("CARGO_TARGET_TMPDIR"));
    p.push(format!("{name}-diff.jsonl"));
    let mut f = std::fs::File::create(&p).unwrap();
    if !ma.is_zero() || !mb.is_zero() {
        let v = serde_json::json!({ "metadata": { "a": ma, "b": mb } });
        writeln!(f, "{v}").unwrap();
    }
    for d in diffs {
        writeln!(f, "{}", diff_record_json(d)).unwrap();
    }
    p.to_string_lossy().into_owned()
}

fn diff_record_json(d: &DiffRecord) -> serde_json::Value {
    let side = |s: &Option<obsj::diff::SignalDiff>| {
        s.as_ref().map(|sd| {
            let mut v = serde_json::to_value(sd.v).unwrap();
            if sd.ll {
                v["ll"] = serde_json::json!(true);
            }
            v
        })
    };
    serde_json::json!({
        "t": d.t.to_string(),
        "sat": d.sat.as_str(),
        "sig": d.sig.as_str(),
        "a": side(&d.a),
        "b": side(&d.b),
    })
}

#[test]
fn golden_m8t_20251217() {
    run_case(&CASES[0]);
}

#[test]
fn golden_f9t_20251217() {
    run_case(&CASES[1]);
}

#[test]
fn golden_rtcm_20260519() {
    run_case(&CASES[2]);
}

#[test]
fn golden_um980_rtcm_20260527() {
    run_case(&CASES[3]);
}

/// Self-contained obsj coverage with no convbin golden: convert a committed UBX
/// fixture to obsj, read it back, convert again, and assert the observation
/// model is identical at exact f64 (the obsj path is the bit-exact path).
#[test]
fn obsj_path_round_trips() {
    let exact = ObsTolerances {
        pr: 0.0,
        cp: 0.0,
        dop: 0.0,
        cn0: 0.0,
    };

    // UBX -> obsj.
    let mut bytes1 = Vec::new();
    let args1 = vec![
        "--to".to_string(),
        "obsj".to_string(),
        testdata("m8t-20251217.ubx"),
    ];
    convobs::run_to_writer(&args1, &mut bytes1, fixed_now()).expect("ubx -> obsj");
    let (meta1, obs1) = obsj::json::read_obsj(Cursor::new(bytes1.clone())).expect("read obsj 1");
    assert!(!obs1.is_empty(), "ubx -> obsj produced no observations");

    // obsj -> obsj, through a temp file.
    let mut tmp = PathBuf::from(env!("CARGO_TARGET_TMPDIR"));
    tmp.push("m8t-roundtrip.obsj");
    std::fs::write(&tmp, &bytes1).unwrap();
    let mut buf2 = Vec::new();
    let args2 = vec![
        "--from".to_string(),
        "obsj".to_string(),
        "--to".to_string(),
        "obsj".to_string(),
        tmp.to_string_lossy().into_owned(),
    ];
    convobs::run_to_writer(&args2, &mut buf2, fixed_now()).expect("obsj -> obsj");
    let (meta2, obs2) = obsj::json::read_obsj(Cursor::new(buf2)).expect("read obsj 2");

    let diffs = diff_observations(&obs1, &obs2, exact, false);
    assert!(
        diffs.is_empty(),
        "obsj round-trip changed {} observation records",
        diffs.len()
    );
    let (ma, mb) = diff_metadata(&meta1, &meta2, META_TOL, false);
    assert!(
        ma.is_zero() && mb.is_zero(),
        "obsj round-trip changed metadata"
    );
}

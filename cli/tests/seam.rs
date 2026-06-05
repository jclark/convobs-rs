//! Smoke test for the `run_to_writer` seam (Step 1 of TESTING-PLAN): a parsed
//! conversion runs against a caller-supplied buffer and a fixed clock, with no
//! filesystem output and no wall-clock dependency — the foundation every
//! in-process golden test builds on.

mod common;

use common::{fixed_now, SharedBuf};
use std::fs;
use std::path::PathBuf;

fn tmp_path(name: &str) -> PathBuf {
    let mut p = PathBuf::from(env!("CARGO_TARGET_TMPDIR"));
    p.push(name);
    p
}

#[test]
fn run_to_writer_converts_obsj_into_a_buffer() {
    // A tiny obsj input: one metadata record then one observation.
    let input = concat!(
        "{\"marker\":{\"name\":\"SEAM\"}}\n",
        "{\"t\":\"2025-06-30T23:59:59.0000000\",\"sat\":\"G03\",\"sig\":\"1C\",",
        "\"pr\":22187868.655,\"cp\":116598092.035,\"cn0\":48}\n",
    );
    let path = tmp_path("seam-input.obsj");
    fs::write(&path, input).unwrap();

    let args = [
        "--from".to_string(),
        "obsj".to_string(),
        "--to".to_string(),
        "obsj".to_string(),
        path.to_string_lossy().into_owned(),
    ];
    let out = SharedBuf::new();
    convobs::run_to_writer(&args, Box::new(out.clone()), fixed_now()).expect("run_to_writer");

    // The provided writer received the conversion: the metadata and the
    // observation both survive the obsj -> obsj round trip.
    let (meta, obs) = obsj::json::read_obsj(std::io::Cursor::new(out.bytes())).unwrap();
    assert_eq!(meta.marker.name, "SEAM");
    assert_eq!(obs.len(), 1);
    assert_eq!(obs[0].sat.as_str(), "G03");
    assert_eq!(obs[0].sig.as_str(), "1C");
    assert_eq!(obs[0].v.pr, Some(22187868.655));
    assert_eq!(obs[0].v.cp, Some(116598092.035));
    // The injected clock is fully honoured: the run.date metadata default comes
    // from `now`, not the wall clock, so the conversion is deterministic.
    assert_eq!(meta.run.date, Some(fixed_now()));
}

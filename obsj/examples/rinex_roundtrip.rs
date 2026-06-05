//! Exercises the DIY RINEX backend against a real file: read it, write it back,
//! re-read it, and emit both reads as obsj so they can be compared with diffobs
//! (a stable round-trip yields zero differences). Reports how many blank-phase
//! observations — a pseudorange-only signal carrying only a loss-of-lock flag —
//! were read, the case the `rinex` crate cannot represent.
//!
//! Usage: cargo run -p obsj --features rinexobs --example rinex_roundtrip -- IN.obs OUT1.obsj OUT2.obsj

use obsj::obs::{Metadata, SignalObservation};
use obsj::rinexobs::{read_observation_file, write_observation_file};
use obsj::{ObsJsonSink, Sink};
use std::fs::File;
use std::io::{BufReader, BufWriter, Cursor};

fn emit_obsj(path: &str, obs: &[SignalObservation]) {
    let mut sink = ObsJsonSink::new(BufWriter::new(File::create(path).unwrap()));
    for o in obs {
        sink.observation(o).unwrap();
    }
    sink.flush().unwrap();
}

fn main() {
    let mut args = std::env::args().skip(1);
    let input = args.next().expect("usage: rinex_roundtrip IN.obs OUT1.obsj OUT2.obsj");
    let out1 = args.next().expect("missing OUT1.obsj");
    let out2 = args.next().expect("missing OUT2.obsj");

    let (meta, obs) = read_observation_file(BufReader::new(File::open(&input).unwrap()))
        .unwrap_or_else(|e| panic!("read {input}: {e}"));
    let blank = obs.iter().filter(|o| o.v.cp.is_none() && o.v.arc > 0).count();
    eprintln!("read {} observations ({} blank-phase with arc>0)", obs.len(), blank);
    emit_obsj(&out1, &obs);

    let mut meta2: Metadata = meta;
    let mut buf = Vec::new();
    write_observation_file(&mut buf, &mut meta2, &obs).unwrap();
    let (_, obs2) = read_observation_file(Cursor::new(buf)).unwrap();
    assert_eq!(obs.len(), obs2.len(), "round-trip changed the observation count");
    emit_obsj(&out2, &obs2);
    eprintln!("round-trip stable: {} observations", obs2.len());
}

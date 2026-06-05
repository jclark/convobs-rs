// Dump the rinex crate's JSON for one epoch, to compare with obsj.
use rinex::prelude::*;
use std::io::BufReader;

fn main() {
    let path = std::env::args().nth(1).unwrap();
    let f = std::fs::File::open(&path).unwrap();
    let mut br = BufReader::new(f);
    let rnx = Rinex::parse(&mut br).unwrap();
    // Serialize the whole thing, then also just one epoch's Observations.
    if let Some(rec) = rnx.record.as_obs() {
        let (k, v) = rec.iter().next().unwrap();
        println!("=== ObsKey (map key) serialized ===");
        println!("{}", serde_json::to_string(k).unwrap());
        println!("=== Observations (one epoch) serialized ===");
        println!("{}", serde_json::to_string_pretty(v).unwrap());
        println!("=== a few SignalObservation entries ===");
        for s in v.signals.iter().take(4) {
            println!("{}", serde_json::to_string(s).unwrap());
        }
    }
    println!("=== Header serialized (truncated) ===");
    let hj = serde_json::to_string_pretty(&rnx.header).unwrap();
    for l in hj.lines().take(40) { println!("{}", l); }
}

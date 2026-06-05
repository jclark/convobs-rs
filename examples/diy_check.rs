// Read the golden via the DIY reader; round-trip it; show the blank-phase case.
use convobs::rinexobs::{read_observation_file, write_observation_file};
use std::io::BufReader;

fn main() {
    let path = std::env::args().nth(1).unwrap();
    let f = std::fs::File::open(&path).unwrap();
    let (mut meta, obs) = read_observation_file(BufReader::new(f)).unwrap();
    println!("read {} observations", obs.len());
    // The C08 2I blank-phase case at 13:31:24:
    for o in &obs {
        if o.sat.as_str() == "C08" && o.sig.as_str() == "2I" {
            println!("C08 2I @ {}: pr={:?} cp={:?} do={:?} cn0={:?} arc={}",
                o.t, o.v.pr, o.v.cp, o.v.dop, o.v.cn0, o.v.arc);
            break;
        }
    }
    // Round-trip: write back out, re-read, confirm the count is stable.
    let mut buf = Vec::new();
    write_observation_file(&mut buf, &mut meta, &obs).unwrap();
    let (_m2, obs2) = read_observation_file(BufReader::new(&buf[..])).unwrap();
    println!("round-trip re-read {} observations (stable: {})", obs2.len(), obs2.len() == obs.len());
}

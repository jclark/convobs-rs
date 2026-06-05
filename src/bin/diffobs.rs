//! diffobs: compares two observation files (`.obs[.gz]` or `.obsj`) into the
//! midpoint and reports semantic differences.
//! Reads `.obs[.gz]` and `.obsj`, and reports semantic differences.
//!
//! Exit codes: 0 identical, 1 differences, 2 error.

use convobs::diff::{diff_metadata, diff_observations, MetadataTolerances, ObsTolerances, SignalDiff};
use convobs::obs::{Metadata, SignalObservation};
use convobs::obsj::read_obsj;
use convobs::rinexio::read_observation_file;
use std::fs::File;
use std::io::{self, BufReader, Write};
use std::process::ExitCode;

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    match run(&args) {
        Ok(code) => code,
        Err(e) => {
            eprintln!("diffobs: {}", e);
            ExitCode::from(2)
        }
    }
}

fn run(args: &[String]) -> Result<ExitCode, String> {
    let mut pr = None;
    let mut cp = None;
    let mut dop = None;
    let mut cn0 = None;
    let mut approx_pos = 0.00005;
    let mut antenna_delta = 0.00005;
    let mut files: Vec<&str> = Vec::new();

    let mut i = 0;
    while i < args.len() {
        let a = args[i].clone();
        let key = a.trim_start_matches('-').to_string();
        let mut take_val = |i: &mut usize| -> Result<f64, String> {
            *i += 1;
            args.get(*i)
                .ok_or_else(|| format!("missing value for {}", a))?
                .parse()
                .map_err(|_| format!("invalid number for {}", a))
        };
        match key.as_str() {
            "pr-tol" => pr = Some(take_val(&mut i)?),
            "cp-tol" => cp = Some(take_val(&mut i)?),
            "do-tol" => dop = Some(take_val(&mut i)?),
            "cn0-tol" => cn0 = Some(take_val(&mut i)?),
            "approx-pos-tol" => approx_pos = take_val(&mut i)?,
            "antenna-delta-tol" => antenna_delta = take_val(&mut i)?,
            _ => files.push(Box::leak(a.into_boxed_str())),
        }
        i += 1;
    }
    if files.len() != 2 {
        return Err("usage: diffobs [options] a.obs[.gz]|a.obsj b....".to_string());
    }

    // Default tolerance: exact (0) when both inputs are obsj, else 5e-4.
    let both_obsj = files.iter().all(|f| f.ends_with(".obsj"));
    let default_tol = if both_obsj { 0.0 } else { 0.0005 };
    let tol = ObsTolerances {
        pr: pr.unwrap_or(default_tol),
        cp: cp.unwrap_or(default_tol),
        dop: dop.unwrap_or(default_tol),
        cn0: cn0.unwrap_or(default_tol),
    };
    let mtol = MetadataTolerances {
        approx_pos,
        antenna_delta,
    };

    let (a_meta, a_obs) = read_file(files[0]).map_err(|e| format!("{}: {}", files[0], e))?;
    let (b_meta, b_obs) = read_file(files[1]).map_err(|e| format!("{}: {}", files[1], e))?;

    let stdout = io::stdout();
    let mut out = io::BufWriter::new(stdout.lock());
    let mut n = 0u64;

    let (a_only, b_only) = diff_metadata(&a_meta, &b_meta, mtol);
    if !a_only.is_zero() || !b_only.is_zero() {
        out.write_all(b"{\"metadata\":true}\n").map_err(|e| e.to_string())?;
        n += 1;
    }

    for d in diff_observations(&a_obs, &b_obs, tol) {
        let mut line = String::new();
        line.push_str("{\"t\":\"");
        line.push_str(&d.t.to_string());
        line.push_str("\",\"sat\":\"");
        line.push_str(d.sat.as_str());
        line.push_str("\",\"sig\":\"");
        line.push_str(d.sig.as_str());
        line.push('"');
        if let Some(a) = d.a {
            line.push_str(",\"a\":");
            append_diff(&mut line, &a);
        }
        if let Some(b) = d.b {
            line.push_str(",\"b\":");
            append_diff(&mut line, &b);
        }
        line.push_str("}\n");
        out.write_all(line.as_bytes()).map_err(|e| e.to_string())?;
        n += 1;
    }

    Ok(if n != 0 {
        ExitCode::from(1)
    } else {
        ExitCode::SUCCESS
    })
}

fn append_diff(out: &mut String, d: &SignalDiff) {
    use std::fmt::Write;
    out.push('{');
    let mut first = true;
    let mut comma = |out: &mut String, first: &mut bool| {
        if !*first {
            out.push(',');
        }
        *first = false;
    };
    if let Some(v) = d.v.frq {
        comma(out, &mut first);
        let _ = write!(out, "\"frq\":{}", v);
    }
    if let Some(v) = d.v.pr {
        comma(out, &mut first);
        let _ = write!(out, "\"pr\":{}", v);
    }
    if let Some(v) = d.v.cp {
        comma(out, &mut first);
        let _ = write!(out, "\"cp\":{}", v);
    }
    if let Some(v) = d.v.dop {
        comma(out, &mut first);
        let _ = write!(out, "\"do\":{}", v);
    }
    if let Some(v) = d.v.cn0 {
        comma(out, &mut first);
        let _ = write!(out, "\"cn0\":{}", v);
    }
    if d.v.hc {
        comma(out, &mut first);
        out.push_str("\"hc\":true");
    }
    if d.v.bt {
        comma(out, &mut first);
        out.push_str("\"bt\":true");
    }
    if d.ll {
        comma(out, &mut first);
        out.push_str("\"ll\":true");
    }
    out.push('}');
}

fn read_file(path: &str) -> Result<(Metadata, Vec<SignalObservation>), String> {
    let f = File::open(path).map_err(|e| e.to_string())?;
    if path.ends_with(".obsj") {
        read_obsj(BufReader::new(f))
    } else if path.ends_with(".gz") {
        let gz = flate2::read::GzDecoder::new(f);
        read_observation_file(BufReader::new(gz))
    } else {
        read_observation_file(BufReader::new(f))
    }
}

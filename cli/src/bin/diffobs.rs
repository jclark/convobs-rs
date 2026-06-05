//! diffobs: compares two observation files (obsj or RINEX) semantically.
//!
//! Each input's format is given by an explicit option — never inferred from the
//! filename; only gzip compression is detected from content. Tolerances default
//! to exact f64 when both inputs are obsj, else 5e-4 (RINEX text precision).
//!
//! Exit codes: 0 identical, 1 differences, 2 error.

use convobs::error::Error;
use convobs::{read_obs_file, ObsFormat};
use obsj::diff::{diff_metadata, diff_observations, MetadataTolerances, ObsTolerances, SignalDiff};
use std::io::{self, Write};
use std::process::ExitCode;

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    match run(&args) {
        Ok(code) => code,
        Err(e) => {
            eprintln!("diffobs: {e}");
            ExitCode::from(2)
        }
    }
}

const USAGE: &str = "usage: diffobs [options] a b\n\
  --format FMT             obsj|rinex for both inputs (default obsj)\n\
  --a-format FMT           format of the first input\n\
  --b-format FMT           format of the second input\n\
  --rinex-backend BACKEND  diy|crate|auto for RINEX inputs (default auto)\n\
  --ignore-blank-phase     skip cp/ll where one side has no carrier phase\n\
  --ignore-marker          ignore marker metadata fields\n\
  --pr-tol --cp-tol --do-tol --cn0-tol --approx-pos-tol --antenna-delta-tol N\n";

fn run(args: &[String]) -> Result<ExitCode, Error> {
    let mut pr = None;
    let mut cp = None;
    let mut dop = None;
    let mut cn0 = None;
    let mut approx_pos = 0.00005;
    let mut antenna_delta = 0.00005;
    let mut ignore_blank_phase = false;
    let mut ignore_marker = false;
    let mut both_fmt = None;
    let mut a_fmt = None;
    let mut b_fmt = None;
    let mut backend_arg = "auto".to_string();
    let mut files: Vec<String> = Vec::new();

    let mut i = 0;
    while i < args.len() {
        let a = &args[i];
        let key = a.trim_start_matches('-');
        let val = |i: &mut usize| -> Result<String, Error> {
            *i += 1;
            args.get(*i)
                .cloned()
                .ok_or_else(|| Error::usage(format!("missing value for {a}")))
        };
        let num = |s: String, a: &str| -> Result<f64, Error> {
            s.parse().map_err(|_| Error::usage(format!("invalid number for {a}")))
        };
        match key {
            "h" | "help" => {
                print!("{USAGE}");
                return Ok(ExitCode::SUCCESS);
            }
            "pr-tol" => pr = Some(num(val(&mut i)?, a)?),
            "cp-tol" => cp = Some(num(val(&mut i)?, a)?),
            "do-tol" => dop = Some(num(val(&mut i)?, a)?),
            "cn0-tol" => cn0 = Some(num(val(&mut i)?, a)?),
            "approx-pos-tol" => approx_pos = num(val(&mut i)?, a)?,
            "antenna-delta-tol" => antenna_delta = num(val(&mut i)?, a)?,
            "format" => both_fmt = Some(val(&mut i)?),
            "a-format" => a_fmt = Some(val(&mut i)?),
            "b-format" => b_fmt = Some(val(&mut i)?),
            "rinex-backend" => backend_arg = val(&mut i)?,
            "ignore-blank-phase" => ignore_blank_phase = true,
            "ignore-marker" => ignore_marker = true,
            _ if a.starts_with('-') && a != "-" => {
                return Err(Error::usage(format!("unknown option {a}")))
            }
            _ => files.push(a.clone()),
        }
        i += 1;
    }
    if files.len() != 2 {
        return Err(Error::usage("expected exactly two input files".to_string()));
    }

    let a_format = parse_format(a_fmt.or(both_fmt.clone()))?;
    let b_format = parse_format(b_fmt.or(both_fmt))?;
    let backend = convobs::parse_rinex_backend(&backend_arg).map_err(Error::Usage)?;

    // Exact f64 for obsj on both sides; the looser RINEX text precision otherwise.
    let default_tol = if a_format == ObsFormat::Obsj && b_format == ObsFormat::Obsj {
        0.0
    } else {
        0.0005
    };
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

    let (a_meta, a_obs) = read_obs_file(&files[0], a_format, backend)?;
    let (b_meta, b_obs) = read_obs_file(&files[1], b_format, backend)?;

    let stdout = io::stdout();
    let mut out = io::BufWriter::new(stdout.lock());
    let mut n = 0u64;

    let (a_only, b_only) = diff_metadata(&a_meta, &b_meta, mtol, ignore_marker);
    if !a_only.is_zero() || !b_only.is_zero() {
        out.write_all(b"{\"metadata\":true}\n").map_err(write_err)?;
        n += 1;
    }

    for d in diff_observations(&a_obs, &b_obs, tol, ignore_blank_phase) {
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
        out.write_all(line.as_bytes()).map_err(write_err)?;
        n += 1;
    }

    Ok(if n != 0 {
        ExitCode::from(1)
    } else {
        ExitCode::SUCCESS
    })
}

fn write_err(e: io::Error) -> Error {
    Error::io("", e)
}

fn parse_format(s: Option<String>) -> Result<ObsFormat, Error> {
    match s.as_deref().unwrap_or("obsj").to_lowercase().as_str() {
        "obsj" => Ok(ObsFormat::Obsj),
        "rinex" => Ok(ObsFormat::Rinex),
        other => Err(Error::usage(format!(
            "unsupported format {other:?} (expected obsj or rinex)"
        ))),
    }
}

fn append_diff(out: &mut String, d: &SignalDiff) {
    use std::fmt::Write;
    out.push('{');
    let mut first = true;
    let comma = |out: &mut String, first: &mut bool| {
        if !*first {
            out.push(',');
        }
        *first = false;
    };
    if let Some(v) = d.v.frq {
        comma(out, &mut first);
        let _ = write!(out, "\"frq\":{v}");
    }
    if let Some(v) = d.v.pr {
        comma(out, &mut first);
        let _ = write!(out, "\"pr\":{v}");
    }
    if let Some(v) = d.v.cp {
        comma(out, &mut first);
        let _ = write!(out, "\"cp\":{v}");
    }
    if let Some(v) = d.v.dop {
        comma(out, &mut first);
        let _ = write!(out, "\"do\":{v}");
    }
    if let Some(v) = d.v.cn0 {
        comma(out, &mut first);
        let _ = write!(out, "\"cn0\":{v}");
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

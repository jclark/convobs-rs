//! RINEX backend selection.
//!
//! The self-contained DIY backend (in the `obsj` crate) is the default and
//! handles plain RINEX 3.x observation files. The `rinex`-crate bridge backend
//! is compiled in only behind the `rinex-crate` feature and is used when asked
//! for explicitly or when a job needs a capability the DIY backend lacks — CRINEX
//! (Hatanaka) input, detected from content. In a lean build, asking for the crate
//! backend (or feeding CRINEX) fails cleanly.

use obsj::obs::{Metadata, SignalObservation};
use obsj::sink::Sink;
use std::io::{BufRead, Write};

#[derive(Clone, Copy, PartialEq)]
pub enum RinexBackend {
    Diy,
    Crate,
}

const NOT_COMPILED: &str =
    "the crate RINEX backend is not compiled in; rebuild with --features rinex-crate";

const fn crate_available() -> bool {
    cfg!(feature = "rinex-crate")
}

/// Parses the `--rinex-backend` value. `auto` (the default) returns `None`.
pub fn parse_backend(s: &str) -> Result<Option<RinexBackend>, String> {
    match s.to_lowercase().as_str() {
        "auto" => Ok(None),
        "diy" => Ok(Some(RinexBackend::Diy)),
        "crate" => Ok(Some(RinexBackend::Crate)),
        other => Err(format!(
            "unsupported RINEX backend {other:?} (expected diy, crate, or auto)"
        )),
    }
}

/// Resolves the backend for a RINEX input by peeking its content for CRINEX,
/// returning the (unconsumed) reader alongside the chosen backend.
pub fn open_rinex_input(
    explicit: Option<RinexBackend>,
    mut r: Box<dyn BufRead>,
) -> Result<(RinexBackend, Box<dyn BufRead>), String> {
    let crinex = is_crinex(r.fill_buf().map_err(|e| e.to_string())?);
    Ok((resolve_input(explicit, crinex)?, r))
}

fn resolve_input(explicit: Option<RinexBackend>, crinex: bool) -> Result<RinexBackend, String> {
    match explicit {
        Some(RinexBackend::Diy) if crinex => {
            Err("CRINEX input needs the crate RINEX backend; do not pass --rinex-backend diy".into())
        }
        Some(RinexBackend::Crate) if !crate_available() => Err(NOT_COMPILED.into()),
        Some(b) => Ok(b),
        None if crinex && !crate_available() => {
            Err("CRINEX input needs the crate RINEX backend; rebuild with --features rinex-crate".into())
        }
        None if crinex => Ok(RinexBackend::Crate),
        None => Ok(RinexBackend::Diy),
    }
}

/// A CRINEX (Hatanaka) file begins with a `CRINEX VERS / TYPE` header line.
fn is_crinex(head: &[u8]) -> bool {
    let line_end = head.iter().position(|&b| b == b'\n').unwrap_or(head.len());
    head[..line_end].windows(11).any(|w| w == b"CRINEX VERS")
}

/// Reads a RINEX observation input into the obsj model with the given backend.
pub fn read_rinex(
    backend: RinexBackend,
    r: Box<dyn BufRead>,
) -> Result<(Metadata, Vec<SignalObservation>), String> {
    match backend {
        RinexBackend::Diy => obsj::rinexobs::read_observation_file(r),
        RinexBackend::Crate => read_crate(r),
    }
}

/// Builds a RINEX output sink for the given backend (output is never CRINEX).
pub fn rinex_sink(backend: RinexBackend, w: Box<dyn Write>) -> Result<Box<dyn Sink>, String> {
    match backend {
        RinexBackend::Diy => Ok(Box::new(obsj::rinexobs::RinexSink::new(w))),
        RinexBackend::Crate => sink_crate(w),
    }
}

#[cfg(feature = "rinex-crate")]
fn read_crate(r: Box<dyn BufRead>) -> Result<(Metadata, Vec<SignalObservation>), String> {
    rinex_obsj::read_observation_file(r)
}

#[cfg(not(feature = "rinex-crate"))]
fn read_crate(_r: Box<dyn BufRead>) -> Result<(Metadata, Vec<SignalObservation>), String> {
    Err(NOT_COMPILED.into())
}

#[cfg(feature = "rinex-crate")]
fn sink_crate(w: Box<dyn Write>) -> Result<Box<dyn Sink>, String> {
    Ok(Box::new(rinex_obsj::RinexSink::new(w)))
}

#[cfg(not(feature = "rinex-crate"))]
fn sink_crate(_w: Box<dyn Write>) -> Result<Box<dyn Sink>, String> {
    Err(NOT_COMPILED.into())
}

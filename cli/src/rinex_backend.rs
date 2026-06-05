//! RINEX backend selection.
//!
//! The self-contained internal backend (in the `obsj` crate) is the default and
//! handles plain RINEX 3.x observation files. The external `rinex`-crate bridge
//! backend is compiled in only behind the `rinex-crate` feature and is used when
//! asked for explicitly or when a job needs a capability the internal backend
//! lacks — CRINEX (Hatanaka) input, detected from content. In a lean build,
//! asking for the external backend (or feeding CRINEX) fails cleanly.

use obsj::error::Error;
use obsj::obs::{Metadata, SignalObservation};
use obsj::sink::Sink;
use std::io::{BufRead, Write};

#[derive(Clone, Copy, PartialEq)]
pub enum RinexBackend {
    Internal,
    External,
}

const NOT_COMPILED: &str =
    "the external RINEX backend is not compiled in; rebuild with --features rinex-crate";

const fn external_available() -> bool {
    cfg!(feature = "rinex-crate")
}

/// Parses the `--rinex-backend` value. `auto` (the default) returns `None`.
pub fn parse_backend(s: &str) -> Result<Option<RinexBackend>, String> {
    match s.to_lowercase().as_str() {
        "auto" => Ok(None),
        "internal" => Ok(Some(RinexBackend::Internal)),
        "external" => Ok(Some(RinexBackend::External)),
        other => Err(format!(
            "unsupported RINEX backend {other:?} (expected internal, external, or auto)"
        )),
    }
}

/// Resolves the backend for a RINEX input by peeking its content for CRINEX,
/// returning the (unconsumed) reader alongside the chosen backend.
pub fn open_rinex_input(
    explicit: Option<RinexBackend>,
    mut r: Box<dyn BufRead>,
) -> Result<(RinexBackend, Box<dyn BufRead>), Error> {
    let crinex = is_crinex(r.fill_buf()?);
    Ok((resolve_input(explicit, crinex)?, r))
}

fn resolve_input(explicit: Option<RinexBackend>, crinex: bool) -> Result<RinexBackend, Error> {
    match explicit {
        Some(RinexBackend::Internal) if crinex => Err(Error::Rinex(
            "CRINEX input needs the external RINEX backend; do not pass --rinex-backend internal"
                .into(),
        )),
        Some(RinexBackend::External) if !external_available() => {
            Err(Error::Rinex(NOT_COMPILED.into()))
        }
        Some(b) => Ok(b),
        None if crinex && !external_available() => Err(Error::Rinex(
            "CRINEX input needs the external RINEX backend; rebuild with --features rinex-crate"
                .into(),
        )),
        None if crinex => Ok(RinexBackend::External),
        None => Ok(RinexBackend::Internal),
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
) -> Result<(Metadata, Vec<SignalObservation>), Error> {
    match backend {
        RinexBackend::Internal => obsj::rinexobs::read_observation_file(r),
        RinexBackend::External => read_external(r),
    }
}

/// Builds a RINEX output sink for the given backend (output is never CRINEX).
pub fn rinex_sink<'a>(
    backend: RinexBackend,
    w: Box<dyn Write + 'a>,
) -> Result<Box<dyn Sink + 'a>, Error> {
    match backend {
        RinexBackend::Internal => Ok(Box::new(obsj::rinexobs::RinexSink::new(w))),
        RinexBackend::External => sink_external(w),
    }
}

#[cfg(feature = "rinex-crate")]
fn read_external(r: Box<dyn BufRead>) -> Result<(Metadata, Vec<SignalObservation>), Error> {
    rinex_obsj::read_observation_file(r)
}

#[cfg(not(feature = "rinex-crate"))]
fn read_external(_r: Box<dyn BufRead>) -> Result<(Metadata, Vec<SignalObservation>), Error> {
    Err(Error::Rinex(NOT_COMPILED.into()))
}

#[cfg(feature = "rinex-crate")]
fn sink_external<'a>(w: Box<dyn Write + 'a>) -> Result<Box<dyn Sink + 'a>, Error> {
    Ok(Box::new(rinex_obsj::RinexSink::new(w)))
}

#[cfg(not(feature = "rinex-crate"))]
fn sink_external<'a>(_w: Box<dyn Write + 'a>) -> Result<Box<dyn Sink + 'a>, Error> {
    Err(Error::Rinex(NOT_COMPILED.into()))
}

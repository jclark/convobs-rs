//! obsj (JSON-lines) read/write.
//!
//! Records serialize with serde: floats use `ryu` shortest form, which
//! round-trips bit-exactly, and obsj is validated by diffobs at exact-f64, so
//! no particular byte layout is required. The reader dispatches on a top-level
//! `t` (observation vs metadata record) and rejects the legacy keys
//! `ssi`/`lli`/`ll`.

use crate::obs::{
    Antenna, GpsTime, Instant, Marker, Metadata, MetadataRun, Receiver, SatId, SigId,
    SignalObservation, SignalValues,
};
use crate::sink::Sink;
use serde::de::IgnoredAny;
use serde::Deserialize;
use serde_json::value::RawValue;
use std::io::{self, BufRead, Write};

/// Streaming obsj sink: one JSON line per record, O(1) memory.
pub struct ObsJsonSink<W: Write> {
    w: W,
}

impl<W: Write> ObsJsonSink<W> {
    pub fn new(w: W) -> Self {
        ObsJsonSink { w }
    }

    fn write_record<T: serde::Serialize>(&mut self, record: &T) -> io::Result<()> {
        serde_json::to_writer(&mut self.w, record).map_err(io::Error::from)?;
        self.w.write_all(b"\n")
    }
}

impl<W: Write> Sink for ObsJsonSink<W> {
    fn metadata(&mut self, m: &Metadata) -> io::Result<()> {
        self.write_record(m)
    }
    fn observation(&mut self, o: &SignalObservation) -> io::Result<()> {
        self.write_record(o)
    }
    fn flush(&mut self) -> io::Result<()> {
        self.w.flush()
    }
}

/// Streams obsj records from `r` into `sink` — O(1) memory: each line is parsed
/// and pushed (metadata or observation) without buffering the whole file.
pub fn stream_obsj<S: Sink>(r: impl BufRead, sink: &mut S) -> Result<(), String> {
    for (i, line) in r.lines().enumerate() {
        let line = line.map_err(|e| e.to_string())?;
        let trimmed = line.trim();
        if trimmed.is_empty() {
            continue;
        }
        match parse_record(trimmed).map_err(|e| format!("obsj line {}: {}", i + 1, e))? {
            Record::Observation(o) => sink.observation(&o).map_err(|e| e.to_string())?,
            Record::Metadata(m) => sink.metadata(&m).map_err(|e| e.to_string())?,
        }
    }
    Ok(())
}

/// Reads all obsj records into memory, merging metadata and collecting
/// observations. Used where random access is needed (the diff comparator);
/// conversion uses [`stream_obsj`] instead.
pub fn read_obsj(r: impl BufRead) -> Result<(Metadata, Vec<SignalObservation>), String> {
    #[derive(Default)]
    struct Collector {
        meta: Metadata,
        obs: Vec<SignalObservation>,
    }
    impl Sink for Collector {
        fn metadata(&mut self, m: &Metadata) -> io::Result<()> {
            self.meta.merge(m);
            Ok(())
        }
        fn observation(&mut self, o: &SignalObservation) -> io::Result<()> {
            self.obs.push(*o);
            Ok(())
        }
        fn flush(&mut self) -> io::Result<()> {
            Ok(())
        }
    }
    let mut c = Collector::default();
    stream_obsj(r, &mut c)?;
    Ok((c.meta, c.obs))
}

enum Record {
    Observation(SignalObservation),
    Metadata(Metadata),
}

/// One obsj line, in a single serde pass. A flat struct (no `Value`
/// intermediate, no `#[serde(flatten)]`) so deserialization is one pass — the
/// `flatten` on the wire type is what forces a buffering pass on read. An
/// observation record carries `t`; a metadata record does not. Observation
/// floats are captured as raw JSON tokens and rounded with std `f64::from_str`
/// (correctly rounded) so they round-trip bit-exactly. The legacy keys are
/// captured so they can be rejected.
#[derive(Deserialize)]
struct RawRecord<'a> {
    t: Option<GpsTime>,
    sat: Option<SatId>,
    sig: Option<SigId>,
    frq: Option<i8>,
    #[serde(borrow)]
    pr: Option<&'a RawValue>,
    cp: Option<&'a RawValue>,
    #[serde(rename = "do")]
    dop: Option<&'a RawValue>,
    cn0: Option<&'a RawValue>,
    arc: Option<u32>,
    hc: Option<bool>,
    bt: Option<bool>,
    version: Option<String>,
    run: Option<MetadataRun>,
    comment: Option<Vec<String>>,
    marker: Option<Marker>,
    observer: Option<String>,
    agency: Option<String>,
    receiver: Option<Receiver>,
    antenna: Option<Antenna>,
    #[serde(rename = "approxPosition")]
    approx_position: Option<[f64; 3]>,
    #[serde(rename = "antennaDelta")]
    antenna_delta: Option<[f64; 3]>,
    interval: Option<f64>,
    #[serde(rename = "leapSeconds")]
    leap_seconds: Option<i16>,
    ssi: Option<IgnoredAny>,
    lli: Option<IgnoredAny>,
    ll: Option<IgnoredAny>,
}

/// Parses a raw JSON number token with std's correctly-rounded float parser.
fn token_f64(v: Option<&RawValue>) -> Result<Option<f64>, String> {
    match v {
        Some(raw) => raw
            .get()
            .parse::<f64>()
            .map(Some)
            .map_err(|_| format!("invalid number {:?}", raw.get())),
        None => Ok(None),
    }
}

fn token_f32(v: Option<&RawValue>) -> Result<Option<f32>, String> {
    match v {
        Some(raw) => raw
            .get()
            .parse::<f32>()
            .map(Some)
            .map_err(|_| format!("invalid number {:?}", raw.get())),
        None => Ok(None),
    }
}

fn parse_record(line: &str) -> Result<Record, String> {
    let r: RawRecord = serde_json::from_str(line).map_err(|e| e.to_string())?;
    match r.t {
        Some(t) => {
            if r.ssi.is_some() {
                return Err("obsj field \"ssi\" is not supported".to_string());
            }
            if r.lli.is_some() {
                return Err("obsj field \"lli\" is not supported".to_string());
            }
            if r.ll.is_some() {
                return Err("obsj field \"ll\" is not supported; use \"arc\"".to_string());
            }
            let sat = r.sat.ok_or("obsj observation record missing \"sat\"")?;
            let sig = r.sig.ok_or("obsj observation record missing \"sig\"")?;
            let v = SignalValues {
                frq: r.frq,
                pr: token_f64(r.pr)?,
                cp: token_f64(r.cp)?,
                dop: token_f64(r.dop)?,
                cn0: token_f32(r.cn0)?,
                arc: r.arc.unwrap_or(0),
                hc: r.hc.unwrap_or(false),
                bt: r.bt.unwrap_or(false),
                ll: false,
            };
            Ok(Record::Observation(SignalObservation { t, sat, sig, v }))
        }
        None => {
            let mut m = Metadata::default();
            if let Some(x) = r.version {
                m.version = x;
            }
            if let Some(x) = r.run {
                m.run = x;
            }
            if let Some(x) = r.comment {
                m.comment = x;
            }
            if let Some(x) = r.marker {
                m.marker = x;
            }
            if let Some(x) = r.observer {
                m.observer = x;
            }
            if let Some(x) = r.agency {
                m.agency = x;
            }
            if let Some(x) = r.receiver {
                m.receiver = x;
            }
            if let Some(x) = r.antenna {
                m.antenna = x;
            }
            m.approx_position = r.approx_position;
            m.antenna_delta = r.antenna_delta;
            m.interval = r.interval;
            m.leap_seconds = r.leap_seconds;
            Ok(Record::Metadata(m))
        }
    }
}

/// Parses an RFC3339 timestamp (packet-log `t`, metadata `run.date`).
pub fn parse_rfc3339_public(s: &str) -> Option<Instant> {
    let (date, rest) = s.split_once('T')?;
    let mut dmy = date.splitn(3, '-');
    let year: i64 = dmy.next()?.parse().ok()?;
    let month: u32 = dmy.next()?.parse().ok()?;
    let day: u32 = dmy.next()?.parse().ok()?;

    let (hms_frac, off_secs) = split_zone(rest)?;
    let (hms, frac) = hms_frac.split_once('.').unwrap_or((hms_frac, ""));
    let mut hms_parts = hms.splitn(3, ':');
    let hour: u32 = hms_parts.next()?.parse().ok()?;
    let minute: u32 = hms_parts.next()?.parse().ok()?;
    let second: u32 = hms_parts.next()?.parse().ok()?;

    let nanos = if frac.is_empty() {
        0
    } else {
        let mut padded = frac.to_string();
        padded.truncate(9);
        while padded.len() < 9 {
            padded.push('0');
        }
        padded.parse().ok()?
    };

    let mut inst = Instant::from_civil(crate::obs::Civil {
        year,
        month,
        day,
        hour,
        minute,
        second,
        nanos,
    });
    inst.secs -= off_secs;
    Some(inst)
}

/// Splits an RFC3339 time-of-day from its zone, returning the offset in seconds.
fn split_zone(s: &str) -> Option<(&str, i64)> {
    if let Some(stripped) = s.strip_suffix('Z') {
        return Some((stripped, 0));
    }
    let bytes = s.as_bytes();
    for (i, &b) in bytes.iter().enumerate().rev() {
        if b == b'+' || b == b'-' {
            let sign = if b == b'+' { 1 } else { -1 };
            let (h, m) = s[i + 1..].split_once(':')?;
            let h: i64 = h.parse().ok()?;
            let m: i64 = m.parse().ok()?;
            return Some((&s[..i], sign * (h * 3600 + m * 60)));
        }
    }
    Some((s, 0))
}

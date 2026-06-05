//! obsj (JSON-lines) read/write.
//!
//! Records serialize with serde: floats use `ryu` shortest form, which
//! round-trips bit-exactly, and obsj is validated by diffobs at exact-f64, so
//! no particular byte layout is required. The reader dispatches on a top-level
//! `t` (observation vs metadata record) and rejects the legacy keys
//! `ssi`/`lli`/`ll`.

use crate::obs::{Instant, Metadata, SignalObservation};
use crate::sink::Sink;
use serde_json::Value;
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

/// Reads all obsj records from `r`, merging metadata and collecting observations.
pub fn read_obsj(r: impl BufRead) -> Result<(Metadata, Vec<SignalObservation>), String> {
    let mut meta = Metadata::default();
    let mut obs = Vec::new();
    for (i, line) in r.lines().enumerate() {
        let line = line.map_err(|e| e.to_string())?;
        let trimmed = line.trim();
        if trimmed.is_empty() {
            continue;
        }
        match parse_record(trimmed).map_err(|e| format!("obsj line {}: {}", i + 1, e))? {
            Record::Observation(o) => obs.push(o),
            Record::Metadata(m) => meta.merge(&m),
        }
    }
    Ok((meta, obs))
}

enum Record {
    Observation(SignalObservation),
    Metadata(Metadata),
}

fn parse_record(line: &str) -> Result<Record, String> {
    let value: Value = serde_json::from_str(line).map_err(|e| e.to_string())?;
    let obj = value.as_object().ok_or("record is not a JSON object")?;
    if obj.contains_key("t") {
        for legacy in ["ssi", "lli"] {
            if obj.contains_key(legacy) {
                return Err(format!("obsj field {legacy:?} is not supported"));
            }
        }
        if obj.contains_key("ll") {
            return Err("obsj field \"ll\" is not supported; use \"arc\"".to_string());
        }
        serde_json::from_value(value)
            .map(Record::Observation)
            .map_err(|e| e.to_string())
    } else {
        serde_json::from_value(value)
            .map(Record::Metadata)
            .map_err(|e| e.to_string())
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

//! The midpoint observation model that every converter and writer speaks.
//!
//! `GpsTime` is i64 ticks of 100 ns since the GPS epoch 1980-01-06, with no
//! leap-second adjustment, so a civil time label maps to the tick with the same
//! label. The model carries `arc` (a monotonic carrier-phase arc counter), from
//! which a RINEX loss-of-lock indicator is derived at the RINEX boundary.

use serde::{de, Deserialize, Deserializer, Serialize, Serializer};
use std::fmt;

pub const TICK_NS: i64 = 100;
pub const MS_PER_WEEK: i64 = 7 * 24 * 60 * 60 * 1000;
pub const DEFAULT_GPS_UTC_SECONDS: i16 = 18;
pub const BDT_GPS_OFFSET_SECONDS: i64 = 14;

/// Unix seconds at the GPS epoch 1980-01-06T00:00:00Z.
const EPOCH_UNIX: i64 = 315964800;

// LLI bits.
pub const LLI_LOST_LOCK: u8 = 1;
pub const LLI_HALF_CYCLE: u8 = 2;
pub const LLI_BOC: u8 = 4;

/// GPS time as 100 ns ticks from the GPS epoch.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct GpsTime(pub i64);

/// A civil (UTC-labelled) broken-down time.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Civil {
    pub year: i64,
    pub month: u32,
    pub day: u32,
    pub hour: u32,
    pub minute: u32,
    pub second: u32,
    pub nanos: u32,
}

fn div_mod(n: i64, d: i64) -> (i64, i64) {
    let mut q = n / d;
    let mut r = n % d;
    if r < 0 {
        q -= 1;
        r += d;
    }
    (q, r)
}

fn floor_div(n: i64, d: i64) -> i64 {
    let q = n / d;
    let r = n % d;
    if r < 0 {
        q - 1
    } else {
        q
    }
}

/// Civil date from days since the Unix epoch (Howard Hinnant's algorithm).
fn civil_from_days(z: i64) -> (i64, u32, u32) {
    let z = z + 719468;
    let era = if z >= 0 { z } else { z - 146096 } / 146097;
    let doe = z - era * 146097; // [0, 146096]
    let yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365; // [0, 399]
    let y = yoe + era * 400;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100); // [0, 365]
    let mp = (5 * doy + 2) / 153; // [0, 11]
    let d = doy - (153 * mp + 2) / 5 + 1; // [1, 31]
    let m = if mp < 10 { mp + 3 } else { mp - 9 }; // [1, 12]
    (y + if m <= 2 { 1 } else { 0 }, m as u32, d as u32)
}

/// Days since the Unix epoch for a civil date.
fn days_from_civil(y: i64, m: u32, d: u32) -> i64 {
    let y = y - if m <= 2 { 1 } else { 0 };
    let era = if y >= 0 { y } else { y - 399 } / 400;
    let yoe = y - era * 400;
    let mp = if m > 2 { m - 3 } else { m + 9 } as i64;
    let doy = (153 * mp + 2) / 5 + d as i64 - 1;
    let doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    era * 146097 + doe - 719468
}

impl GpsTime {
    /// Converts ticks to a civil UTC-labelled time, with no leap adjustment.
    pub fn civil(self) -> Civil {
        let (sec, nsec) = div_mod(self.0 * TICK_NS, 1_000_000_000);
        let unix = EPOCH_UNIX + sec;
        let days = floor_div(unix, 86400);
        let sod = unix - days * 86400;
        let (year, month, day) = civil_from_days(days);
        Civil {
            year,
            month,
            day,
            hour: (sod / 3600) as u32,
            minute: ((sod % 3600) / 60) as u32,
            second: (sod % 60) as u32,
            nanos: nsec as u32,
        }
    }

    /// Inverse of [`civil`], flooring to 100 ns precision.
    pub fn from_civil(c: Civil) -> GpsTime {
        let days = days_from_civil(c.year, c.month, c.day);
        let unix = days * 86400 + c.hour as i64 * 3600 + c.minute as i64 * 60 + c.second as i64;
        let ns = (unix - EPOCH_UNIX) * 1_000_000_000 + c.nanos as i64;
        GpsTime(floor_div(ns, TICK_NS))
    }

    pub fn from_gps_week_millis(week: i64, tow_ms: u32) -> GpsTime {
        GpsTime((week * MS_PER_WEEK + tow_ms as i64) * 1_000_000 / TICK_NS)
    }

    pub fn from_gps_week_seconds(week: i64, tow: f64) -> GpsTime {
        GpsTime(
            week * MS_PER_WEEK * 1_000_000 / TICK_NS + (tow * 1e9 / TICK_NS as f64 + 0.5) as i64,
        )
    }

    pub fn gps_week_millis(self) -> (i64, u32) {
        let (ms, _) = div_mod(self.0 * TICK_NS, 1_000_000);
        let (week, tow) = div_mod(ms, MS_PER_WEEK);
        (week, tow as u32)
    }

    /// Nanoseconds since the GPS epoch (for week-constraint comparisons).
    pub fn epoch_nanos(self) -> i64 {
        self.0 * TICK_NS
    }
}

impl fmt::Display for GpsTime {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let c = self.civil();
        write!(
            f,
            "{:04}-{:02}-{:02}T{:02}:{:02}:{:02}.{:07}",
            c.year,
            c.month,
            c.day,
            c.hour,
            c.minute,
            c.second,
            c.nanos / 100
        )
    }
}

/// Parses an obsj `t` label: `YYYY-MM-DDTHH:MM:SS.fffffff` (7 frac digits).
pub fn parse_time(s: &str) -> Result<GpsTime, String> {
    let err = || format!("rinex: invalid time {:?}", s);
    let (date, time) = s.split_once('T').ok_or_else(err)?;
    let d: Vec<&str> = date.split('-').collect();
    if d.len() != 3 {
        return Err(err());
    }
    let (hms, frac) = time.split_once('.').ok_or_else(err)?;
    let t: Vec<&str> = hms.split(':').collect();
    if t.len() != 3 || frac.len() != 7 {
        return Err(err());
    }
    let year: i64 = d[0].parse().map_err(|_| err())?;
    let month: u32 = d[1].parse().map_err(|_| err())?;
    let day: u32 = d[2].parse().map_err(|_| err())?;
    let hour: u32 = t[0].parse().map_err(|_| err())?;
    let minute: u32 = t[1].parse().map_err(|_| err())?;
    let second: u32 = t[2].parse().map_err(|_| err())?;
    let frac7: u32 = frac.parse().map_err(|_| err())?;
    if !(1..=12).contains(&month)
        || !(1..=31).contains(&day)
        || hour > 23
        || minute > 59
        || second > 60
    {
        return Err(err());
    }
    Ok(GpsTime::from_civil(Civil {
        year,
        month,
        day,
        hour,
        minute,
        second,
        nanos: frac7 * 100,
    }))
}

/// A RINEX satellite identifier such as `G03`, stored as three ASCII bytes.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct SatId(pub [u8; 3]);

impl SatId {
    pub fn system(self) -> u8 {
        self.0[0]
    }

    pub fn is_valid(self) -> bool {
        is_valid_sat(&self.0)
    }

    pub fn as_str(&self) -> &str {
        std::str::from_utf8(&self.0).unwrap_or("")
    }

    /// Builds a `Sys`+`%02d` satellite id, e.g. ("G", 3) -> `G03`.
    pub fn format(sys: u8, num: u8) -> SatId {
        SatId([sys, b'0' + (num / 10) % 10, b'0' + num % 10])
    }
}

/// Error parsing a [`SatId`] or [`SigId`] from a string.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ParseIdError;

impl fmt::Display for ParseIdError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str("invalid GNSS identifier")
    }
}

impl std::error::Error for ParseIdError {}

impl std::str::FromStr for SatId {
    type Err = ParseIdError;
    fn from_str(s: &str) -> Result<SatId, ParseIdError> {
        let b = s.as_bytes();
        if b.len() == 3 {
            let id = SatId([b[0], b[1], b[2]]);
            if id.is_valid() {
                return Ok(id);
            }
        }
        Err(ParseIdError)
    }
}

fn is_valid_sat(b: &[u8]) -> bool {
    if b.len() != 3 || !b"GRESJCI".contains(&b[0]) {
        return false;
    }
    if !b[1].is_ascii_digit() || !b[2].is_ascii_digit() {
        return false;
    }
    let n = (b[1] - b'0') as u32 * 10 + (b[2] - b'0') as u32;
    n != 0
}

impl fmt::Display for SatId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.as_str())
    }
}

/// A RINEX signal identifier such as `1C`, stored as two ASCII bytes.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct SigId(pub [u8; 2]);

impl SigId {
    pub fn band(self) -> u8 {
        self.0[0]
    }

    pub fn is_valid(self) -> bool {
        let b = self.0[0];
        let a = self.0[1];
        (b'1'..=b'9').contains(&b) && a.is_ascii_uppercase()
    }

    pub fn as_str(&self) -> &str {
        std::str::from_utf8(&self.0).unwrap_or("")
    }

    /// The full three-character observation code for `typ` (`C`/`L`/`D`/`S`).
    pub fn code(self, typ: u8) -> ObsCode {
        ObsCode([typ, self.0[0], self.0[1]])
    }
}

impl std::str::FromStr for SigId {
    type Err = ParseIdError;
    fn from_str(s: &str) -> Result<SigId, ParseIdError> {
        let b = s.as_bytes();
        if b.len() == 2 {
            let id = SigId([b[0], b[1]]);
            if id.is_valid() {
                return Ok(id);
            }
        }
        Err(ParseIdError)
    }
}

impl fmt::Display for SigId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.as_str())
    }
}

/// A complete three-character RINEX observation code such as `C1C`.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct ObsCode(pub [u8; 3]);

impl ObsCode {
    pub fn as_str(&self) -> &str {
        std::str::from_utf8(&self.0).unwrap_or("")
    }
    pub fn obs_type(self) -> u8 {
        self.0[0]
    }
    pub fn signal(self) -> SigId {
        SigId([self.0[1], self.0[2]])
    }
}

impl fmt::Display for ObsCode {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.as_str())
    }
}

pub const TYPE_CODE: u8 = b'C';
pub const TYPE_PHASE: u8 = b'L';
pub const TYPE_DOPPLER: u8 = b'D';
pub const TYPE_SIGNAL_STRENGTH: u8 = b'S';

/// Per-signal observation values; a small `Copy` struct with no heap data.
///
/// The serde attributes define the obsj wire form: every value is omitted when
/// absent/zero, `dop` serializes as `do`, and `arc`/`hc`/`bt` are omitted when
/// zero/false. obsj is validated by diffobs at exact-f64, so the only
/// requirement on serialization is that values round-trip — which serde's `ryu`
/// formatting (plus `arbitrary_precision` on read) guarantees.
#[derive(Clone, Copy, Default, PartialEq, Serialize, Deserialize)]
pub struct SignalValues {
    #[serde(skip_serializing_if = "Option::is_none", default)]
    pub frq: Option<i8>,
    #[serde(skip_serializing_if = "Option::is_none", default)]
    pub pr: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none", default)]
    pub cp: Option<f64>,
    #[serde(rename = "do", skip_serializing_if = "Option::is_none", default)]
    pub dop: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none", default)]
    pub cn0: Option<f32>,
    #[serde(skip_serializing_if = "is_zero", default)]
    pub arc: u32,
    #[serde(skip_serializing_if = "is_false", default)]
    pub hc: bool,
    #[serde(skip_serializing_if = "is_false", default)]
    pub bt: bool,
    /// Transient loss-of-lock flag emitted by converters, consumed by
    /// [`LossOfLockSink`](crate::arc::LossOfLockSink) to compute `arc`. Never on
    /// the wire (the canonical form is `arc`); `ll` as an obsj key is rejected.
    #[serde(skip)]
    pub(crate) ll: bool,
}

fn is_zero(n: &u32) -> bool {
    *n == 0
}
fn is_false(b: &bool) -> bool {
    !*b
}

impl SignalValues {
    pub fn is_zero(&self) -> bool {
        self.frq.is_none()
            && self.pr.is_none()
            && self.cp.is_none()
            && self.dop.is_none()
            && self.cn0.is_none()
            && self.arc == 0
            && !self.hc
            && !self.bt
    }

    /// Sets the transient loss-of-lock flag and the half-cycle/BOC bits from a
    /// RINEX LLI byte (the inverse of [`rinex_lli`](Self::rinex_lli)). `arc` is
    /// assigned later by the [`LossOfLockSink`](crate::arc::LossOfLockSink).
    pub fn set_lli(&mut self, lli: u8) {
        self.ll = lli & LLI_LOST_LOCK != 0;
        self.hc = lli & LLI_HALF_CYCLE != 0;
        self.bt = lli & LLI_BOC != 0;
    }

    pub fn rinex_lli(&self, arc_changed: bool) -> u8 {
        let mut x = 0u8;
        if arc_changed {
            x |= LLI_LOST_LOCK;
        }
        if self.hc {
            x |= LLI_HALF_CYCLE;
        }
        if self.bt {
            x |= LLI_BOC;
        }
        x
    }
}

/// One satellite-signal observation at one epoch.
///
/// Deserialization uses serde's `flatten` to fold the `SignalValues` fields up
/// into the record. Serialization is hand-written instead: `flatten` on the
/// write side routes every record through serde_json's intermediate `Content`
/// map, which dominates output time on large conversions; emitting the fields
/// directly produces byte-identical JSON without that buffering.
#[derive(Clone, Copy, Deserialize)]
pub struct SignalObservation {
    pub t: GpsTime,
    pub sat: SatId,
    pub sig: SigId,
    #[serde(flatten)]
    pub v: SignalValues,
}

impl Serialize for SignalObservation {
    fn serialize<S: Serializer>(&self, s: S) -> Result<S::Ok, S::Error> {
        use serde::ser::SerializeMap;
        let v = &self.v;
        let mut m = s.serialize_map(None)?;
        m.serialize_entry("t", &self.t)?;
        m.serialize_entry("sat", &self.sat)?;
        m.serialize_entry("sig", &self.sig)?;
        if let Some(x) = v.frq {
            m.serialize_entry("frq", &x)?;
        }
        if let Some(x) = v.pr {
            m.serialize_entry("pr", &x)?;
        }
        if let Some(x) = v.cp {
            m.serialize_entry("cp", &x)?;
        }
        if let Some(x) = v.dop {
            m.serialize_entry("do", &x)?;
        }
        if let Some(x) = v.cn0 {
            m.serialize_entry("cn0", &x)?;
        }
        if v.arc != 0 {
            m.serialize_entry("arc", &v.arc)?;
        }
        if v.hc {
            m.serialize_entry("hc", &true)?;
        }
        if v.bt {
            m.serialize_entry("bt", &true)?;
        }
        m.end()
    }
}

impl SignalObservation {
    pub fn system(&self) -> u8 {
        self.sat.system()
    }

    /// Whether any observation code (C/L/D/S) is present.
    pub fn has_any_code(&self) -> bool {
        self.v.pr.is_some() || self.v.cp.is_some() || self.v.dop.is_some() || self.v.cn0.is_some()
    }
}

#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct SignalKey {
    pub sat: SatId,
    pub sig: SigId,
}

/// An instant in time, used for metadata run dates (Unix scale).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Instant {
    pub secs: i64,
    pub nanos: u32,
}

impl Instant {
    pub fn civil(self) -> Civil {
        let days = floor_div(self.secs, 86400);
        let sod = self.secs - days * 86400;
        let (year, month, day) = civil_from_days(days);
        Civil {
            year,
            month,
            day,
            hour: (sod / 3600) as u32,
            minute: ((sod % 3600) / 60) as u32,
            second: (sod % 60) as u32,
            nanos: self.nanos,
        }
    }

    pub fn from_civil(c: Civil) -> Instant {
        let days = days_from_civil(c.year, c.month, c.day);
        Instant {
            secs: days * 86400 + c.hour as i64 * 3600 + c.minute as i64 * 60 + c.second as i64,
            nanos: c.nanos,
        }
    }

    /// Nanoseconds since the GPS epoch (for week-constraint comparisons).
    pub fn gps_nanos(self) -> i64 {
        (self.secs - EPOCH_UNIX) * 1_000_000_000 + self.nanos as i64
    }

    pub fn is_zero(self) -> bool {
        self.secs == 0 && self.nanos == 0
    }
}

#[derive(Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct MetadataRun {
    #[serde(default, skip_serializing_if = "String::is_empty")]
    pub program: String,
    #[serde(default, skip_serializing_if = "String::is_empty")]
    pub by: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub date: Option<Instant>,
}

impl MetadataRun {
    pub fn is_zero(&self) -> bool {
        self.program.is_empty() && self.by.is_empty() && self.date.is_none()
    }
}

#[derive(Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct Marker {
    #[serde(default, skip_serializing_if = "String::is_empty")]
    pub name: String,
    #[serde(default, skip_serializing_if = "String::is_empty")]
    pub number: String,
    #[serde(rename = "type", default, skip_serializing_if = "String::is_empty")]
    pub type_: String,
}

impl Marker {
    pub fn is_zero(&self) -> bool {
        self.name.is_empty() && self.number.is_empty() && self.type_.is_empty()
    }
}

#[derive(Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct Receiver {
    #[serde(default, skip_serializing_if = "String::is_empty")]
    pub number: String,
    #[serde(rename = "type", default, skip_serializing_if = "String::is_empty")]
    pub type_: String,
    #[serde(default, skip_serializing_if = "String::is_empty")]
    pub version: String,
}

impl Receiver {
    pub fn is_zero(&self) -> bool {
        self.number.is_empty() && self.type_.is_empty() && self.version.is_empty()
    }
}

#[derive(Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct Antenna {
    #[serde(default, skip_serializing_if = "String::is_empty")]
    pub number: String,
    #[serde(rename = "type", default, skip_serializing_if = "String::is_empty")]
    pub type_: String,
}

impl Antenna {
    pub fn is_zero(&self) -> bool {
        self.number.is_empty() && self.type_.is_empty()
    }
}

/// Header-related facts; an obsj metadata record.
#[derive(Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct Metadata {
    #[serde(default, skip_serializing_if = "String::is_empty")]
    pub version: String,
    #[serde(default, skip_serializing_if = "MetadataRun::is_zero")]
    pub run: MetadataRun,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub comment: Vec<String>,
    #[serde(default, skip_serializing_if = "Marker::is_zero")]
    pub marker: Marker,
    #[serde(default, skip_serializing_if = "String::is_empty")]
    pub observer: String,
    #[serde(default, skip_serializing_if = "String::is_empty")]
    pub agency: String,
    #[serde(default, skip_serializing_if = "Receiver::is_zero")]
    pub receiver: Receiver,
    #[serde(default, skip_serializing_if = "Antenna::is_zero")]
    pub antenna: Antenna,
    #[serde(
        rename = "approxPosition",
        default,
        skip_serializing_if = "Option::is_none"
    )]
    pub approx_position: Option<[f64; 3]>,
    #[serde(
        rename = "antennaDelta",
        default,
        skip_serializing_if = "Option::is_none"
    )]
    pub antenna_delta: Option<[f64; 3]>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub interval: Option<f64>,
    #[serde(
        rename = "leapSeconds",
        default,
        skip_serializing_if = "Option::is_none"
    )]
    pub leap_seconds: Option<i16>,
}

impl Metadata {
    pub fn is_zero(&self) -> bool {
        self.version.is_empty()
            && self.run.is_zero()
            && self.comment.is_empty()
            && self.marker.name.is_empty()
            && self.marker.number.is_empty()
            && self.marker.type_.is_empty()
            && self.observer.is_empty()
            && self.agency.is_empty()
            && self.receiver.is_zero()
            && self.antenna.is_zero()
            && self.approx_position.is_none()
            && self.antenna_delta.is_none()
            && self.interval.is_none()
            && self.leap_seconds.is_none()
    }

    /// Merges `b` into `self`, with `b` taking precedence where set.
    pub fn merge(&mut self, b: &Metadata) {
        if !b.version.is_empty() {
            self.version = b.version.clone();
        }
        if !b.run.is_zero() {
            self.run = b.run.clone();
        }
        if !b.comment.is_empty() {
            self.comment.extend_from_slice(&b.comment);
        }
        if !b.marker.name.is_empty() {
            self.marker.name = b.marker.name.clone();
        }
        if !b.marker.number.is_empty() {
            self.marker.number = b.marker.number.clone();
        }
        if !b.marker.type_.is_empty() {
            self.marker.type_ = b.marker.type_.clone();
        }
        if !b.observer.is_empty() {
            self.observer = b.observer.clone();
        }
        if !b.agency.is_empty() {
            self.agency = b.agency.clone();
        }
        if !b.receiver.is_zero() {
            self.receiver = b.receiver.clone();
        }
        if !b.antenna.is_zero() {
            self.antenna = b.antenna.clone();
        }
        if b.approx_position.is_some() {
            self.approx_position = b.approx_position;
        }
        if b.antenna_delta.is_some() {
            self.antenna_delta = b.antenna_delta;
        }
        if b.interval.is_some() {
            self.interval = b.interval;
        }
        if b.leap_seconds.is_some() {
            self.leap_seconds = b.leap_seconds;
        }
    }
}

// ---- serde for the wire types ----

impl Serialize for GpsTime {
    fn serialize<S: Serializer>(&self, s: S) -> Result<S::Ok, S::Error> {
        s.collect_str(self)
    }
}

impl<'de> Deserialize<'de> for GpsTime {
    fn deserialize<D: Deserializer<'de>>(d: D) -> Result<Self, D::Error> {
        let s = <std::borrow::Cow<str>>::deserialize(d)?;
        parse_time(&s).map_err(de::Error::custom)
    }
}

impl Serialize for SatId {
    fn serialize<S: Serializer>(&self, s: S) -> Result<S::Ok, S::Error> {
        s.serialize_str(self.as_str())
    }
}

impl<'de> Deserialize<'de> for SatId {
    fn deserialize<D: Deserializer<'de>>(d: D) -> Result<Self, D::Error> {
        let s = <std::borrow::Cow<str>>::deserialize(d)?;
        s.parse()
            .map_err(|_| de::Error::custom(format!("invalid satellite identifier {s:?}")))
    }
}

impl Serialize for SigId {
    fn serialize<S: Serializer>(&self, s: S) -> Result<S::Ok, S::Error> {
        s.serialize_str(self.as_str())
    }
}

impl<'de> Deserialize<'de> for SigId {
    fn deserialize<D: Deserializer<'de>>(d: D) -> Result<Self, D::Error> {
        let s = <std::borrow::Cow<str>>::deserialize(d)?;
        s.parse()
            .map_err(|_| de::Error::custom(format!("invalid signal identifier {s:?}")))
    }
}

impl Serialize for Instant {
    fn serialize<S: Serializer>(&self, s: S) -> Result<S::Ok, S::Error> {
        s.collect_str(&ZonelessDateTime(*self))
    }
}

impl<'de> Deserialize<'de> for Instant {
    fn deserialize<D: Deserializer<'de>>(d: D) -> Result<Self, D::Error> {
        let s = <std::borrow::Cow<str>>::deserialize(d)?;
        crate::json::parse_rfc3339_public(&s)
            .ok_or_else(|| de::Error::custom(format!("invalid time {s:?}")))
    }
}

/// Formats an `Instant` as a zone-less civil datetime (obsj dates carry no
/// timezone), trailing-zero-trimmed in the fractional part.
struct ZonelessDateTime(Instant);

impl fmt::Display for ZonelessDateTime {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let c = self.0.civil();
        write!(
            f,
            "{:04}-{:02}-{:02}T{:02}:{:02}:{:02}",
            c.year, c.month, c.day, c.hour, c.minute, c.second
        )?;
        if c.nanos != 0 {
            let frac = format!("{:09}", c.nanos);
            write!(f, ".{}", frac.trim_end_matches('0'))?;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn time_roundtrip_label() {
        // 2026-05-28T05:21:37.0969400 must round-trip through Display/parse.
        let s = "2026-05-28T05:21:37.0969400";
        let t = parse_time(s).unwrap();
        assert_eq!(t.to_string(), s);
    }

    #[test]
    fn week_millis() {
        let t = GpsTime::from_gps_week_millis(2316, 19437000);
        let (w, tow) = t.gps_week_millis();
        assert_eq!((w, tow), (2316, 19437000));
    }

    #[test]
    fn sat_format() {
        assert_eq!(SatId::format(b'G', 3).as_str(), "G03");
        assert_eq!(SatId::format(b'C', 14).as_str(), "C14");
        assert!("G03".parse::<SatId>().is_ok());
        assert!("X03".parse::<SatId>().is_err());
        assert!("G00".parse::<SatId>().is_err());
    }
}

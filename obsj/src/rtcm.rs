//! RTCM MSM7 → obsj conversion.
//!
//! Framing, CRC-24Q, and the MSM bit-unpacking are handled by the `rtcm-rs`
//! crate; this module is the converter algorithm on top of it: cell math, slip
//! detection (emitted as the loss-of-lock flag — the [`LossOfLockSink`] turns it
//! into `arc`), GLONASS-week resolution, and metadata extraction.
//!
//! Where `rtcm-rs` hands back a *pre-scaled* `f64` for a fine field (DF405/406/
//! 404/408/398), the value equals the raw integer times an exact power of two,
//! so the same arithmetic the reference performs reproduces the identical `f64`.
//! The one exception is CN0, whose reference value is computed in `f32`, so the
//! raw integer is recovered first.
//!
//! [`LossOfLockSink`]: crate::arc::LossOfLockSink

use crate::freq::signal_frequency_hz;
use crate::obs::*;
use crate::sink::Sink;
use rtcm_rs::prelude::{next_msg_frame, Message, MessageFrame};

const SPEED_OF_LIGHT: f64 = 299792458.0;
const RANGE_MS: f64 = SPEED_OF_LIGHT * 0.001;
const RINEX_TICKS_PER_MS: i64 = 10000;
const SECOND_MS: i64 = 1000;
const HOUR_MS: i64 = 60 * 60 * SECOND_MS;
const DAY_MS: i64 = 24 * HOUR_MS;
const WEEK_MS: i64 = 7 * DAY_MS;
const HALF_WEEK_MS: i64 = WEEK_MS / 2;
const BDT_OFFSET_MS: i64 = 14 * SECOND_MS;
const GLONASS_UTC_OFFSET_MS: i64 = 3 * HOUR_MS;
const DEFAULT_GPS_UTC_MS: i64 = 18 * SECOND_MS;

/// A UTC interval (ns since the GPS epoch) constraining epoch resolution.
/// `dur_ns == 0` means "no constraint".
#[derive(Clone, Copy, Default)]
pub struct TimeInterval {
    pub start_ns: i64,
    pub dur_ns: i64,
}

impl TimeInterval {
    pub fn is_zero(&self) -> bool {
        self.start_ns == 0 && self.dur_ns == 0
    }
}

#[derive(Clone, Copy)]
pub struct Options {
    pub use_spec_phase_range_rate_sign: bool,
    pub omit_zero_do: bool,
}

#[derive(Clone, Copy, PartialEq, Eq, Hash)]
enum Gnss {
    Gps,
    Glonass,
    Galileo,
    Sbas,
    Qzss,
    Beidou,
    Irnss,
}

impl Gnss {
    fn rinex_sys(self) -> u8 {
        match self {
            Gnss::Gps => b'G',
            Gnss::Glonass => b'R',
            Gnss::Galileo => b'E',
            Gnss::Sbas => b'S',
            Gnss::Qzss => b'J',
            Gnss::Beidou => b'C',
            Gnss::Irnss => b'I',
        }
    }
}

// ---------------------------------------------------------------------------
// Normalized per-message data
//
// The seven MSM7 message types share a satellite fragment and an identically
// shaped signal fragment (differing only in the signal-id enum). They are
// flattened into these structs so one converter handles them all. Fine fields
// stay as `rtcm-rs`'s pre-scaled `f64`: each equals raw × 2⁻ⁿ, exactly what the
// reference arithmetic multiplies by.
// ---------------------------------------------------------------------------

struct SatData {
    satellite_id: u8,
    /// DF397 rough range integer ms (`None` ⇒ invalid).
    range_int: Option<u8>,
    /// DF398 rough range modulo, already scaled (raw × 2⁻¹⁰ ms).
    range_mod: f64,
    /// DF399 rough phase-range rate (`None` ⇒ invalid).
    phase_rate: Option<i16>,
    /// DF419 GLONASS frequency channel, biased by −7 (`None` outside GLONASS).
    glo_channel: Option<i8>,
}

struct SigData {
    satellite_id: u8,
    band: u8,
    attribute: char,
    /// DF405 fine pseudorange, scaled (raw × 2⁻²⁹ ms).
    pseudorange: Option<f64>,
    /// DF406 fine phase range, scaled (raw × 2⁻³¹ ms).
    phase_range: Option<f64>,
    /// DF404 fine phase-range rate, scaled (raw × 1e-4 m/s).
    phase_rate: Option<f64>,
    /// DF407 lock-time indicator.
    lock: u16,
    /// DF420 half-cycle ambiguity.
    half_cycle: bool,
    /// DF408 CN0, scaled (raw × 2⁻⁴ dB-Hz; `None` ⇒ invalid).
    cnr: Option<f64>,
}

/// Flattens a satellite fragment into [`SatData`]. The two arms differ only in
/// the GLONASS frequency channel: present (GLONASS) or absent (everything else).
macro_rules! sat_data {
    ($seg:expr, glo) => {
        $seg.satellite_data
            .iter()
            .map(|s| SatData {
                satellite_id: s.satellite_id,
                range_int: s.gnss_satellite_rough_range_integer_ms,
                range_mod: s.gnss_satellite_rough_range_mod1ms_ms,
                phase_rate: s.gnss_satellite_rough_phaserange_rates_m_s,
                glo_channel: s.glonass_satellite_frequency_channel_number,
            })
            .collect::<Vec<_>>()
    };
    ($seg:expr, none) => {
        $seg.satellite_data
            .iter()
            .map(|s| SatData {
                satellite_id: s.satellite_id,
                range_int: s.gnss_satellite_rough_range_integer_ms,
                range_mod: s.gnss_satellite_rough_range_mod1ms_ms,
                phase_rate: s.gnss_satellite_rough_phaserange_rates_m_s,
                glo_channel: None,
            })
            .collect::<Vec<_>>()
    };
}

macro_rules! sig_data {
    ($seg:expr) => {
        $seg.signal_data
            .iter()
            .map(|s| SigData {
                satellite_id: s.satellite_id,
                band: s.signal_id.band(),
                attribute: s.signal_id.attribute(),
                pseudorange: s.gnss_signal_fine_pseudorange_ext_ms,
                phase_range: s.gnss_signal_fine_phaserange_ext_ms,
                phase_rate: s.gnss_signal_fine_phaserange_rate_m_s,
                lock: s.gnss_phaserange_lock_time_ext_ind,
                half_cycle: s.half_cycle_ambiguity_ind != 0,
                cnr: s.gnss_signal_cnr_ext_dbhz,
            })
            .collect::<Vec<_>>()
    };
}

// ---------------------------------------------------------------------------
// Converter
// ---------------------------------------------------------------------------

pub struct Converter<S: Sink> {
    sink: S,
    opts: Options,
    week: TimeInterval,
    leap_ms: i64,
    time: std::collections::HashMap<Gnss, GpsTime>,
    lock: std::collections::HashMap<SignalKey, u16>,
    slip: std::collections::HashMap<SignalKey, bool>,
}

impl<S: Sink> Converter<S> {
    pub fn new(sink: S, opts: Options) -> Self {
        Converter {
            sink,
            opts,
            week: TimeInterval::default(),
            leap_ms: DEFAULT_GPS_UTC_MS,
            time: std::collections::HashMap::new(),
            lock: std::collections::HashMap::new(),
            slip: std::collections::HashMap::new(),
        }
    }

    /// Forwards metadata directly to the sink (for the initial CLI metadata).
    pub fn sink_metadata(&mut self, m: &Metadata) -> Result<(), String> {
        self.sink.metadata(m).map_err(|e| e.to_string())
    }

    pub fn flush(&mut self) -> Result<(), String> {
        self.sink.flush().map_err(|e| e.to_string())
    }

    /// Converts one RTCM frame. `week` is the per-message epoch constraint.
    /// Returns whether an observation/metadata record was produced.
    pub fn convert_frame(&mut self, frame: &[u8], week: TimeInterval) -> Result<bool, String> {
        let mf = match MessageFrame::new(frame) {
            Ok(m) => m,
            Err(_) => {
                self.set_week(week);
                return Ok(false);
            }
        };
        match mf.get_message() {
            Message::Msg1077(t) => {
                self.convert_msm(Gnss::Gps, t.gps_epoch_time_ms, &t.data_segment, week)
            }
            Message::Msg1087(t) => {
                let epoch = ((t.glo_day_of_week.unwrap_or(7) as u32) << 27) | t.glo_epoch_time_ms;
                self.convert_msm(Gnss::Glonass, epoch, &t.data_segment, week)
            }
            Message::Msg1097(t) => {
                self.convert_msm(Gnss::Galileo, t.gal_epoch_time_ms, &t.data_segment, week)
            }
            Message::Msg1107(t) => {
                self.convert_msm(Gnss::Sbas, t.gps_epoch_time_ms, &t.data_segment, week)
            }
            Message::Msg1117(t) => {
                self.convert_msm(Gnss::Qzss, t.qzss_epoch_time_ms, &t.data_segment, week)
            }
            Message::Msg1127(t) => {
                self.convert_msm(Gnss::Beidou, t.bds_epoch_time_ms, &t.data_segment, week)
            }
            Message::Msg1137(t) => {
                self.convert_msm(Gnss::Irnss, t.navic_epoch_time_ms, &t.data_segment, week)
            }
            Message::Msg1005(t) => {
                self.set_week(week);
                let mut m = Metadata::default();
                m.approx_position = Some([
                    t.antenna_ref_point_ecef_x_m,
                    t.antenna_ref_point_ecef_y_m,
                    t.antenna_ref_point_ecef_z_m,
                ]);
                m.marker.number = t.reference_station_id.to_string();
                self.emit_metadata(&m)
            }
            Message::Msg1006(t) => {
                self.set_week(week);
                let mut m = Metadata::default();
                m.approx_position = Some([
                    t.antenna_ref_point_ecef_x_m,
                    t.antenna_ref_point_ecef_y_m,
                    t.antenna_ref_point_ecef_z_m,
                ]);
                m.antenna_delta = Some([t.antenna_height_m, 0.0, 0.0]);
                m.marker.number = t.reference_station_id.to_string();
                self.emit_metadata(&m)
            }
            Message::Msg1007(t) => {
                self.set_week(week);
                let mut m = Metadata::default();
                m.antenna.type_ = clean(&t.antenna_descriptor_str);
                m.marker.number = t.reference_station_id.to_string();
                self.emit_metadata(&m)
            }
            Message::Msg1008(t) => {
                self.set_week(week);
                let mut m = Metadata::default();
                m.antenna.type_ = clean(&t.antenna_descriptor_str);
                m.antenna.number = clean(&t.antenna_serial_number_str);
                m.marker.number = t.reference_station_id.to_string();
                self.emit_metadata(&m)
            }
            Message::Msg1013(t) => {
                self.set_week(week);
                let mut m = Metadata::default();
                if let Some(leap) = t.leap_seconds_gps_utc_s {
                    let leap = leap as i16;
                    self.leap_ms = leap as i64 * SECOND_MS;
                    m.leap_seconds = Some(leap);
                }
                m.marker.number = t.reference_station_id.to_string();
                self.emit_metadata(&m)
            }
            Message::Msg1033(t) => {
                self.set_week(week);
                let mut m = Metadata::default();
                m.antenna.type_ = clean(&t.antenna_descriptor_str);
                m.antenna.number = clean(&t.antenna_serial_number_str);
                m.receiver = Receiver {
                    number: clean(&t.receiver_serial_number_str),
                    type_: clean(&t.receiver_type_descriptor_str),
                    version: clean(&t.receiver_firmware_version_str),
                };
                m.marker.number = t.reference_station_id.to_string();
                self.emit_metadata(&m)
            }
            Message::Msg1230(t) => {
                self.set_week(week);
                let mut m = Metadata::default();
                m.marker.number = t.reference_station_id.to_string();
                self.emit_metadata(&m)
            }
            _ => {
                self.set_week(week);
                Ok(false)
            }
        }
    }

    fn set_week(&mut self, week: TimeInterval) {
        if !week.is_zero() {
            self.week = week;
        }
    }

    fn emit_metadata(&mut self, meta: &Metadata) -> Result<bool, String> {
        self.sink.metadata(meta).map_err(|e| e.to_string())?;
        Ok(true)
    }

    /// Converts one MSM7 data segment. `seg` is the `rtcm-rs` data segment; the
    /// `sat_data!`/`sig_data!` macros flatten its per-type fields.
    fn convert_msm<Seg>(
        &mut self,
        gnss: Gnss,
        epoch_time: u32,
        seg: &Seg,
        week: TimeInterval,
    ) -> Result<bool, String>
    where
        Seg: HasMsmData,
    {
        self.set_week(week);
        let t = self.resolve_time(epoch_time, gnss, week)?;
        self.time.insert(gnss, t);
        let (sats, sigs) = seg.normalize();
        let mut seen = false;
        for sig in &sigs {
            if let Some(sat) = sats.iter().find(|s| s.satellite_id == sig.satellite_id) {
                if self.convert_cell(t, gnss, sat, sig)? {
                    seen = true;
                }
            }
        }
        Ok(seen)
    }

    fn convert_cell(&mut self, t: GpsTime, gnss: Gnss, sat: &SatData, sig: &SigData) -> Result<bool, String> {
        let sys = gnss.rinex_sys();
        let sat_num = rinex_sat_num(gnss, sat.satellite_id);
        if sat_num == 0 {
            return Ok(false);
        }
        let sig_id = SigId([b'0' + sig.band, sig.attribute as u8]);
        if !sig_id.is_valid() {
            return Ok(false);
        }
        let satid = SatId::format(sys, sat_num);
        let mut o = SignalObservation {
            t,
            sat: satid,
            sig: sig_id,
            v: SignalValues::default(),
        };

        let mut frq = None;
        if gnss == Gnss::Glonass {
            if let Some(c) = sat.glo_channel.filter(|&c| (-7..=6).contains(&c)) {
                o.v.frq = Some(c);
                frq = Some(c);
            }
        }
        let rough = rough_range(sat);
        if let Some(pr) = pseudorange(rough, sig.pseudorange) {
            o.v.pr = Some(pr);
        }
        let freq = signal_frequency_hz(sys, sig_id, frq);
        if let Some(cp) = carrier_phase(rough, sig.phase_range, freq) {
            o.v.cp = Some(cp);
        }
        if let Some(dop) = doppler(sat.phase_rate, sig.phase_rate, freq, self.opts.use_spec_phase_range_rate_sign) {
            if !self.opts.omit_zero_do || dop != 0.0 {
                o.v.dop = Some(dop);
            }
        }
        if let Some(c) = cn0(sig.cnr) {
            o.v.cn0 = Some(c);
        }
        let (ll, hc) = self.slip_hc(satid, sig_id, sig, o.v.cp.is_some());
        o.v.ll = ll;
        o.v.hc = hc;
        if !o.has_any_code() {
            return Ok(false);
        }
        self.sink.observation(&o).map_err(|e| e.to_string())?;
        Ok(true)
    }

    /// Detects a carrier-phase slip (loss of lock) and the half-cycle bit for one
    /// cell. Returns the per-observation `ll` flag; the downstream
    /// [`LossOfLockSink`](crate::arc::LossOfLockSink) turns it into `arc`. A slip
    /// on an epoch without phase is deferred to the next epoch that has it.
    fn slip_hc(&mut self, sat: SatId, sig: SigId, s: &SigData, has_phase: bool) -> (bool, bool) {
        let k = SignalKey { sat, sig };
        let mut ll = *self.slip.get(&k).unwrap_or(&false);
        let prev = *self.lock.get(&k).unwrap_or(&0);
        if s.lock < prev || (s.lock == 0 && prev == 0) {
            ll = true;
        }
        self.lock.insert(k, s.lock);
        if ll && !has_phase {
            self.slip.insert(k, true);
        } else {
            self.slip.remove(&k);
        }
        (ll, s.half_cycle)
    }

    fn resolve_time(&self, epoch_time: u32, gnss: Gnss, week: TimeInterval) -> Result<GpsTime, String> {
        let offsets = self.epoch_week_offsets(epoch_time, gnss)?;
        if !week.is_zero() {
            return resolve_week(&offsets, week);
        }
        let prev = self.time.get(&gnss).copied();
        if prev.is_none() && !self.week.is_zero() {
            return resolve_week(&offsets, self.week);
        }
        if let Some(prev) = prev {
            return Ok(resolve_continuity(&offsets, prev));
        }
        Err("RTCM MSM7 epoch needs a week constraint".to_string())
    }

    fn epoch_week_offsets(&self, epoch_time: u32, gnss: Gnss) -> Result<Vec<i64>, String> {
        match gnss {
            Gnss::Glonass => self.glonass_epoch_week_offsets(epoch_time),
            Gnss::Beidou => {
                if epoch_time as i64 >= WEEK_MS {
                    return Err(format!("invalid RTCM MSM7 epoch time {}", epoch_time));
                }
                Ok(vec![epoch_time as i64 + BDT_OFFSET_MS])
            }
            _ => {
                if epoch_time as i64 >= WEEK_MS {
                    return Err(format!("invalid RTCM MSM7 epoch time {}", epoch_time));
                }
                Ok(vec![epoch_time as i64])
            }
        }
    }

    fn glonass_epoch_week_offsets(&self, epoch: u32) -> Result<Vec<i64>, String> {
        let day = (epoch >> 27) as i64;
        let tod = (epoch & ((1 << 27) - 1)) as i64;
        if tod >= DAY_MS {
            return Err(format!("invalid GLONASS time of day {}", tod));
        }
        if day != 7 {
            return Ok(vec![day * DAY_MS + tod - GLONASS_UTC_OFFSET_MS + self.leap_ms]);
        }
        Ok((0..7)
            .map(|d| d * DAY_MS + tod - GLONASS_UTC_OFFSET_MS + self.leap_ms)
            .collect())
    }
}

/// Bridges the per-type `rtcm-rs` data segments to the normalized form, so
/// [`Converter::convert_msm`] is written once. Implemented via the macro below.
trait HasMsmData {
    fn normalize(&self) -> (Vec<SatData>, Vec<SigData>);
}

macro_rules! impl_has_msm_data {
    ($($t:ty),+ $(,)?) => {$(
        impl HasMsmData for $t {
            fn normalize(&self) -> (Vec<SatData>, Vec<SigData>) {
                (sat_data!(self, none), sig_data!(self))
            }
        }
    )+};
}

impl_has_msm_data!(
    rtcm_rs::msg::Msg1077Data,
    rtcm_rs::msg::Msg1097Data,
    rtcm_rs::msg::Msg1107Data,
    rtcm_rs::msg::Msg1117Data,
    rtcm_rs::msg::Msg1127Data,
    rtcm_rs::msg::Msg1137Data,
);

// GLONASS uses a distinct satellite fragment that carries the frequency channel.
impl HasMsmData for rtcm_rs::msg::Msg1087Data {
    fn normalize(&self) -> (Vec<SatData>, Vec<SigData>) {
        (sat_data!(self, glo), sig_data!(self))
    }
}

// ---------------------------------------------------------------------------
// Cell math — uses `rtcm-rs`'s pre-scaled `f64`s, which equal raw × 2⁻ⁿ.
// ---------------------------------------------------------------------------

fn rough_range(sat: &SatData) -> Option<f64> {
    let rint = sat.range_int?;
    Some(rint as f64 * RANGE_MS + sat.range_mod * RANGE_MS)
}

fn pseudorange(rough: Option<f64>, fine: Option<f64>) -> Option<f64> {
    Some(rough? + fine? * RANGE_MS)
}

fn carrier_phase(rough: Option<f64>, fine: Option<f64>, freq: Option<f64>) -> Option<f64> {
    let freq = freq?;
    Some((rough? + fine? * RANGE_MS) * freq / SPEED_OF_LIGHT)
}

fn doppler(rough: Option<i16>, fine: Option<f64>, freq: Option<f64>, spec_sign: bool) -> Option<f64> {
    let freq = freq?;
    let mut prr = rough? as f64 + fine?;
    if spec_sign {
        prr = -prr;
    }
    // The f32 narrowing is load-bearing for the exact-f64 obsj value.
    Some(((prr * freq / SPEED_OF_LIGHT) as f32) as f64)
}

fn cn0(cnr: Option<f64>) -> Option<f32> {
    // Recover the raw integer (DF408 scales by 2⁻⁴, exact) and apply the f32
    // arithmetic the reference uses for CN0.
    let raw = (cnr? * 16.0).round() as u16;
    Some(raw as f32 * 0.0625)
}

fn rinex_sat_num(gnss: Gnss, sat_id: u8) -> u8 {
    let in_range = |hi| (1..=hi).contains(&sat_id);
    match gnss {
        Gnss::Gps => in_range(63).then_some(sat_id).unwrap_or(0),
        Gnss::Glonass => in_range(24).then_some(sat_id).unwrap_or(0),
        Gnss::Galileo => in_range(50).then_some(sat_id).unwrap_or(0),
        Gnss::Sbas => in_range(39).then_some(sat_id + 19).unwrap_or(0),
        Gnss::Qzss => in_range(10).then_some(sat_id).unwrap_or(0),
        Gnss::Beidou => in_range(63).then_some(sat_id).unwrap_or(0),
        Gnss::Irnss => in_range(14).then_some(sat_id).unwrap_or(0),
    }
}

/// Trims trailing spaces and NULs from an RTCM descriptor string.
fn clean(s: &impl core::fmt::Display) -> String {
    s.to_string().trim_end_matches([' ', '\0']).to_string()
}

// ---- week resolution ----

fn floor_div(n: i64, d: i64) -> i64 {
    let q = n / d;
    let r = n % d;
    if r < 0 {
        q - 1
    } else {
        q
    }
}

fn resolve_week(offsets: &[i64], week: TimeInterval) -> Result<GpsTime, String> {
    let end_ns = week.start_ns + week.dur_ns;
    let start_ms = floor_div(week.start_ns, 1_000_000);
    let mut matched = GpsTime(0);
    let mut nmatch = 0;
    let mut seen: Vec<i64> = Vec::new();
    for &offset in offsets {
        let w = floor_div(start_ms - offset, WEEK_MS);
        for i in -1..=2 {
            let ticks = ((w + i) * WEEK_MS + offset) * RINEX_TICKS_PER_MS;
            if seen.contains(&ticks) {
                continue;
            }
            seen.push(ticks);
            let instant_ns = ticks * TICK_NS;
            if instant_ns >= week.start_ns && instant_ns < end_ns {
                matched = GpsTime(ticks);
                nmatch += 1;
            }
        }
    }
    match nmatch {
        0 => Err("no MSM7 epoch matches the week constraint".to_string()),
        1 => Ok(matched),
        _ => Err("MSM7 epoch is ambiguous in the week constraint".to_string()),
    }
}

fn resolve_continuity(offsets: &[i64], prev: GpsTime) -> GpsTime {
    let prev_ms = prev.0 / RINEX_TICKS_PER_MS;
    let prev_week = floor_div(prev_ms, WEEK_MS);
    let mut best = continuity_candidate(prev_week, offsets[0], prev_ms);
    let mut best_diff = (best.0 / RINEX_TICKS_PER_MS - prev_ms).abs();
    for &offset in &offsets[1..] {
        let t = continuity_candidate(prev_week, offset, prev_ms);
        let diff = (t.0 / RINEX_TICKS_PER_MS - prev_ms).abs();
        if diff < best_diff {
            best = t;
            best_diff = diff;
        }
    }
    best
}

fn continuity_candidate(prev_week: i64, offset: i64, prev_ms: i64) -> GpsTime {
    let mut cand_ms = prev_week * WEEK_MS + offset;
    let diff = cand_ms - prev_ms;
    if diff < -HALF_WEEK_MS {
        cand_ms += WEEK_MS;
    } else if diff > HALF_WEEK_MS {
        cand_ms -= WEEK_MS;
    }
    GpsTime(cand_ms * RINEX_TICKS_PER_MS)
}

// ---- frame scanning ----

/// Iterator over CRC-valid RTCM frames in a byte buffer, via `rtcm-rs`'s framer.
/// Bytes that do not start a valid frame are skipped (resync).
pub struct Frames<'a> {
    data: &'a [u8],
    pos: usize,
}

pub fn frames(data: &[u8]) -> Frames<'_> {
    Frames { data, pos: 0 }
}

impl<'a> Iterator for Frames<'a> {
    type Item = &'a [u8];
    fn next(&mut self) -> Option<&'a [u8]> {
        if self.pos >= self.data.len() {
            return None;
        }
        let (consumed, frame) = next_msg_frame(&self.data[self.pos..]);
        match frame {
            Some(mf) => {
                let len = mf.frame_len();
                let start = self.pos + consumed - len;
                let slice = &self.data[start..start + len];
                self.pos += consumed;
                Some(slice)
            }
            None => {
                self.pos = self.data.len();
                None
            }
        }
    }
}

/// Byte offset of the first CRC-valid RTCM frame, for raw-stream family
/// detection. `None` if the buffer holds no complete valid frame.
pub fn first_frame_pos(data: &[u8]) -> Option<usize> {
    let (consumed, frame) = next_msg_frame(data);
    frame.map(|m| consumed - m.frame_len())
}

/// The 12-bit RTCM message number from a frame header.
pub fn extract_msg_type(frame: &[u8]) -> u16 {
    if frame.len() <= 5 {
        return 0;
    }
    ((frame[3] as u16) << 4) | ((frame[4] as u16) >> 4)
}

pub fn is_msm7_frame(frame: &[u8]) -> bool {
    let mt = extract_msg_type(frame);
    (1071..=1137).contains(&mt) && mt % 10 == 7
}

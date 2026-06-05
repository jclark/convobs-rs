//! RTCM MSM7 -> RINEX conversion, ported from `gps/lib/rnxrtcm/rtcm.go`,
//! `gps/lib/rtcmbin/{mt,rinex}.go`, with a direct bit reader replacing Go's
//! reflection-based `bitsenc`.

use crate::crc24q;
use crate::freq::signal_frequency_hz;
use crate::obs::*;
use crate::sink::Sink;
use std::collections::HashMap;

const SPEED_OF_LIGHT: f64 = 299792458.0;
const RANGE_MS: f64 = SPEED_OF_LIGHT * 0.001;
const P2_10: f64 = 9.765625e-4;
const P2_29: f64 = 1.862645149230957e-9;
const P2_31: f64 = 4.656612873077393e-10;
const RINEX_TICKS_PER_MS: i64 = 10000;
const SECOND_MS: i64 = 1000;
const HOUR_MS: i64 = 60 * 60 * SECOND_MS;
const DAY_MS: i64 = 24 * HOUR_MS;
const WEEK_MS: i64 = 7 * DAY_MS;
const HALF_WEEK_MS: i64 = WEEK_MS / 2;
const BDT_OFFSET_MS: i64 = 14 * SECOND_MS;
const GLONASS_UTC_OFFSET_MS: i64 = 3 * HOUR_MS;
const DEFAULT_GPS_UTC_MS: i64 = 18 * SECOND_MS;
const METER: f64 = 10000.0;

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
    fn from_msg_num(n: u16) -> Option<Gnss> {
        match (n - 1) / 10 {
            107 => Some(Gnss::Gps),
            108 => Some(Gnss::Glonass),
            109 => Some(Gnss::Galileo),
            110 => Some(Gnss::Sbas),
            111 => Some(Gnss::Qzss),
            112 => Some(Gnss::Beidou),
            113 => Some(Gnss::Irnss),
            _ => None,
        }
    }
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
// Bit reader
// ---------------------------------------------------------------------------

struct BitReader<'a> {
    data: &'a [u8],
    bitpos: usize,
}

impl<'a> BitReader<'a> {
    fn new(data: &'a [u8]) -> Self {
        BitReader { data, bitpos: 0 }
    }
    fn remaining(&self) -> usize {
        self.data.len() * 8 - self.bitpos.min(self.data.len() * 8)
    }
    fn u(&mut self, n: u32) -> u64 {
        let mut v = 0u64;
        let mut remaining = n;
        while remaining > 0 {
            let byte_idx = self.bitpos >> 3;
            let bit_in_byte = (self.bitpos & 7) as u32;
            let avail = 8 - bit_in_byte;
            let take = remaining.min(avail);
            let byte = *self.data.get(byte_idx).unwrap_or(&0) as u64;
            let shifted = (byte >> (avail - take)) & ((1u64 << take) - 1);
            v = (v << take) | shifted;
            self.bitpos += take as usize;
            remaining -= take;
        }
        v
    }
    fn i(&mut self, n: u32) -> i64 {
        let v = self.u(n);
        let sign = 1u64 << (n - 1);
        if v & sign != 0 {
            (v as i64) - (1i64 << n)
        } else {
            v as i64
        }
    }
    fn bit(&mut self) -> bool {
        self.u(1) != 0
    }
}

// ---------------------------------------------------------------------------
// Parsed messages
// ---------------------------------------------------------------------------

struct Msm7 {
    msg_num: u16,
    epoch_time: u32,
    cell_mask: u64,
    nsat: usize,
    nsig: usize,
    sats: Vec<u8>,
    sigs: Vec<u8>,
    range_int: Vec<u8>,
    ext_info: Vec<u8>,
    range_mod: Vec<u16>,
    sat_phase_rate: Vec<i16>,
    pseudorange: Vec<i32>,
    phase_range: Vec<i32>,
    lock_time: Vec<u16>,
    half_cycle: Vec<bool>,
    cnr: Vec<u16>,
    sig_phase_rate: Vec<i16>,
}

enum Parsed {
    Msm7(Msm7),
    Meta(Metadata),
    /// MT1013: leap seconds in ms, plus the metadata to emit.
    Leap(i64, Metadata),
    Other,
}

/// Extracts the 12-bit message type from a full RTCM frame.
pub fn extract_msg_type(frame: &[u8]) -> u16 {
    if frame.len() <= 6 {
        return 0;
    }
    ((frame[3] as u16) << 4) | ((frame[4] as u16) >> 4)
}

pub fn is_msm7_frame(frame: &[u8]) -> bool {
    let mt = extract_msg_type(frame);
    is_msm(mt) && mt % 10 == 7
}

fn is_msm(mt: u16) -> bool {
    let m = mt % 10;
    (1071..=1137).contains(&mt) && (1..=7).contains(&m)
}

fn payload_of(frame: &[u8]) -> &[u8] {
    &frame[3..frame.len() - 3]
}

fn parse_msg(frame: &[u8]) -> Parsed {
    let mt = extract_msg_type(frame);
    let payload = payload_of(frame);
    match mt {
        1005 => parse_1005(payload).map(Parsed::Meta).unwrap_or(Parsed::Other),
        1006 => parse_1006(payload).map(Parsed::Meta).unwrap_or(Parsed::Other),
        1007 => parse_1007(payload).map(Parsed::Meta).unwrap_or(Parsed::Other),
        1008 => parse_1008(payload).map(Parsed::Meta).unwrap_or(Parsed::Other),
        1013 => parse_1013(payload).unwrap_or(Parsed::Other),
        1033 => parse_1033(payload).map(Parsed::Meta).unwrap_or(Parsed::Other),
        1230 => parse_1230(payload).map(Parsed::Meta).unwrap_or(Parsed::Other),
        _ => {
            if is_msm(mt) && mt % 10 == 7 {
                parse_msm7(mt, payload).map(Parsed::Msm7).unwrap_or(Parsed::Other)
            } else {
                Parsed::Other
            }
        }
    }
}

fn parse_msm7(mt: u16, payload: &[u8]) -> Option<Msm7> {
    let mut r = BitReader::new(payload);
    let _msg_num = r.u(12) as u16;
    let _station = r.u(12);
    let epoch_time = r.u(30) as u32;
    let _multiple = r.bit();
    let _iods = r.u(3);
    let _reserved = r.u(7);
    let _clock_steering = r.u(2);
    let _ext_clock = r.u(2);
    let _div_free = r.bit();
    let _smoothing = r.u(3);
    let sat_mask = r.u(64);
    let sig_mask = r.u(32) as u32;

    let nsat = sat_mask.count_ones() as usize;
    let nsig = sig_mask.count_ones() as usize;
    let cell_bits = nsat * nsig;
    if cell_bits > 64 {
        return None;
    }
    let cell_mask = r.u(cell_bits as u32);
    let ncell = cell_mask.count_ones() as usize;

    // Bits required after the cell mask, for a basic completeness check.
    let need = nsat * (8 + 4 + 10 + 14) + ncell * (20 + 24 + 10 + 1 + 10 + 15);
    if r.remaining() < need {
        return None;
    }

    let sats = mask_bits_64(sat_mask, 64);
    let sigs = mask_bits_32(sig_mask);

    let range_int: Vec<u8> = (0..nsat).map(|_| r.u(8) as u8).collect();
    let ext_info: Vec<u8> = (0..nsat).map(|_| r.u(4) as u8).collect();
    let range_mod: Vec<u16> = (0..nsat).map(|_| r.u(10) as u16).collect();
    let sat_phase_rate: Vec<i16> = (0..nsat).map(|_| r.i(14) as i16).collect();
    let pseudorange: Vec<i32> = (0..ncell).map(|_| r.i(20) as i32).collect();
    let phase_range: Vec<i32> = (0..ncell).map(|_| r.i(24) as i32).collect();
    let lock_time: Vec<u16> = (0..ncell).map(|_| r.u(10) as u16).collect();
    let half_cycle: Vec<bool> = (0..ncell).map(|_| r.bit()).collect();
    let cnr: Vec<u16> = (0..ncell).map(|_| r.u(10) as u16).collect();
    let sig_phase_rate: Vec<i16> = (0..ncell).map(|_| r.i(15) as i16).collect();

    Some(Msm7 {
        msg_num: mt,
        epoch_time,
        cell_mask,
        nsat,
        nsig,
        sats,
        sigs,
        range_int,
        ext_info,
        range_mod,
        sat_phase_rate,
        pseudorange,
        phase_range,
        lock_time,
        half_cycle,
        cnr,
        sig_phase_rate,
    })
}

/// Satellite IDs (1-based) from a 64-bit mask, MSB first.
fn mask_bits_64(mask: u64, width: u32) -> Vec<u8> {
    let mut out = Vec::with_capacity(mask.count_ones() as usize);
    for i in 0..width {
        if mask >> (width - 1 - i) & 1 != 0 {
            out.push((i + 1) as u8);
        }
    }
    out
}

fn mask_bits_32(mask: u32) -> Vec<u8> {
    let mut out = Vec::with_capacity(mask.count_ones() as usize);
    for i in 0..32u32 {
        if mask >> (31 - i) & 1 != 0 {
            out.push((i + 1) as u8);
        }
    }
    out
}

// ---- metadata message decode ----

fn skip_header(r: &mut BitReader) -> u16 {
    let _msg_num = r.u(12);
    let station = r.u(12) as u16;
    station
}

fn parse_1005(payload: &[u8]) -> Option<Metadata> {
    let mut r = BitReader::new(payload);
    let station = skip_header(&mut r);
    let _itrf = r.u(6);
    let _gps = r.bit();
    let _glo = r.bit();
    let _gal = r.bit();
    let _ref = r.bit();
    let x = r.i(38);
    let _single = r.bit();
    let _reserved = r.u(1);
    let y = r.i(38);
    let _quarter = r.u(2);
    let z = r.i(38);
    let mut m = Metadata::default();
    m.approx_position = Some([x as f64 / METER, y as f64 / METER, z as f64 / METER]);
    m.marker.number = station.to_string();
    Some(m)
}

fn parse_1006(payload: &[u8]) -> Option<Metadata> {
    let mut r = BitReader::new(payload);
    let station = skip_header(&mut r);
    let _itrf = r.u(6);
    let _gps = r.bit();
    let _glo = r.bit();
    let _gal = r.bit();
    let _ref = r.bit();
    let x = r.i(38);
    let _single = r.bit();
    let _reserved = r.u(1);
    let y = r.i(38);
    let _quarter = r.u(2);
    let z = r.i(38);
    let height = r.u(16);
    let mut m = Metadata::default();
    m.approx_position = Some([x as f64 / METER, y as f64 / METER, z as f64 / METER]);
    m.antenna_delta = Some([height as f64 / METER, 0.0, 0.0]);
    m.marker.number = station.to_string();
    Some(m)
}

fn read_ascii(r: &mut BitReader, n: usize) -> String {
    let mut bytes = Vec::with_capacity(n);
    for _ in 0..n {
        bytes.push(r.u(8) as u8);
    }
    clean_ascii(&bytes)
}

fn clean_ascii(b: &[u8]) -> String {
    let end = b
        .iter()
        .rposition(|&c| c != 0 && c != b' ')
        .map_or(0, |i| i + 1);
    String::from_utf8_lossy(&b[..end]).into_owned()
}

fn parse_1007(payload: &[u8]) -> Option<Metadata> {
    let mut r = BitReader::new(payload);
    let station = skip_header(&mut r);
    let n = r.u(8) as usize;
    let descriptor = read_ascii(&mut r, n);
    let mut m = Metadata::default();
    m.antenna.type_ = descriptor;
    m.marker.number = station.to_string();
    Some(m)
}

fn parse_1008(payload: &[u8]) -> Option<Metadata> {
    let mut r = BitReader::new(payload);
    let station = skip_header(&mut r);
    let n = r.u(8) as usize;
    let descriptor = read_ascii(&mut r, n);
    let _setup = r.u(8);
    let mn = r.u(8) as usize;
    let serial = read_ascii(&mut r, mn);
    let mut m = Metadata::default();
    m.antenna.type_ = descriptor;
    m.antenna.number = serial;
    m.marker.number = station.to_string();
    Some(m)
}

fn parse_1033(payload: &[u8]) -> Option<Metadata> {
    let mut r = BitReader::new(payload);
    let station = skip_header(&mut r);
    let n = r.u(8) as usize;
    let descriptor = read_ascii(&mut r, n);
    let _setup = r.u(8);
    let mn = r.u(8) as usize;
    let serial = read_ascii(&mut r, mn);
    let ri = r.u(8) as usize;
    let rx_type = read_ascii(&mut r, ri);
    let fj = r.u(8) as usize;
    let firmware = read_ascii(&mut r, fj);
    let rk = r.u(8) as usize;
    let rx_serial = read_ascii(&mut r, rk);
    let mut m = Metadata::default();
    m.antenna.type_ = descriptor;
    m.antenna.number = serial;
    m.receiver = Receiver {
        number: rx_serial,
        type_: rx_type,
        version: firmware,
    };
    m.marker.number = station.to_string();
    Some(m)
}

fn parse_1013(payload: &[u8]) -> Option<Parsed> {
    let mut r = BitReader::new(payload);
    let station = skip_header(&mut r);
    let _mjd = r.u(16);
    let _sod = r.u(17);
    let _nm = r.u(5);
    let leap = r.u(8) as i16;
    let mut m = Metadata::default();
    m.leap_seconds = Some(leap);
    m.marker.number = station.to_string();
    Some(Parsed::Leap(leap as i64 * SECOND_MS, m))
}

fn parse_1230(payload: &[u8]) -> Option<Metadata> {
    let mut r = BitReader::new(payload);
    let station = skip_header(&mut r);
    let mut m = Metadata::default();
    m.marker.number = station.to_string();
    Some(m)
}

// ---- RINEX mapping tables (rtcmbin/rinex.go) ----

fn rinex_sat_num(gnss: Gnss, sat_id: u8) -> u8 {
    match gnss {
        Gnss::Gps => {
            if (1..=63).contains(&sat_id) {
                sat_id
            } else {
                0
            }
        }
        Gnss::Glonass => {
            if (1..=24).contains(&sat_id) {
                sat_id
            } else {
                0
            }
        }
        Gnss::Galileo => {
            if (1..=50).contains(&sat_id) {
                sat_id
            } else {
                0
            }
        }
        Gnss::Sbas => {
            if (1..=39).contains(&sat_id) {
                sat_id + 19
            } else {
                0
            }
        }
        Gnss::Qzss => {
            if (1..=10).contains(&sat_id) {
                sat_id
            } else {
                0
            }
        }
        Gnss::Beidou => {
            if (1..=63).contains(&sat_id) {
                sat_id
            } else {
                0
            }
        }
        Gnss::Irnss => {
            if (1..=14).contains(&sat_id) {
                sat_id
            } else {
                0
            }
        }
    }
}

fn rinex_sig(gnss: Gnss, sig_id: u8) -> Option<[u8; 2]> {
    let s: &[u8; 2] = match gnss {
        Gnss::Gps => match sig_id {
            2 => b"1C",
            3 => b"1P",
            4 => b"1W",
            8 => b"2C",
            9 => b"2P",
            10 => b"2W",
            15 => b"2S",
            16 => b"2L",
            17 => b"2X",
            22 => b"5I",
            23 => b"5Q",
            24 => b"5X",
            30 => b"1S",
            31 => b"1L",
            32 => b"1X",
            _ => return None,
        },
        Gnss::Glonass => match sig_id {
            2 => b"1C",
            3 => b"1P",
            8 => b"2C",
            9 => b"2P",
            _ => return None,
        },
        Gnss::Galileo => match sig_id {
            2 => b"1C",
            3 => b"1A",
            4 => b"1B",
            5 => b"1X",
            6 => b"1Z",
            8 => b"6C",
            9 => b"6A",
            10 => b"6B",
            11 => b"6X",
            12 => b"6Z",
            14 => b"7I",
            15 => b"7Q",
            16 => b"7X",
            18 => b"8I",
            19 => b"8Q",
            20 => b"8X",
            22 => b"5I",
            23 => b"5Q",
            24 => b"5X",
            _ => return None,
        },
        Gnss::Sbas => match sig_id {
            2 => b"1C",
            22 => b"5I",
            23 => b"5Q",
            24 => b"5X",
            _ => return None,
        },
        Gnss::Qzss => match sig_id {
            2 => b"1C",
            9 => b"6S",
            10 => b"6L",
            11 => b"6X",
            15 => b"2S",
            16 => b"2L",
            17 => b"2X",
            22 => b"5I",
            23 => b"5Q",
            24 => b"5X",
            30 => b"1S",
            31 => b"1L",
            32 => b"1X",
            _ => return None,
        },
        Gnss::Beidou => match sig_id {
            2 => b"2I",
            3 => b"2Q",
            4 => b"2X",
            8 => b"6I",
            9 => b"6Q",
            10 => b"6X",
            14 => b"7I",
            15 => b"7Q",
            16 => b"7X",
            22 => b"5D",
            23 => b"5P",
            24 => b"5X",
            25 => b"7D",
            30 => b"1D",
            31 => b"1P",
            32 => b"1X",
            _ => return None,
        },
        Gnss::Irnss => match sig_id {
            8 => b"9A",
            22 => b"5A",
            _ => return None,
        },
    };
    Some(*s)
}

// ---------------------------------------------------------------------------
// Converter
// ---------------------------------------------------------------------------

pub struct Converter<S: Sink> {
    sink: S,
    opts: Options,
    week: TimeInterval,
    leap_ms: i64,
    time: HashMap<Gnss, GpsTime>,
    lock: HashMap<SignalKey, u16>,
    arc: HashMap<SignalKey, u32>,
    slip: HashMap<SignalKey, bool>,
}

impl<S: Sink> Converter<S> {
    pub fn new(sink: S, opts: Options) -> Self {
        Converter {
            sink,
            opts,
            week: TimeInterval::default(),
            leap_ms: DEFAULT_GPS_UTC_MS,
            time: HashMap::new(),
            lock: HashMap::new(),
            arc: HashMap::new(),
            slip: HashMap::new(),
        }
    }

    /// Converts one RTCM frame. `week` is the per-message epoch constraint.
    /// Returns whether an observation/metadata record was produced.
    pub fn convert_frame(&mut self, frame: &[u8], week: TimeInterval) -> Result<bool, String> {
        match parse_msg(frame) {
            Parsed::Msm7(m) => self.convert_msm7(&m, week),
            Parsed::Leap(leap_ms, meta) => {
                self.set_week(week);
                self.leap_ms = leap_ms;
                self.emit_metadata(&meta)
            }
            Parsed::Meta(meta) => {
                self.set_week(week);
                self.emit_metadata(&meta)
            }
            Parsed::Other => {
                self.set_week(week);
                Ok(false)
            }
        }
    }

    /// Forwards metadata directly to the sink (for the initial CLI metadata).
    pub fn sink_metadata(&mut self, m: &Metadata) -> Result<(), String> {
        self.sink.metadata(m).map_err(|e| e.to_string())
    }

    /// Flushes the underlying sink.
    pub fn flush(&mut self) -> Result<(), String> {
        self.sink.flush().map_err(|e| e.to_string())
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

    fn convert_msm7(&mut self, m: &Msm7, week: TimeInterval) -> Result<bool, String> {
        self.set_week(week);
        let gnss = Gnss::from_msg_num(m.msg_num).ok_or_else(|| {
            format!("unknown RTCM MSM7 GNSS for message {}", m.msg_num)
        })?;
        let t = self.resolve_time(m, gnss, week)?;
        let ncell_bits = m.nsat * m.nsig;
        if ncell_bits > 64 {
            return Err(format!("RTCM MSM7 cell mask has {} bits, max 64", ncell_bits));
        }
        self.time.insert(gnss, t);
        let mut seen = false;
        let mut cell = 0usize;
        for (i, &sat_id) in m.sats.iter().enumerate() {
            for (j, &sig_id) in m.sigs.iter().enumerate() {
                let k = i * m.sigs.len() + j;
                if !cell_set(m.cell_mask, ncell_bits, k) {
                    continue;
                }
                if self.convert_cell(t, gnss, m, i, cell, sat_id, sig_id)? {
                    seen = true;
                }
                cell += 1;
            }
        }
        Ok(seen)
    }

    fn convert_cell(
        &mut self,
        t: GpsTime,
        gnss: Gnss,
        m: &Msm7,
        sat_index: usize,
        cell_index: usize,
        sat_id: u8,
        sig_id: u8,
    ) -> Result<bool, String> {
        let sys = gnss.rinex_sys();
        let sat_num = rinex_sat_num(gnss, sat_id);
        let sig = match rinex_sig(gnss, sig_id) {
            Some(s) => s,
            None => return Ok(false),
        };
        if sat_num == 0 {
            return Ok(false);
        }
        let sat = SatId::format(sys, sat_num);
        let sig = SigId(sig);
        let mut o = SignalObservation {
            t,
            sat,
            sig,
            v: SignalValues::default(),
        };
        let mut frq: Option<i8> = None;
        if let Some(v) = glonass_frequency_channel(gnss, m, sat_index) {
            o.v.frq = Some(v);
            frq = Some(v);
        }
        if let Some(pr) = pseudorange(m, sat_index, cell_index) {
            o.v.pr = Some(pr);
        }
        let freq = signal_frequency_hz(sys, sig, frq);
        if let Some(cp) = carrier_phase(m, sat_index, cell_index, freq) {
            o.v.cp = Some(cp);
        }
        if let Some(dop) = doppler(m, sat_index, cell_index, freq, self.opts.use_spec_phase_range_rate_sign) {
            if !self.opts.omit_zero_do || dop != 0.0 {
                o.v.dop = Some(dop);
            }
        }
        if let Some(v) = cn0(m, cell_index) {
            o.v.cn0 = Some(v);
        }
        let (arc, hc) = self.arc_hc(sat, sig, m, cell_index, o.v.cp.is_some());
        o.v.arc = arc;
        o.v.hc = hc;
        if !o.has_any_code() {
            return Ok(false);
        }
        self.sink.observation(&o).map_err(|e| e.to_string())?;
        Ok(true)
    }

    fn arc_hc(&mut self, sat: SatId, sig: SigId, m: &Msm7, cell_index: usize, has_phase: bool) -> (u32, bool) {
        let k = SignalKey { sat, sig };
        let mut ll = *self.slip.get(&k).unwrap_or(&false);
        if let Some(&cur) = m.lock_time.get(cell_index) {
            let prev = *self.lock.get(&k).unwrap_or(&0);
            if cur < prev || (cur == 0 && prev == 0) {
                ll = true;
            }
            self.lock.insert(k, cur);
        }
        if ll {
            *self.arc.entry(k).or_insert(0) += 1;
        }
        if ll && !has_phase {
            self.slip.insert(k, true);
        } else {
            self.slip.remove(&k);
        }
        let mut hc = false;
        if let Some(&half) = m.half_cycle.get(cell_index) {
            if half {
                hc = true;
            }
        }
        (*self.arc.get(&k).unwrap_or(&0), hc)
    }

    fn resolve_time(&self, m: &Msm7, gnss: Gnss, week: TimeInterval) -> Result<GpsTime, String> {
        let offsets = self.epoch_week_offsets(m, gnss)?;
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

    fn epoch_week_offsets(&self, m: &Msm7, gnss: Gnss) -> Result<Vec<i64>, String> {
        match gnss {
            Gnss::Glonass => self.glonass_epoch_week_offsets(m.epoch_time),
            Gnss::Beidou => {
                if m.epoch_time as i64 >= WEEK_MS {
                    return Err(format!("invalid RTCM MSM7 epoch time {}", m.epoch_time));
                }
                Ok(vec![m.epoch_time as i64 + BDT_OFFSET_MS])
            }
            _ => {
                if m.epoch_time as i64 >= WEEK_MS {
                    return Err(format!("invalid RTCM MSM7 epoch time {}", m.epoch_time));
                }
                Ok(vec![m.epoch_time as i64])
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

fn cell_set(mask: u64, ncell: usize, cell: usize) -> bool {
    if ncell == 0 || cell >= ncell {
        return false;
    }
    mask >> (ncell - 1 - cell) & 1 != 0
}

fn rough_range(m: &Msm7, sat_index: usize) -> Option<f64> {
    let rint = *m.range_int.get(sat_index)?;
    let rmod = *m.range_mod.get(sat_index)?;
    if rint == 255 {
        return None;
    }
    Some(rint as f64 * RANGE_MS + rmod as f64 * P2_10 * RANGE_MS)
}

fn pseudorange(m: &Msm7, sat_index: usize, cell_index: usize) -> Option<f64> {
    let r = rough_range(m, sat_index)?;
    let fine = *m.pseudorange.get(cell_index)?;
    if fine == -524288 {
        return None;
    }
    Some(r + fine as f64 * P2_29 * RANGE_MS)
}

fn carrier_phase(m: &Msm7, sat_index: usize, cell_index: usize, freq: Option<f64>) -> Option<f64> {
    let freq = freq?;
    let r = rough_range(m, sat_index)?;
    let fine = *m.phase_range.get(cell_index)?;
    if fine == -8388608 {
        return None;
    }
    Some((r + fine as f64 * P2_31 * RANGE_MS) * freq / SPEED_OF_LIGHT)
}

fn doppler(m: &Msm7, sat_index: usize, cell_index: usize, freq: Option<f64>, spec_sign: bool) -> Option<f64> {
    let freq = freq?;
    let rough = *m.sat_phase_rate.get(sat_index)?;
    let fine = *m.sig_phase_rate.get(cell_index)?;
    if rough == -8192 || fine == -16384 {
        return None;
    }
    let mut prr = rough as f64 + fine as f64 * 0.0001;
    if spec_sign {
        prr = -prr;
    }
    let d = ((prr * freq / SPEED_OF_LIGHT) as f32) as f64;
    Some(d)
}

fn cn0(m: &Msm7, cell_index: usize) -> Option<f32> {
    let v = *m.cnr.get(cell_index)?;
    if v == 0 {
        return None;
    }
    Some(v as f32 * 0.0625)
}

fn glonass_frequency_channel(gnss: Gnss, m: &Msm7, sat_index: usize) -> Option<i8> {
    if gnss != Gnss::Glonass {
        return None;
    }
    let v = *m.ext_info.get(sat_index)?;
    if v > 13 {
        return None;
    }
    Some(v as i8 - 7)
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
            let ticks = (((w + i) * WEEK_MS + offset) * RINEX_TICKS_PER_MS) as i64;
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

/// Iterator over CRC-valid RTCM frames in a byte buffer. Bytes that do not
/// start a valid frame are skipped (resync), matching the scanner's behaviour
/// on clean single-frame packet-log payloads.
pub struct Frames<'a> {
    data: &'a [u8],
    i: usize,
}

pub fn frames(data: &[u8]) -> Frames<'_> {
    Frames { data, i: 0 }
}

impl<'a> Iterator for Frames<'a> {
    type Item = &'a [u8];
    fn next(&mut self) -> Option<&'a [u8]> {
        let data = self.data;
        while self.i + 6 <= data.len() {
            if data[self.i] != 0xD3 || data[self.i + 1] & 0xFC != 0 {
                self.i += 1;
                continue;
            }
            let len = (((data[self.i + 1] & 0x03) as usize) << 8) | data[self.i + 2] as usize;
            let frame_end = self.i + 3 + len + 3;
            if frame_end > data.len() {
                return None;
            }
            let frame = &data[self.i..frame_end];
            let crc_calc = crc24q::checksum(&frame[..3 + len]);
            let crc_recv = ((frame[3 + len] as u32) << 16)
                | ((frame[3 + len + 1] as u32) << 8)
                | frame[3 + len + 2] as u32;
            if crc_calc == crc_recv {
                self.i = frame_end;
                return Some(frame);
            } else {
                self.i += 1;
            }
        }
        None
    }
}

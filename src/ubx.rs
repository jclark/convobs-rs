//! UBX RXM-RAWX -> RINEX conversion, ported from `gps/lib/rnxubx/ubx.go`,
//! `gps/lib/ubxbin/{rxm,rinex,common}.go`.

use crate::obs::*;
use crate::sink::Sink;
use std::collections::HashMap;

// GNSS ids (ubxbin/common.go).
const GPS: u8 = 0;
const SBAS: u8 = 1;
const GAL: u8 = 2;
const BDS: u8 = 3;
const QZSS: u8 = 5;
const GLO: u8 = 6;
const NAVIC: u8 = 7;

// TrkStat flags.
const PR_VALID: u8 = 1;
const CP_VALID: u8 = 2;
const HALF_CYC: u8 = 4;
const SUB_HALF_CYC: u8 = 8;
const CP_STD_MASK: u8 = 0x0F;

const RAWX_CLASS: u8 = 0x02;
const RAWX_ID: u8 = 0x15;

#[derive(Clone, Copy)]
pub struct Options {
    pub slip_threshold: u8,
    pub bds_geo_half_cycle: bool,
}

#[derive(Clone, Copy, Default)]
struct SignalState {
    lock: u16,
    sub_half_cyc: bool,
    arc: u32,
    pending: bool,
    seen: bool,
}

struct Meas {
    pr_mes: f64,
    cp_mes: f64,
    do_mes: f32,
    gnss_id: u8,
    sv_id: u8,
    sig_id: u8,
    freq_id: u8,
    lock_time: u16,
    cno: u8,
    cp_stdev: u8,
    trk_stat: u8,
}

struct Rawx {
    rcv_tow: f64,
    week: u16,
    meas: Vec<Meas>,
}

pub struct Converter<S: Sink> {
    opts: Options,
    sink: S,
    state: HashMap<SignalKey, SignalState>,
}

impl<S: Sink> Converter<S> {
    pub fn new(sink: S, mut opts: Options) -> Self {
        if opts.slip_threshold == 0 {
            opts.slip_threshold = 15;
        }
        Converter {
            opts,
            sink,
            state: HashMap::new(),
        }
    }

    /// Converts one UBX frame, returning whether it was an RXM-RAWX message.
    pub fn convert_frame(&mut self, frame: &[u8]) -> Result<bool, String> {
        if packet_msg_is_rawx(frame) {
            if let Some(rawx) = parse_rawx(frame) {
                self.convert_rawx(&rawx)?;
                return Ok(true);
            }
        }
        Ok(false)
    }

    pub fn sink_metadata(&mut self, m: &Metadata) -> Result<(), String> {
        self.sink.metadata(m).map_err(|e| e.to_string())
    }

    pub fn flush(&mut self) -> Result<(), String> {
        self.sink.flush().map_err(|e| e.to_string())
    }

    fn convert_rawx(&mut self, m: &Rawx) -> Result<(), String> {
        let t = GpsTime::from_gps_week_seconds(m.week as i64, m.rcv_tow);
        for meas in &m.meas {
            if let Some(obs) = self.observation(t, meas) {
                self.sink.observation(&obs).map_err(|e| e.to_string())?;
            }
        }
        Ok(())
    }

    fn observation(&mut self, t: GpsTime, meas: &Meas) -> Option<SignalObservation> {
        let sys = rinex_sys(meas.gnss_id)?;
        let sat_num = rinex_sat_num(meas.gnss_id, meas.sv_id);
        let sig = rinex_sig(meas.gnss_id, meas.sig_id)?;
        if sat_num == 0 {
            return None;
        }
        let sat = SatId::format(sys, sat_num);
        let sig = SigId(sig);
        let mut o = SignalObservation {
            t,
            sat,
            sig,
            v: SignalValues::default(),
        };
        if meas.gnss_id == GLO {
            o.v.frq = Some(meas.freq_id as i8 - 7);
        }
        if meas.trk_stat & PR_VALID != 0 && meas.pr_mes.is_finite() {
            o.v.pr = Some(meas.pr_mes);
        }
        let cp = carrier_phase(meas, self.opts.bds_geo_half_cycle);
        let (arc, hc) = self.arc_hc(sat, sig, meas, cp.is_some());
        if let Some(cp) = cp {
            o.v.cp = Some(cp);
        }
        o.v.arc = arc;
        o.v.hc = hc;
        if meas.do_mes.is_finite() {
            o.v.dop = Some(meas.do_mes as f64);
        }
        if meas.cno != 0 {
            o.v.cn0 = Some(meas.cno as f32);
        }
        if !o.has_any_code() {
            return None;
        }
        Some(o)
    }

    fn arc_hc(&mut self, sat: SatId, sig: SigId, meas: &Meas, phase: bool) -> (u32, bool) {
        let k = SignalKey { sat, sig };
        let mut st = self.state.get(&k).copied().unwrap_or_default();
        let sub = meas.trk_stat & SUB_HALF_CYC != 0;
        let sub_changed = sub != st.sub_half_cyc;
        let mut ll = false;
        if meas.lock_time == 0
            || (st.seen && meas.lock_time < st.lock)
            || sub_changed
            || meas.cp_stdev & CP_STD_MASK >= self.opts.slip_threshold
        {
            st.pending = true;
        }
        if sub_changed {
            ll = true;
        }
        if phase && st.pending {
            ll = true;
            st.pending = false;
        }
        if ll {
            st.arc += 1;
        }
        let mut hc = false;
        if phase && half_cycle_unresolved(meas) {
            hc = true;
        }
        st.lock = meas.lock_time;
        st.sub_half_cyc = sub;
        st.seen = true;
        self.state.insert(k, st);
        (st.arc, hc)
    }
}

fn carrier_phase(meas: &Meas, bds_geo_half_cycle: bool) -> Option<f64> {
    if meas.trk_stat & CP_VALID == 0 || !meas.cp_mes.is_finite() {
        return None;
    }
    let mut cp = meas.cp_mes;
    if bds_geo_half_cycle && is_bds_geo(meas) {
        cp += 0.5;
    }
    Some(cp)
}

fn is_bds_geo(meas: &Meas) -> bool {
    meas.gnss_id == BDS && (meas.sv_id <= 5 || meas.sv_id >= 59)
}

fn half_cycle_unresolved(meas: &Meas) -> bool {
    if meas.gnss_id == SBAS {
        meas.lock_time <= 8000
    } else {
        meas.trk_stat & HALF_CYC == 0
    }
}

fn rinex_sys(gnss_id: u8) -> Option<u8> {
    Some(match gnss_id {
        GPS => b'G',
        SBAS => b'S',
        GAL => b'E',
        BDS => b'C',
        QZSS => b'J',
        GLO => b'R',
        NAVIC => b'I',
        _ => return None,
    })
}

fn rinex_sat_num(gnss_id: u8, sv_id: u8) -> u8 {
    match gnss_id {
        SBAS => {
            if sv_id >= 120 {
                sv_id - 100
            } else {
                sv_id
            }
        }
        GLO => {
            if sv_id == 255 {
                0
            } else {
                sv_id
            }
        }
        _ => sv_id,
    }
}

fn rinex_sig(gnss_id: u8, sig_id: u8) -> Option<[u8; 2]> {
    let s: &[u8; 2] = match gnss_id {
        GPS => match sig_id {
            0 => b"1C",
            3 => b"2L",
            4 => b"2S",
            6 => b"5I",
            7 => b"5Q",
            _ => return None,
        },
        SBAS => match sig_id {
            0 => b"1C",
            _ => return None,
        },
        GAL => match sig_id {
            0 => b"1C",
            1 => b"1B",
            3 => b"5I",
            4 => b"5Q",
            5 => b"7I",
            6 => b"7Q",
            8 => b"6B",
            9 => b"6C",
            10 => b"6A",
            _ => return None,
        },
        BDS => match sig_id {
            0 => b"2I",
            1 => b"2I",
            2 => b"7I",
            3 => b"7I",
            4 => b"6I",
            5 => b"1P",
            6 => b"1D",
            7 => b"5P",
            8 => b"5D",
            10 => b"6I",
            _ => return None,
        },
        QZSS => match sig_id {
            0 => b"1C",
            1 => b"1Z",
            4 => b"2S",
            5 => b"2L",
            8 => b"5I",
            9 => b"5Q",
            12 => b"1E",
            _ => return None,
        },
        GLO => match sig_id {
            0 => b"1C",
            2 => b"2C",
            _ => return None,
        },
        NAVIC => match sig_id {
            0 => b"5A",
            _ => return None,
        },
        _ => return None,
    };
    Some(*s)
}

// ---- parse ----

fn packet_msg_is_rawx(frame: &[u8]) -> bool {
    frame.len() >= 6 && frame[2] == RAWX_CLASS && frame[3] == RAWX_ID
}

fn le_f64(b: &[u8]) -> f64 {
    f64::from_le_bytes(b[..8].try_into().unwrap())
}
fn le_f32(b: &[u8]) -> f32 {
    f32::from_le_bytes(b[..4].try_into().unwrap())
}
fn le_u16(b: &[u8]) -> u16 {
    u16::from_le_bytes(b[..2].try_into().unwrap())
}

fn parse_rawx(frame: &[u8]) -> Option<Rawx> {
    // frame: B5 62 cls id lenLo lenHi payload... ckA ckB
    if frame.len() < 8 {
        return None;
    }
    let len = le_u16(&frame[4..6]) as usize;
    if frame.len() < 6 + len + 2 {
        return None;
    }
    let payload = &frame[6..6 + len];
    if payload.len() < 16 {
        return None;
    }
    let version = payload[13];
    if version != 1 {
        return None;
    }
    let n = (payload.len() - 16) / 32;
    if 16 + n * 32 != payload.len() {
        return None;
    }
    let rcv_tow = le_f64(&payload[0..8]);
    let week = le_u16(&payload[8..10]);
    let mut meas = Vec::with_capacity(n);
    for i in 0..n {
        let m = &payload[16 + i * 32..16 + (i + 1) * 32];
        meas.push(Meas {
            pr_mes: le_f64(&m[0..8]),
            cp_mes: le_f64(&m[8..16]),
            do_mes: le_f32(&m[16..20]),
            gnss_id: m[20],
            sv_id: m[21],
            sig_id: m[22],
            freq_id: m[23],
            lock_time: le_u16(&m[24..26]),
            cno: m[26],
            cp_stdev: m[28],
            trk_stat: m[30],
        });
    }
    Some(Rawx { rcv_tow, week, meas })
}

fn checksum(data: &[u8]) -> (u8, u8) {
    let mut a: u8 = 0;
    let mut b: u8 = 0;
    for &x in data {
        a = a.wrapping_add(x);
        b = b.wrapping_add(a);
    }
    (a, b)
}

/// Iterator over checksum-valid UBX frames in a byte buffer.
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
        while self.i + 8 <= data.len() {
            if data[self.i] != 0xB5 || data[self.i + 1] != 0x62 {
                self.i += 1;
                continue;
            }
            let len = le_u16(&data[self.i + 4..self.i + 6]) as usize;
            let frame_end = self.i + 6 + len + 2;
            if frame_end > data.len() {
                return None;
            }
            let frame = &data[self.i..frame_end];
            let (a, b) = checksum(&frame[2..6 + len]);
            if a == frame[6 + len] && b == frame[6 + len + 1] {
                self.i = frame_end;
                return Some(frame);
            } else {
                self.i += 1;
            }
        }
        None
    }
}

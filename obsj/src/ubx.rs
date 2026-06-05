//! UBX RXM-RAWX → obsj conversion.
//!
//! Framing and field decoding are handled by the `ublox` crate; this module is
//! the converter algorithm on top of it. RXM-RAWX carries pseudorange and
//! carrier phase as `f64` and Doppler as `f32` directly (no scaling), so the
//! values pass through unchanged. Slip detection is emitted as the per-
//! observation loss-of-lock flag, which the [`LossOfLockSink`] turns into `arc`.
//!
//! `ublox` 0.10 exposes the signal id as `reserved2()` (the offset-22 byte).
//!
//! [`LossOfLockSink`]: crate::arc::LossOfLockSink

use crate::obs::*;
use crate::sink::Sink;
use std::collections::HashMap;
use ublox::proto23::PacketRef;
use ublox::{Parser, UbxPacket};

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

#[derive(Clone, Copy)]
pub struct Options {
    pub slip_threshold: u8,
    pub bds_geo_half_cycle: bool,
}

#[derive(Clone, Copy, Default)]
struct SignalState {
    lock: u16,
    sub_half_cyc: bool,
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
    parser: Parser<Vec<u8>>,
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
            parser: Parser::default_proto(),
        }
    }

    pub fn sink_metadata(&mut self, m: &Metadata) -> Result<(), String> {
        self.sink.metadata(m).map_err(|e| e.to_string())
    }

    pub fn flush(&mut self) -> Result<(), String> {
        self.sink.flush().map_err(|e| e.to_string())
    }

    /// Feeds a byte chunk to the UBX framer and converts every RXM-RAWX message
    /// it yields. Returns the number of RXM-RAWX messages converted. The parser
    /// buffers across calls, so a frame split between chunks still parses.
    pub fn convert_chunk(&mut self, data: &[u8]) -> Result<u64, String> {
        // Collect owned measurements first: the parser iterator borrows `self`,
        // so the converter state can only be touched once the iteration ends.
        let mut batch: Vec<Rawx> = Vec::new();
        let mut it = self.parser.consume_ubx(data);
        while let Some(packet) = it.next() {
            if let Ok(UbxPacket::Proto23(PacketRef::RxmRawx(rawx))) = packet {
                let meas = rawx
                    .measurements()
                    .map(|m| Meas {
                        pr_mes: m.pr_mes(),
                        cp_mes: m.cp_mes(),
                        do_mes: m.do_mes(),
                        gnss_id: m.gnss_id(),
                        sv_id: m.sv_id(),
                        sig_id: m.reserved2(),
                        freq_id: m.freq_id(),
                        lock_time: m.lock_time(),
                        cno: m.cno(),
                        cp_stdev: m.cp_stdev().bits(),
                        trk_stat: m.trk_stat().bits(),
                    })
                    .collect();
                batch.push(Rawx {
                    rcv_tow: rawx.rcv_tow(),
                    week: rawx.week(),
                    meas,
                });
            }
        }
        drop(it);

        let count = batch.len() as u64;
        for rawx in &batch {
            self.convert_rawx(rawx)?;
        }
        Ok(count)
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
        let (ll, hc) = self.slip_hc(sat, sig, meas, cp.is_some());
        if let Some(cp) = cp {
            o.v.cp = Some(cp);
        }
        o.v.ll = ll;
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

    /// Detects a carrier-phase slip (loss of lock) and the half-cycle bit for one
    /// measurement. Returns the per-observation `ll` flag; the downstream
    /// [`LossOfLockSink`](crate::arc::LossOfLockSink) turns it into `arc`. A slip
    /// is deferred until the next epoch that carries phase.
    fn slip_hc(&mut self, sat: SatId, sig: SigId, meas: &Meas, phase: bool) -> (bool, bool) {
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
        let mut hc = false;
        if phase && half_cycle_unresolved(meas) {
            hc = true;
        }
        st.lock = meas.lock_time;
        st.sub_half_cyc = sub;
        st.seen = true;
        self.state.insert(k, st);
        (ll, hc)
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

/// The UBX 8-bit Fletcher checksum over class+id+len+payload.
fn checksum(data: &[u8]) -> (u8, u8) {
    let mut a: u8 = 0;
    let mut b: u8 = 0;
    for &x in data {
        a = a.wrapping_add(x);
        b = b.wrapping_add(a);
    }
    (a, b)
}

/// Byte offset of the first checksum-valid UBX frame, for raw-stream family
/// detection. `None` if the buffer holds no complete valid frame.
pub fn first_frame_pos(data: &[u8]) -> Option<usize> {
    let mut i = 0;
    while i + 8 <= data.len() {
        if data[i] == 0xB5 && data[i + 1] == 0x62 {
            let len = u16::from_le_bytes([data[i + 4], data[i + 5]]) as usize;
            let end = i + 6 + len + 2;
            if end <= data.len() {
                let (a, b) = checksum(&data[i + 2..i + 6 + len]);
                if a == data[end - 2] && b == data[end - 1] {
                    return Some(i);
                }
            }
        }
        i += 1;
    }
    None
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

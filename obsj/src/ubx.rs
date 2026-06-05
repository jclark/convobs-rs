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

use crate::error::Result;
use crate::obs::*;
use crate::sink::Sink;
use rustc_hash::FxHashMap;
use ublox::proto23::PacketRef;
use ublox::{Parser, UbxPacket};

// UBX protocol GNSS identifier values used by RXM-RAWX.
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
    state: FxHashMap<SignalKey, SignalState>,
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
            state: FxHashMap::default(),
            parser: Parser::default_proto(),
        }
    }

    pub fn sink_metadata(&mut self, m: &Metadata) -> Result<()> {
        self.sink.metadata(m)?;
        Ok(())
    }

    pub fn flush(&mut self) -> Result<()> {
        self.sink.flush()?;
        Ok(())
    }

    /// Feeds a byte chunk to the UBX framer and converts every RXM-RAWX message
    /// it yields. Returns the number of RXM-RAWX messages converted. The parser
    /// buffers across calls, so a frame split between chunks still parses.
    pub fn convert_chunk(&mut self, data: &[u8]) -> Result<u64> {
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

    fn convert_rawx(&mut self, m: &Rawx) -> Result<()> {
        let t = GpsTime::from_gps_week_seconds(m.week as i64, m.rcv_tow);
        for meas in &m.meas {
            if let Some(obs) = self.observation(t, meas) {
                self.sink.observation(&obs)?;
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

/// Whether a single UBX frame is an RXM-RAWX message (class 0x02, id 0x15),
/// from its header bytes only. Lets the packet-log path skip the ~90% of UBX
/// traffic that is other messages without paying for a full decode.
pub fn is_rawx_frame(frame: &[u8]) -> bool {
    frame.len() >= 4 && frame[0] == 0xB5 && frame[1] == 0x62 && frame[2] == 0x02 && frame[3] == 0x15
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

#[cfg(test)]
mod tests {
    use super::*;

    struct NullSink;
    impl Sink for NullSink {
        fn metadata(&mut self, _: &Metadata) -> std::io::Result<()> {
            Ok(())
        }
        fn observation(&mut self, _: &SignalObservation) -> std::io::Result<()> {
            Ok(())
        }
        fn flush(&mut self) -> std::io::Result<()> {
            Ok(())
        }
    }

    fn converter(opts: Options) -> Converter<NullSink> {
        Converter::new(NullSink, opts)
    }

    fn opts() -> Options {
        Options {
            slip_threshold: 15,
            bds_geo_half_cycle: false,
        }
    }

    fn base_meas() -> Meas {
        Meas {
            pr_mes: 23_956_830.530,
            cp_mes: 125_893_980.172,
            do_mes: 0.0,
            gnss_id: GPS,
            sv_id: 1,
            sig_id: 0,
            freq_id: 0,
            lock_time: 100,
            cno: 0,
            cp_stdev: 0,
            trk_stat: PR_VALID | CP_VALID | HALF_CYC,
        }
    }

    fn key() -> (SatId, SigId) {
        (SatId::format(b'G', 1), SigId(*b"1C"))
    }

    // ---- mapping tables ----

    #[test]
    fn rinex_sys_table() {
        assert_eq!(rinex_sys(GPS), Some(b'G'));
        assert_eq!(rinex_sys(GLO), Some(b'R'));
        assert_eq!(rinex_sys(BDS), Some(b'C'));
        assert_eq!(rinex_sys(NAVIC), Some(b'I'));
        assert_eq!(rinex_sys(99), None);
    }

    #[test]
    fn rinex_sat_num_table() {
        // SBAS PRNs >= 120 collapse to two digits.
        assert_eq!(rinex_sat_num(SBAS, 120), 20);
        assert_eq!(rinex_sat_num(SBAS, 100), 100);
        // GLONASS 255 is "unknown".
        assert_eq!(rinex_sat_num(GLO, 255), 0);
        assert_eq!(rinex_sat_num(GLO, 5), 5);
        assert_eq!(rinex_sat_num(GPS, 3), 3);
    }

    #[test]
    fn rinex_sig_table() {
        assert_eq!(rinex_sig(GPS, 0), Some(*b"1C"));
        assert_eq!(rinex_sig(GPS, 3), Some(*b"2L"));
        assert_eq!(rinex_sig(GPS, 99), None);
        assert_eq!(rinex_sig(GAL, 5), Some(*b"7I"));
        assert_eq!(rinex_sig(BDS, 0), Some(*b"2I"));
        assert_eq!(rinex_sig(BDS, 7), Some(*b"5P"));
        assert_eq!(rinex_sig(BDS, 8), Some(*b"5D"));
        assert_eq!(rinex_sig(BDS, 9), None); // gap in the BDS table
        assert_eq!(rinex_sig(GLO, 2), Some(*b"2C"));
        assert_eq!(rinex_sig(99, 0), None);
    }

    // ---- slip / loss-of-lock ----

    #[test]
    fn slip_unset_for_steady_valid_phase() {
        let mut c = converter(opts());
        let (sat, sig) = key();
        let (ll, hc) = c.slip_hc(sat, sig, &base_meas(), true);
        assert!(!ll && !hc);
    }

    #[test]
    fn slip_set_on_zero_lock() {
        let mut c = converter(opts());
        let (sat, sig) = key();
        let mut m = base_meas();
        m.lock_time = 0;
        let (ll, hc) = c.slip_hc(sat, sig, &m, true);
        assert!(ll && !hc);
    }

    #[test]
    fn slip_deferred_without_phase() {
        // No carrier phase this epoch: the slip is pending, not yet emitted.
        let mut c = converter(opts());
        let (sat, sig) = key();
        let mut m = base_meas();
        m.lock_time = 0;
        m.trk_stat = PR_VALID; // no CP_VALID
        let (ll, hc) = c.slip_hc(sat, sig, &m, false);
        assert!(!ll && !hc);
    }

    #[test]
    fn slip_on_lock_time_decrease() {
        let mut c = converter(opts());
        let (sat, sig) = key();
        let mut m = base_meas();
        m.lock_time = 100;
        assert_eq!(c.slip_hc(sat, sig, &m, true), (false, false));
        m.lock_time = 50; // lock counter went backwards
        assert!(c.slip_hc(sat, sig, &m, true).0);
    }

    #[test]
    fn slip_on_sub_half_cycle_change() {
        let mut c = converter(opts());
        let (sat, sig) = key();
        let m = base_meas();
        assert_eq!(c.slip_hc(sat, sig, &m, true), (false, false));
        let mut m2 = base_meas();
        m2.trk_stat |= SUB_HALF_CYC; // sub-half-cycle toggled
        assert!(c.slip_hc(sat, sig, &m2, true).0);
    }

    #[test]
    fn slip_on_cp_stdev_threshold() {
        let (sat, sig) = key();
        // At/over threshold -> slip; below -> none.
        let mut c = converter(opts());
        let mut m = base_meas();
        m.cp_stdev = 15;
        assert!(c.slip_hc(sat, sig, &m, true).0);

        let mut c = converter(opts());
        let mut m = base_meas();
        m.cp_stdev = 14;
        assert!(!c.slip_hc(sat, sig, &m, true).0);
    }

    #[test]
    fn half_cycle_unresolved_rules() {
        let mut m = base_meas();
        // Non-SBAS: unresolved iff the HALF_CYC bit is clear.
        m.trk_stat = PR_VALID | CP_VALID; // HALF_CYC clear
        assert!(half_cycle_unresolved(&m));
        m.trk_stat |= HALF_CYC;
        assert!(!half_cycle_unresolved(&m));
        // SBAS: unresolved while lock time is short (<= 8000).
        let mut s = base_meas();
        s.gnss_id = SBAS;
        s.lock_time = 8000;
        assert!(half_cycle_unresolved(&s));
        s.lock_time = 8001;
        assert!(!half_cycle_unresolved(&s));
    }

    // ---- carrier phase / BDS GEO half-cycle ----

    #[test]
    fn bds_geo_half_cycle_option() {
        let bds = |sv| Meas {
            cp_mes: 100.25,
            gnss_id: BDS,
            sv_id: sv,
            trk_stat: CP_VALID | HALF_CYC,
            ..base_meas()
        };
        // Default: GEO phase unchanged.
        assert_eq!(carrier_phase(&bds(2), false), Some(100.25));
        // Enabled: a BDS GEO (svid <= 5) gets a half-cycle added.
        assert_eq!(carrier_phase(&bds(2), true), Some(100.75));
        // Enabled: a non-GEO BDS satellite is unchanged.
        assert_eq!(carrier_phase(&bds(6), true), Some(100.25));
        // No CP_VALID -> no phase.
        let mut no_cp = bds(2);
        no_cp.trk_stat = PR_VALID;
        assert_eq!(carrier_phase(&no_cp, true), None);
    }

    #[test]
    fn rawx_frame_recognition() {
        assert!(is_rawx_frame(&[0xB5, 0x62, 0x02, 0x15]));
        assert!(!is_rawx_frame(&[0xB5, 0x62, 0x02, 0x14])); // wrong id
        assert!(!is_rawx_frame(&[0xB5, 0x62, 0x01, 0x15])); // wrong class
        assert!(!is_rawx_frame(&[0xB5, 0x62])); // too short
    }
}

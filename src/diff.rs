//! Semantic observation comparator: aligns two observation streams by epoch
//! and `(sat, sig)` and reports per-field differences within tolerance. It is
//! the validation oracle — obsj is compared at exact-f64 (tolerance 0), RINEX
//! at 5e-4 (its three-decimal text precision).

use crate::obs::{GpsTime, Metadata, SatId, SigId, SignalObservation, SignalValues};
use std::collections::HashMap;

#[derive(Clone, Copy)]
pub struct ObsTolerances {
    pub pr: f64,
    pub cp: f64,
    pub dop: f64,
    pub cn0: f64,
}

#[derive(Clone, Copy)]
pub struct MetadataTolerances {
    pub approx_pos: f64,
    pub antenna_delta: f64,
}

/// One side's differing values, plus the LL transition bit.
#[derive(Clone, Copy, Default)]
pub struct SignalDiff {
    pub v: SignalValues,
    pub ll: bool,
}

impl SignalDiff {
    fn is_zero(&self) -> bool {
        self.v.is_zero() && !self.ll
    }
}

pub struct DiffRecord {
    pub t: GpsTime,
    pub sat: SatId,
    pub sig: SigId,
    pub a: Option<SignalDiff>,
    pub b: Option<SignalDiff>,
}

struct ObsIndex<'a> {
    epochs: HashMap<GpsTime, HashMap<(SatId, SigId), usize>>,
    times: Vec<GpsTime>,
    obs: &'a [SignalObservation],
}

fn index_observations(obs: &[SignalObservation]) -> ObsIndex<'_> {
    let mut epochs: HashMap<GpsTime, HashMap<(SatId, SigId), usize>> = HashMap::new();
    let mut times: Vec<GpsTime> = Vec::new();
    for (i, o) in obs.iter().enumerate() {
        let e = epochs.entry(o.t).or_insert_with(|| {
            times.push(o.t);
            HashMap::new()
        });
        e.entry((o.sat, o.sig)).or_insert(i);
    }
    ObsIndex { epochs, times, obs }
}

/// Compares two observation streams, returning the list of differences.
pub fn diff_observations(
    a: &[SignalObservation],
    b: &[SignalObservation],
    tol: ObsTolerances,
) -> Vec<DiffRecord> {
    let ai = index_observations(a);
    let bi = index_observations(b);
    let mut out = Vec::new();

    let mut a_arc = ArcTracker::default();
    let mut b_arc = ArcTracker::default();

    for t in diff_times(&ai, &bi) {
        for k in diff_keys(ai.epochs.get(&t), bi.epochs.get(&t)) {
            let (mut af, mut bf) = compare_at(t, k, &ai, &bi, tol);
            let a_ll = a_arc.transition(k, &ai, t);
            let b_ll = b_arc.transition(k, &bi, t);
            if let (Some(af_v), Some(bf_v)) = (af.as_mut(), bf.as_mut()) {
                if a_ll != b_ll {
                    af_v.ll = a_ll;
                    bf_v.ll = b_ll;
                }
                if af_v.is_zero() && bf_v.is_zero() {
                    continue;
                }
            }
            out.push(DiffRecord {
                t,
                sat: k.0,
                sig: k.1,
                a: af,
                b: bf,
            });
        }
    }
    out
}

#[derive(Default)]
struct ArcTracker {
    prev: HashMap<(SatId, SigId), u32>,
    seen: HashMap<(SatId, SigId), bool>,
}

impl ArcTracker {
    fn transition(&mut self, k: (SatId, SigId), idx: &ObsIndex, t: GpsTime) -> bool {
        let i = match idx.epochs.get(&t).and_then(|m| m.get(&k)) {
            Some(&i) => i,
            None => return false,
        };
        let arc = idx.obs[i].v.arc;
        let transitioned = *self.seen.get(&k).unwrap_or(&false) && arc != *self.prev.get(&k).unwrap_or(&0);
        self.prev.insert(k, arc);
        self.seen.insert(k, true);
        transitioned
    }
}

fn diff_times(a: &ObsIndex, b: &ObsIndex) -> Vec<GpsTime> {
    let mut seen: HashMap<GpsTime, ()> = HashMap::new();
    let mut out = Vec::with_capacity(a.times.len() + b.times.len());
    for &t in a.times.iter().chain(b.times.iter()) {
        if seen.insert(t, ()).is_none() {
            out.push(t);
        }
    }
    out.sort_unstable_by_key(|t| t.0);
    out
}

fn diff_keys(
    a: Option<&HashMap<(SatId, SigId), usize>>,
    b: Option<&HashMap<(SatId, SigId), usize>>,
) -> Vec<(SatId, SigId)> {
    let mut seen: HashMap<(SatId, SigId), ()> = HashMap::new();
    let mut out = Vec::new();
    for m in [a, b].into_iter().flatten() {
        for &k in m.keys() {
            if seen.insert(k, ()).is_none() {
                out.push(k);
            }
        }
    }
    out.sort_unstable_by(|x, y| (x.0, x.1).cmp(&(y.0, y.1)));
    out
}

fn compare_at(
    t: GpsTime,
    k: (SatId, SigId),
    ai: &ObsIndex,
    bi: &ObsIndex,
    tol: ObsTolerances,
) -> (Option<SignalDiff>, Option<SignalDiff>) {
    let av = ai.epochs.get(&t).and_then(|m| m.get(&k)).map(|&i| &ai.obs[i].v);
    let bv = bi.epochs.get(&t).and_then(|m| m.get(&k)).map(|&i| &bi.obs[i].v);
    diff_signal(av, bv, tol)
}

fn values_as_diff(v: &SignalValues) -> SignalDiff {
    let mut d = SignalDiff {
        v: *v,
        ll: false,
    };
    d.v.arc = 0;
    d
}

fn diff_signal(
    a: Option<&SignalValues>,
    b: Option<&SignalValues>,
    tol: ObsTolerances,
) -> (Option<SignalDiff>, Option<SignalDiff>) {
    match (a, b) {
        (None, None) => (None, None),
        (None, Some(b)) => (None, Some(values_as_diff(b))),
        (Some(a), None) => (Some(values_as_diff(a)), None),
        (Some(a), Some(b)) => {
            let mut ar = SignalDiff::default();
            let mut br = SignalDiff::default();
            // frq (exact)
            if !(a.frq == b.frq) {
                ar.v.frq = a.frq;
                br.v.frq = b.frq;
            }
            cmp_f64(&mut ar.v.pr, &mut br.v.pr, a.pr, b.pr, tol.pr);
            cmp_f64(&mut ar.v.cp, &mut br.v.cp, a.cp, b.cp, tol.cp);
            cmp_f64(&mut ar.v.dop, &mut br.v.dop, a.dop, b.dop, tol.dop);
            cmp_f32(&mut ar.v.cn0, &mut br.v.cn0, a.cn0, b.cn0, tol.cn0);
            if a.hc != b.hc {
                ar.v.hc = a.hc;
                br.v.hc = b.hc;
            }
            if a.bt != b.bt {
                ar.v.bt = a.bt;
                br.v.bt = b.bt;
            }
            (Some(ar), Some(br))
        }
    }
}

fn cmp_f64(a: &mut Option<f64>, b: &mut Option<f64>, av: Option<f64>, bv: Option<f64>, tol: f64) {
    let near = match (av, bv) {
        (None, None) => true,
        (Some(x), Some(y)) => (x - y).abs() <= tol,
        _ => false,
    };
    if near {
        return;
    }
    *a = av;
    *b = bv;
}

fn cmp_f32(a: &mut Option<f32>, b: &mut Option<f32>, av: Option<f32>, bv: Option<f32>, tol: f64) {
    let near = match (av, bv) {
        (None, None) => true,
        (Some(x), Some(y)) => ((x - y) as f64).abs() <= tol,
        _ => false,
    };
    if near {
        return;
    }
    *a = av;
    *b = bv;
}

// ---- metadata diff ----

/// Compares two metadata records, returning the differing fields per side.
/// Run and Comment are ignored (per the diffobs spec).
pub fn diff_metadata(a: &Metadata, b: &Metadata, tol: MetadataTolerances) -> (Metadata, Metadata) {
    let mut ao = Metadata::default();
    let mut bo = Metadata::default();
    cmp_str(&mut ao.version, &mut bo.version, &a.version, &b.version);
    cmp_str(&mut ao.marker.name, &mut bo.marker.name, &a.marker.name, &b.marker.name);
    cmp_str(&mut ao.marker.number, &mut bo.marker.number, &a.marker.number, &b.marker.number);
    cmp_str(&mut ao.marker.type_, &mut bo.marker.type_, &a.marker.type_, &b.marker.type_);
    cmp_str(&mut ao.observer, &mut bo.observer, &a.observer, &b.observer);
    cmp_str(&mut ao.agency, &mut bo.agency, &a.agency, &b.agency);
    cmp_str(&mut ao.receiver.number, &mut bo.receiver.number, &a.receiver.number, &b.receiver.number);
    cmp_str(&mut ao.receiver.type_, &mut bo.receiver.type_, &a.receiver.type_, &b.receiver.type_);
    cmp_str(&mut ao.receiver.version, &mut bo.receiver.version, &a.receiver.version, &b.receiver.version);
    cmp_str(&mut ao.antenna.number, &mut bo.antenna.number, &a.antenna.number, &b.antenna.number);
    cmp_str(&mut ao.antenna.type_, &mut bo.antenna.type_, &a.antenna.type_, &b.antenna.type_);
    cmp_triple(&mut ao.approx_position, &mut bo.approx_position, a.approx_position, b.approx_position, tol.approx_pos);
    cmp_triple(&mut ao.antenna_delta, &mut bo.antenna_delta, a.antenna_delta, b.antenna_delta, tol.antenna_delta);
    cmp_opt(&mut ao.interval, &mut bo.interval, a.interval, b.interval);
    cmp_opt(&mut ao.leap_seconds, &mut bo.leap_seconds, a.leap_seconds, b.leap_seconds);
    (ao, bo)
}

fn cmp_str(a: &mut String, b: &mut String, av: &str, bv: &str) {
    if av != bv {
        *a = av.to_string();
        *b = bv.to_string();
    }
}

fn cmp_opt<T: PartialEq + Copy>(a: &mut Option<T>, b: &mut Option<T>, av: Option<T>, bv: Option<T>) {
    if av != bv {
        *a = av;
        *b = bv;
    }
}

fn cmp_triple(
    a: &mut Option<[f64; 3]>,
    b: &mut Option<[f64; 3]>,
    av: Option<[f64; 3]>,
    bv: Option<[f64; 3]>,
    tol: f64,
) {
    let near = match (av, bv) {
        (None, None) => true,
        (Some(x), Some(y)) => (0..3).all(|i| (x[i] - y[i]).abs() <= tol),
        _ => false,
    };
    if !near {
        *a = av;
        *b = bv;
    }
}

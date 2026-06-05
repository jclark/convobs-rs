//! Semantic observation comparator: aligns two observation streams by epoch
//! and `(sat, sig)` and reports per-field differences within tolerance. It is
//! the validation oracle — obsj is compared at exact-f64 (tolerance 0), RINEX
//! at 5e-4 (its three-decimal text precision).

use crate::arc::ArcToLl;
use crate::obs::{GpsTime, Metadata, SatId, SigId, SignalKey, SignalObservation, SignalValues};
use std::collections::{HashMap, HashSet};

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
///
/// With `ignore_blank_phase`, the carrier-phase and loss-of-lock comparison is
/// skipped for a signal whenever either side carries no carrier phase. That
/// covers the `rinex`-crate backend, which cannot emit a blank phase that exists
/// only to hold a loss-of-lock flag, so it drops the signal's phase entirely.
pub fn diff_observations(
    a: &[SignalObservation],
    b: &[SignalObservation],
    tol: ObsTolerances,
    ignore_blank_phase: bool,
) -> Vec<DiffRecord> {
    let ai = index_observations(a);
    let bi = index_observations(b);
    let mut out = Vec::new();

    let mut a_arc = ArcToLl::new();
    let mut b_arc = ArcToLl::new();

    for t in diff_times(&ai, &bi) {
        for k in diff_keys(ai.epochs.get(&t), bi.epochs.get(&t)) {
            let (mut af, mut bf) = compare_at(t, k, &ai, &bi, tol);
            let a_ll = transition_at(&mut a_arc, &ai, t, k);
            let b_ll = transition_at(&mut b_arc, &bi, t, k);
            if let (Some(af_v), Some(bf_v)) = (af.as_mut(), bf.as_mut()) {
                if ignore_blank_phase && cp_blank(&ai, &bi, t, k) {
                    af_v.v.cp = None;
                    bf_v.v.cp = None;
                } else if a_ll != b_ll {
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

/// Whether the signal `k` at epoch `t` is present in both streams but lacks a
/// carrier phase on at least one side (the blank-phase case).
fn cp_blank(ai: &ObsIndex, bi: &ObsIndex, t: GpsTime, k: (SatId, SigId)) -> bool {
    let has_cp = |idx: &ObsIndex| {
        idx.epochs
            .get(&t)
            .and_then(|m| m.get(&k))
            .map(|&i| idx.obs[i].v.cp.is_some())
    };
    matches!((has_cp(ai), has_cp(bi)), (Some(a), Some(b)) if !a || !b)
}

/// The loss-of-lock transition for key `k` at epoch `t` in one stream. When the
/// signal is absent this epoch the tracker is left untouched, so the transition
/// is always measured against the previous epoch in which it *was* present.
fn transition_at(arc: &mut ArcToLl, idx: &ObsIndex, t: GpsTime, k: (SatId, SigId)) -> bool {
    match idx.epochs.get(&t).and_then(|m| m.get(&k)) {
        Some(&i) => arc.transition(SignalKey { sat: k.0, sig: k.1 }, idx.obs[i].v.arc),
        None => false,
    }
}

fn diff_times(a: &ObsIndex, b: &ObsIndex) -> Vec<GpsTime> {
    let mut seen: HashSet<GpsTime> = HashSet::new();
    let mut out = Vec::with_capacity(a.times.len() + b.times.len());
    for &t in a.times.iter().chain(b.times.iter()) {
        if seen.insert(t) {
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
    let mut seen: HashSet<(SatId, SigId)> = HashSet::new();
    let mut out = Vec::new();
    for m in [a, b].into_iter().flatten() {
        for &k in m.keys() {
            if seen.insert(k) {
                out.push(k);
            }
        }
    }
    out.sort_unstable_by_key(|x| (x.0, x.1));
    out
}

fn compare_at(
    t: GpsTime,
    k: (SatId, SigId),
    ai: &ObsIndex,
    bi: &ObsIndex,
    tol: ObsTolerances,
) -> (Option<SignalDiff>, Option<SignalDiff>) {
    let av = ai
        .epochs
        .get(&t)
        .and_then(|m| m.get(&k))
        .map(|&i| &ai.obs[i].v);
    let bv = bi
        .epochs
        .get(&t)
        .and_then(|m| m.get(&k))
        .map(|&i| &bi.obs[i].v);
    diff_signal(av, bv, tol)
}

fn values_as_diff(v: &SignalValues) -> SignalDiff {
    let mut d = SignalDiff { v: *v, ll: false };
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
            if a.frq != b.frq {
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
/// Run and Comment are always ignored (per the diffobs spec). With
/// `ignore_marker`, the marker fields are ignored too — convbin and SatPulse use
/// the RTCM station id as the marker *name* vs *number* respectively, so the
/// marker is cleaned when validating RTCM goldens.
pub fn diff_metadata(
    a: &Metadata,
    b: &Metadata,
    tol: MetadataTolerances,
    ignore_marker: bool,
) -> (Metadata, Metadata) {
    let mut ao = Metadata::default();
    let mut bo = Metadata::default();
    cmp_str(&mut ao.version, &mut bo.version, &a.version, &b.version);
    if !ignore_marker {
        cmp_str(
            &mut ao.marker.name,
            &mut bo.marker.name,
            &a.marker.name,
            &b.marker.name,
        );
        cmp_str(
            &mut ao.marker.number,
            &mut bo.marker.number,
            &a.marker.number,
            &b.marker.number,
        );
        cmp_str(
            &mut ao.marker.type_,
            &mut bo.marker.type_,
            &a.marker.type_,
            &b.marker.type_,
        );
    }
    cmp_str(&mut ao.observer, &mut bo.observer, &a.observer, &b.observer);
    cmp_str(&mut ao.agency, &mut bo.agency, &a.agency, &b.agency);
    cmp_str(
        &mut ao.receiver.number,
        &mut bo.receiver.number,
        &a.receiver.number,
        &b.receiver.number,
    );
    cmp_str(
        &mut ao.receiver.type_,
        &mut bo.receiver.type_,
        &a.receiver.type_,
        &b.receiver.type_,
    );
    cmp_str(
        &mut ao.receiver.version,
        &mut bo.receiver.version,
        &a.receiver.version,
        &b.receiver.version,
    );
    cmp_str(
        &mut ao.antenna.number,
        &mut bo.antenna.number,
        &a.antenna.number,
        &b.antenna.number,
    );
    cmp_str(
        &mut ao.antenna.type_,
        &mut bo.antenna.type_,
        &a.antenna.type_,
        &b.antenna.type_,
    );
    cmp_triple(
        &mut ao.approx_position,
        &mut bo.approx_position,
        a.approx_position,
        b.approx_position,
        tol.approx_pos,
    );
    cmp_triple(
        &mut ao.antenna_delta,
        &mut bo.antenna_delta,
        a.antenna_delta,
        b.antenna_delta,
        tol.antenna_delta,
    );
    cmp_opt(&mut ao.interval, &mut bo.interval, a.interval, b.interval);
    cmp_opt(
        &mut ao.leap_seconds,
        &mut bo.leap_seconds,
        a.leap_seconds,
        b.leap_seconds,
    );
    (ao, bo)
}

fn cmp_str(a: &mut String, b: &mut String, av: &str, bv: &str) {
    if av != bv {
        *a = av.to_string();
        *b = bv.to_string();
    }
}

fn cmp_opt<T: PartialEq + Copy>(
    a: &mut Option<T>,
    b: &mut Option<T>,
    av: Option<T>,
    bv: Option<T>,
) {
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::obs::{Antenna, Marker, Receiver};

    const TOL: ObsTolerances = ObsTolerances {
        pr: 5e-4,
        cp: 5e-4,
        dop: 5e-4,
        cn0: 5e-4,
    };
    const MTOL: MetadataTolerances = MetadataTolerances {
        approx_pos: 5e-5,
        antenna_delta: 5e-5,
    };

    fn show(v: &SignalValues) -> String {
        serde_json::to_string(v).unwrap()
    }

    fn obs(t: i64, sat: &[u8; 3], sig: &[u8; 2], v: SignalValues) -> SignalObservation {
        SignalObservation {
            t: GpsTime(t),
            sat: SatId(*sat),
            sig: SigId(*sig),
            v,
        }
    }

    #[test]
    fn diff_signal_cases() {
        let a = SignalValues {
            pr: Some(1.0),
            cp: Some(2.0),
            arc: 1,
            ..Default::default()
        };
        let b = SignalValues {
            pr: Some(1.001),
            cp: Some(2.0),
            hc: true,
            ..Default::default()
        };

        // Both missing.
        assert!(matches!(diff_signal(None, None, TOL), (None, None)));

        // One side missing reports the present side verbatim, with arc cleared.
        let (ra, rb) = diff_signal(None, Some(&b), TOL);
        assert!(ra.is_none());
        assert!(rb.unwrap().v == b, "{}", show(&rb.unwrap().v));
        let (ra, rb) = diff_signal(Some(&a), None, TOL);
        assert!(rb.is_none());
        let want_a = SignalValues {
            pr: Some(1.0),
            cp: Some(2.0),
            ..Default::default()
        }; // arc cleared
        assert!(ra.unwrap().v == want_a, "{}", show(&ra.unwrap().v));

        // Identical sides report empty diffs.
        let (ra, rb) = diff_signal(Some(&a), Some(&a), TOL);
        assert!(ra.unwrap().v.is_zero() && rb.unwrap().v.is_zero());

        // Differing values: pr out of tol is reported, cp within tol is not, the
        // hc bit is reported, and arc is never compared.
        let (ra, rb) = diff_signal(Some(&a), Some(&b), TOL);
        let (ra, rb) = (ra.unwrap(), rb.unwrap());
        assert_eq!(ra.v.pr, Some(1.0));
        assert_eq!(rb.v.pr, Some(1.001));
        assert_eq!(ra.v.cp, None);
        assert_eq!(rb.v.cp, None);
        assert!(!ra.v.hc && rb.v.hc);

        // A field present on one side only is a difference.
        let one = SignalValues {
            pr: Some(1.0),
            ..Default::default()
        };
        let (ra, rb) = diff_signal(Some(&one), Some(&SignalValues::default()), TOL);
        assert_eq!(ra.unwrap().v.pr, Some(1.0));
        assert!(rb.unwrap().v.is_zero());

        // Within tolerance on every field is no difference.
        let close_a = SignalValues {
            pr: Some(1.0),
            cn0: Some(45.0),
            ..Default::default()
        };
        let close_b = SignalValues {
            pr: Some(1.0001),
            cn0: Some(45.0001),
            ..Default::default()
        };
        let (ra, rb) = diff_signal(Some(&close_a), Some(&close_b), TOL);
        assert!(ra.unwrap().v.is_zero() && rb.unwrap().v.is_zero());
    }

    #[test]
    fn diff_observations_reports_missing_side() {
        let a = [obs(
            1,
            b"G01",
            b"1C",
            SignalValues {
                pr: Some(1.0),
                ..Default::default()
            },
        )];
        let b = [obs(
            1,
            b"G02",
            b"1C",
            SignalValues {
                pr: Some(2.0),
                ..Default::default()
            },
        )];
        let diffs = diff_observations(&a, &b, TOL, false);
        assert_eq!(diffs.len(), 2);
        // Keys sort (G01, then G02).
        assert!(diffs[0].a.is_some() && diffs[0].b.is_none());
        assert!(diffs[1].a.is_none() && diffs[1].b.is_some());
    }

    #[test]
    fn diff_observations_arc_transition_ll() {
        // PR is identical throughout, so the only differences are loss-of-lock
        // transitions where the two streams' arc counters step at different
        // epochs.
        let mk = |t: i64, arc: u32| {
            obs(
                t,
                b"G01",
                b"1C",
                SignalValues {
                    pr: Some(1.0),
                    arc,
                    ..Default::default()
                },
            )
        };
        // a transitions at t2 and t4; b transitions at t3 and t4.
        let a = [mk(10, 0), mk(20, 1), mk(30, 1), mk(40, 2)];
        let b = [mk(10, 0), mk(20, 0), mk(30, 1), mk(40, 2)];
        let diffs = diff_observations(&a, &b, TOL, false);
        // t4: both transition -> agreement, no record. t1: first-seen, none.
        assert_eq!(diffs.len(), 2);
        assert_eq!(diffs[0].t.0, 20);
        assert!(diffs[0].a.unwrap().ll && !diffs[0].b.unwrap().ll);
        assert_eq!(diffs[1].t.0, 30);
        assert!(!diffs[1].a.unwrap().ll && diffs[1].b.unwrap().ll);
    }

    #[test]
    fn diff_metadata_fields() {
        let a = Metadata {
            version: "3.04".to_string(),
            marker: Marker {
                name: "mark-a".to_string(),
                number: "001".to_string(),
                ..Default::default()
            },
            observer: "obs-a".to_string(),
            agency: "same-agency".to_string(),
            receiver: Receiver {
                number: "rx-a".to_string(),
                type_: "rx-type".to_string(),
                version: "rx-vers".to_string(),
            },
            antenna: Antenna {
                number: "ant-num".to_string(),
                type_: "ant-a".to_string(),
            },
            approx_position: Some([1.0, 2.0, 3.0]),
            antenna_delta: Some([0.0, 0.0, 0.0]),
            interval: Some(30.0),
            leap_seconds: Some(18),
            ..Default::default()
        };
        let b = Metadata {
            version: "4.02".to_string(),
            marker: Marker {
                name: "mark-b".to_string(),
                type_: "GEODETIC".to_string(),
                ..Default::default()
            },
            observer: "obs-b".to_string(),
            agency: "same-agency".to_string(),
            receiver: Receiver {
                number: "rx-b".to_string(),
                type_: "rx-type".to_string(),
                version: "rx-vers".to_string(),
            },
            antenna: Antenna {
                number: "ant-num".to_string(),
                type_: "ant-b".to_string(),
            },
            // Within 5e-5 on x -> not reported.
            approx_position: Some([1.00004, 2.0, 3.0]),
            // 2e-4 on y -> reported.
            antenna_delta: Some([0.0, 0.0002, 0.0]),
            interval: Some(15.0),
            leap_seconds: None,
            ..Default::default()
        };
        let (da, db) = diff_metadata(&a, &b, MTOL, false);
        assert_eq!((da.version.as_str(), db.version.as_str()), ("3.04", "4.02"));
        // Marker (not ignored): name, number, type each reported per side.
        assert_eq!(da.marker.name, "mark-a");
        assert_eq!(db.marker.name, "mark-b");
        assert_eq!(da.marker.number, "001");
        assert_eq!(db.marker.number, "");
        assert_eq!(da.marker.type_, "");
        assert_eq!(db.marker.type_, "GEODETIC");
        assert_eq!(
            (da.observer.as_str(), db.observer.as_str()),
            ("obs-a", "obs-b")
        );
        // agency matches -> not reported.
        assert_eq!((da.agency.as_str(), db.agency.as_str()), ("", ""));
        assert_eq!(da.receiver.number, "rx-a");
        assert_eq!(db.receiver.number, "rx-b");
        assert_eq!(da.receiver.type_, ""); // matches
        assert_eq!(da.antenna.type_, "ant-a");
        assert_eq!(db.antenna.type_, "ant-b");
        assert_eq!(da.antenna.number, ""); // matches

        // approx position within tolerance -> not reported.
        assert_eq!(da.approx_position, None);
        assert_eq!(db.approx_position, None);
        // antenna delta out of tolerance -> reported verbatim.
        assert_eq!(da.antenna_delta, Some([0.0, 0.0, 0.0]));
        assert_eq!(db.antenna_delta, Some([0.0, 0.0002, 0.0]));
        assert_eq!((da.interval, db.interval), (Some(30.0), Some(15.0)));
        assert_eq!((da.leap_seconds, db.leap_seconds), (Some(18), None));
    }

    #[test]
    fn diff_metadata_ignore_marker() {
        // With ignore_marker, differing markers are not reported (the RTCM case,
        // where convbin and convobs split the station id across name vs number).
        let a = Metadata {
            marker: Marker {
                name: "STA".to_string(),
                ..Default::default()
            },
            ..Default::default()
        };
        let b = Metadata {
            marker: Marker {
                number: "STA".to_string(),
                ..Default::default()
            },
            ..Default::default()
        };
        let (da, db) = diff_metadata(&a, &b, MTOL, true);
        assert!(da.is_zero() && db.is_zero());
    }
}

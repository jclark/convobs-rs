//! Centralized loss-of-lock handling.
//!
//! On the wire, loss of lock is the monotonic per-`(sat, sig)` counter `arc`.
//! Converters, however, only ever *detect a slip* — a per-observation boolean.
//! Rather than each converter maintaining its own `arc` map (as the Go reference
//! does), they emit the boolean on [`SignalValues::ll`] and the streaming
//! [`LossOfLockSink`] accumulates it into `arc`. The inverse [`ArcToLl`]
//! transform turns `arc` back into the per-observation transition that feeds
//! RINEX LLI and the diff comparator. One implementation, reused everywhere.

use crate::obs::{SignalKey, SignalObservation};
use crate::sink::Sink;
use rustc_hash::FxHashMap;
use std::io;

/// Streaming converter stage: turns the per-observation loss-of-lock flag
/// ([`SignalValues::ll`](crate::obs::SignalValues::ll)) into the monotonic
/// `arc` counter, per `(sat, sig)`.
///
/// It belongs **upstream of decimation** so that a slip occurring inside a
/// dropped gap still bumps `arc` and therefore surfaces on the next kept epoch.
pub struct LossOfLockSink<S: Sink> {
    sink: S,
    arc: FxHashMap<SignalKey, u32>,
}

impl<S: Sink> LossOfLockSink<S> {
    pub fn new(sink: S) -> Self {
        LossOfLockSink {
            sink,
            arc: FxHashMap::default(),
        }
    }

    /// Returns the wrapped sink (e.g. to recover a buffered result).
    pub fn into_inner(self) -> S {
        self.sink
    }
}

impl<S: Sink> Sink for LossOfLockSink<S> {
    fn metadata(&mut self, m: &crate::obs::Metadata) -> io::Result<()> {
        self.sink.metadata(m)
    }

    fn observation(&mut self, o: &SignalObservation) -> io::Result<()> {
        let mut o = *o;
        let counter = self
            .arc
            .entry(SignalKey {
                sat: o.sat,
                sig: o.sig,
            })
            .or_insert(0);
        if o.v.ll {
            *counter += 1;
        }
        o.v.arc = *counter;
        o.v.ll = false;
        self.sink.observation(&o)
    }

    fn flush(&mut self) -> io::Result<()> {
        self.sink.flush()
    }
}

/// Derives the RINEX loss-of-lock bit (LLI bit 0) from `arc`: it is set when a
/// signal's `arc` differs from its value at the previous observation.
///
/// Two consumers need slightly different first-observation behaviour, so the
/// transform exposes both as methods over one shared per-signal map.
#[derive(Default)]
pub struct ArcToLl {
    prev: FxHashMap<SignalKey, u32>,
}

impl ArcToLl {
    pub fn new() -> Self {
        ArcToLl::default()
    }

    /// LLI bit 0 for a RINEX writer: a first-seen signal reports a slip when its
    /// `arc` is non-zero, so any arc carried into the file is encoded.
    pub fn lli(&mut self, k: SignalKey, arc: u32) -> bool {
        match self.prev.insert(k, arc) {
            Some(prev) => arc != prev,
            None => arc != 0,
        }
    }

    /// Loss-of-lock *transition* for the diff comparator: a first-seen signal
    /// reports no transition, since there is nothing to compare against.
    pub fn transition(&mut self, k: SignalKey, arc: u32) -> bool {
        match self.prev.insert(k, arc) {
            Some(prev) => arc != prev,
            None => false,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::obs::{GpsTime, Metadata, SatId, SigId, SignalValues};

    #[derive(Default)]
    struct Collect {
        obs: Vec<SignalObservation>,
    }
    impl Sink for Collect {
        fn metadata(&mut self, _m: &Metadata) -> io::Result<()> {
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

    fn key() -> SignalKey {
        SignalKey {
            sat: SatId::format(b'G', 1),
            sig: SigId(*b"1C"),
        }
    }

    fn obs(t: i64, ll: bool) -> SignalObservation {
        let v = SignalValues {
            ll,
            cp: Some(1.0),
            ..Default::default()
        };
        SignalObservation {
            t: GpsTime(t),
            sat: SatId::format(b'G', 1),
            sig: SigId(*b"1C"),
            v,
        }
    }

    #[test]
    fn accumulates_arc_from_ll() {
        let mut sink = LossOfLockSink::new(Collect::default());
        for (i, ll) in [false, false, true, false, true].iter().enumerate() {
            sink.observation(&obs(i as i64, *ll)).unwrap();
        }
        let arcs: Vec<u32> = sink.sink.obs.iter().map(|o| o.v.arc).collect();
        assert_eq!(arcs, vec![0, 0, 1, 1, 2]);
        // `ll` is consumed: never leaks past the accumulator.
        assert!(sink.sink.obs.iter().all(|o| !o.v.ll));
    }

    #[test]
    fn writer_lli_first_seen_uses_zero_baseline() {
        assert!(ArcToLl::new().lli(key(), 5)); // carried-in arc -> slip in the file
        assert!(!ArcToLl::new().lli(key(), 0));
    }

    #[test]
    fn diff_transition_ignores_first_seen() {
        let mut t = ArcToLl::new();
        assert!(!t.transition(key(), 5)); // nothing to compare against yet
        assert!(!t.transition(key(), 5)); // 5 -> 5
        assert!(t.transition(key(), 6)); // 5 -> 6
    }
}

//! The `Sink` pipeline: converters push records through optional filters
//! (decimation, require-carrier-phase) into an output writer.

use crate::error::{Error, Result};
use crate::obs::{GpsTime, Metadata, SignalObservation, TICK_NS};
use std::io;

/// Receives conversion records. Errors are I/O errors from the output writer
/// or conversion errors surfaced by a downstream sink.
pub trait Sink {
    fn metadata(&mut self, m: &Metadata) -> io::Result<()>;
    fn observation(&mut self, o: &SignalObservation) -> io::Result<()>;
    fn flush(&mut self) -> io::Result<()>;
}

const DECIMATION_ROUND_TICKS: i64 = 100 * 1_000_000 / TICK_NS; // 100 ms in ticks

/// Validates a decimation interval (nanoseconds) against the same rules Go uses,
/// returning the interval in ticks.
pub fn decimation_interval_ticks(interval_ns: i64) -> Result<i64> {
    if interval_ns < 1_000_000_000 {
        return Err(Error::Interval(
            "decimation interval must be at least 1 second".to_string(),
        ));
    }
    if interval_ns % TICK_NS != 0 {
        return Err(Error::Interval(
            "decimation interval must be a multiple of 100ns".to_string(),
        ));
    }
    if (24 * 3600 * 1_000_000_000i64) % interval_ns != 0 {
        return Err(Error::Interval(
            "decimation interval must divide one GPS day exactly".to_string(),
        ));
    }
    Ok(interval_ns / TICK_NS)
}

pub fn validate_decimation_interval(interval_ns: i64) -> Result<()> {
    decimation_interval_ticks(interval_ns).map(|_| ())
}

fn round_time(t: i64, unit: i64) -> i64 {
    if t < 0 {
        -round_time(-t, unit)
    } else {
        ((t + unit / 2) / unit) * unit
    }
}

/// Emits only observations whose rounded epoch label is on the interval grid.
pub struct DecimationSink<S: Sink> {
    sink: S,
    interval: i64,
}

impl<S: Sink> DecimationSink<S> {
    pub fn new(sink: S, interval_ticks: i64) -> Self {
        DecimationSink {
            sink,
            interval: interval_ticks,
        }
    }

    fn on_grid(&self, t: GpsTime) -> bool {
        let r = round_time(t.0, DECIMATION_ROUND_TICKS);
        let m = r.rem_euclid(self.interval);
        m == 0
    }
}

impl<S: Sink> Sink for DecimationSink<S> {
    fn metadata(&mut self, m: &Metadata) -> io::Result<()> {
        self.sink.metadata(m)
    }
    fn observation(&mut self, o: &SignalObservation) -> io::Result<()> {
        if self.on_grid(o.t) {
            self.sink.observation(o)
        } else {
            Ok(())
        }
    }
    fn flush(&mut self) -> io::Result<()> {
        self.sink.flush()
    }
}

/// Forwards only observations that carry a carrier phase (for `--ppp-ar`).
pub struct RequireCpFilter<S: Sink> {
    sink: S,
}

impl<S: Sink> RequireCpFilter<S> {
    pub fn new(sink: S) -> Self {
        RequireCpFilter { sink }
    }
}

impl<S: Sink> Sink for RequireCpFilter<S> {
    fn metadata(&mut self, m: &Metadata) -> io::Result<()> {
        self.sink.metadata(m)
    }
    fn observation(&mut self, o: &SignalObservation) -> io::Result<()> {
        if o.v.cp.is_none() {
            return Ok(());
        }
        self.sink.observation(o)
    }
    fn flush(&mut self) -> io::Result<()> {
        self.sink.flush()
    }
}

/// Boxed sink used where the pipeline shape is chosen at runtime.
impl Sink for Box<dyn Sink> {
    fn metadata(&mut self, m: &Metadata) -> io::Result<()> {
        (**self).metadata(m)
    }
    fn observation(&mut self, o: &SignalObservation) -> io::Result<()> {
        (**self).observation(o)
    }
    fn flush(&mut self) -> io::Result<()> {
        (**self).flush()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::obs::{Civil, SatId, SigId, SignalValues};

    #[derive(Default)]
    struct Recorder {
        meta: Vec<Metadata>,
        obs: Vec<SignalObservation>,
        flushed: bool,
    }
    impl Sink for Recorder {
        fn metadata(&mut self, m: &Metadata) -> io::Result<()> {
            self.meta.push(m.clone());
            Ok(())
        }
        fn observation(&mut self, o: &SignalObservation) -> io::Result<()> {
            self.obs.push(*o);
            Ok(())
        }
        fn flush(&mut self) -> io::Result<()> {
            self.flushed = true;
            Ok(())
        }
    }

    fn at(second: u32, nanos: u32) -> GpsTime {
        GpsTime::from_civil(Civil {
            year: 2025,
            month: 7,
            day: 1,
            hour: 0,
            minute: 0,
            second,
            nanos,
        })
    }

    fn obs(t: GpsTime, sig: &[u8; 2], v: SignalValues) -> SignalObservation {
        SignalObservation {
            t,
            sat: SatId::format(b'G', 3),
            sig: SigId(*sig),
            v,
        }
    }

    fn pr(t: GpsTime, sig: &[u8; 2], pr: f64) -> SignalObservation {
        obs(
            t,
            sig,
            SignalValues {
                pr: Some(pr),
                ..Default::default()
            },
        )
    }

    #[test]
    fn rejects_invalid_interval() {
        // Subsecond, sub-tick, and non-day-divisor intervals each fail with a
        // distinct message (mirrors Go's NewDecimationSink checks).
        assert!(decimation_interval_ticks(500_000_000)
            .unwrap_err()
            .to_string()
            .contains("at least 1 second"));
        assert!(decimation_interval_ticks(1_000_000_001)
            .unwrap_err()
            .to_string()
            .contains("multiple of 100ns"));
        assert!(decimation_interval_ticks(7_000_000_000)
            .unwrap_err()
            .to_string()
            .contains("divide one GPS day"));
        // A valid interval returns its length in ticks.
        assert_eq!(decimation_interval_ticks(5_000_000_000).unwrap(), 50_000_000);
        assert!(validate_decimation_interval(5_000_000_000).is_ok());
    }

    #[test]
    fn keeps_only_rounded_grid_epochs() {
        let ticks = decimation_interval_ticks(5_000_000_000).unwrap();
        let mut sink = DecimationSink::new(Recorder::default(), ticks);
        let mut meta = Metadata::default();
        meta.marker.name = "MARK".to_string();
        sink.metadata(&meta).unwrap();
        // .049 rounds to 0.0 (on the 5 s grid); .051 -> 0.1 (off); 4.951 -> 5.0
        // (on); 5.051 -> 5.1 (off).
        let records = [
            pr(at(0, 49_000_000), b"1C", 1.0),
            pr(at(0, 51_000_000), b"1C", 2.0),
            pr(at(4, 951_000_000), b"1C", 3.0),
            pr(at(5, 51_000_000), b"1C", 4.0),
        ];
        for o in &records {
            sink.observation(o).unwrap();
        }
        sink.flush().unwrap();
        let r = &sink.sink;
        assert_eq!(r.meta.len(), 1);
        assert_eq!(r.meta[0].marker.name, "MARK");
        assert_eq!(r.obs.len(), 2);
        assert_eq!(r.obs[0].t.0, records[0].t.0);
        assert_eq!(r.obs[1].t.0, records[2].t.0);
        assert!(r.flushed, "flush must reach the wrapped sink");
    }

    #[test]
    fn forwards_fields_unchanged_for_kept_epochs() {
        // DecimationSink only filters by grid; it never touches arc/HC/BT
        // (that is the LossOfLockSink's job, upstream).
        let ticks = decimation_interval_ticks(10_000_000_000).unwrap();
        let mut sink = DecimationSink::new(Recorder::default(), ticks);
        let off_grid = SignalValues {
            arc: 1,
            ..Default::default()
        };
        sink.observation(&obs(at(1, 0), b"1C", off_grid)).unwrap();
        sink.observation(&obs(at(2, 0), b"1C", off_grid)).unwrap();
        sink.observation(&obs(
            at(10, 0),
            b"1C",
            SignalValues {
                pr: Some(1.0),
                arc: 1,
                ..Default::default()
            },
        ))
        .unwrap();
        sink.observation(&obs(
            at(10, 0),
            b"2S",
            SignalValues {
                pr: Some(2.0),
                arc: 1,
                bt: true,
                ..Default::default()
            },
        ))
        .unwrap();
        let r = &sink.sink;
        assert_eq!(r.obs.len(), 2);
        assert_eq!((r.obs[0].v.arc, r.obs[0].v.hc, r.obs[0].v.bt), (1, false, false));
        assert_eq!((r.obs[1].v.arc, r.obs[1].v.hc, r.obs[1].v.bt), (1, false, true));
    }

    #[test]
    fn require_cp_drops_phaseless() {
        let mut sink = RequireCpFilter::new(Recorder::default());
        sink.observation(&obs(
            at(0, 0),
            b"1C",
            SignalValues {
                cp: Some(1.0),
                ..Default::default()
            },
        ))
        .unwrap();
        sink.observation(&pr(at(1, 0), b"1C", 2.0)).unwrap();
        assert_eq!(sink.sink.obs.len(), 1);
        assert_eq!(sink.sink.obs[0].v.cp, Some(1.0));
    }
}

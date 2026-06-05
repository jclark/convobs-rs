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

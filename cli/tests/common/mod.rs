//! Shared helpers for the convobs integration tests.

use obsj::obs::{Civil, Instant};
use std::io::{self, Write};
use std::sync::{Arc, Mutex};

/// A `Write` sink that can be cloned and read back after the conversion runs.
///
/// `run_to_writer` takes an owned `Box<dyn Write + 'static>`, so a test cannot
/// hand it a `&mut Vec<u8>`. This shares one buffer behind an `Arc<Mutex<_>>`:
/// pass a clone into the conversion, read the bytes from the original once it
/// returns.
#[derive(Clone, Default)]
pub struct SharedBuf(Arc<Mutex<Vec<u8>>>);

impl SharedBuf {
    pub fn new() -> Self {
        Self::default()
    }

    /// A snapshot of the bytes written so far.
    pub fn bytes(&self) -> Vec<u8> {
        self.0.lock().unwrap().clone()
    }
}

impl Write for SharedBuf {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        self.0.lock().unwrap().write(buf)
    }
    fn flush(&mut self) -> io::Result<()> {
        self.0.lock().unwrap().flush()
    }
}

/// The fixed clock the golden tests inject: 2026-05-19T00:00:00Z, matching the
/// Go `TestGoldenFiles` `now`. The in-scope cases derive their week from the
/// filename or epoch, so the value only needs to be deterministic.
pub fn fixed_now() -> Instant {
    Instant::from_civil(Civil {
        year: 2026,
        month: 5,
        day: 19,
        hour: 0,
        minute: 0,
        second: 0,
        nanos: 0,
    })
}

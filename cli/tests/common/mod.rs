//! Shared helpers for the convobs integration tests.

use obsj::obs::{Civil, Instant};

/// The fixed clock the golden tests inject: 2026-05-19T00:00:00Z. The in-scope
/// cases derive their week from the filename or epoch, so the value only needs
/// to be deterministic.
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

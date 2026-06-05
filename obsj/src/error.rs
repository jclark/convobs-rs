//! The obsj error type.
//!
//! A single typed error for the crate, so callers can distinguish a malformed
//! obsj record from a bad RINEX file from a conversion failure, and follow the
//! `source()` chain for I/O failures.

/// An error from reading, writing, or converting observations.
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum Error {
    /// A malformed obsj record or field.
    #[error("{0}")]
    Obsj(String),
    /// A malformed RINEX file (DIY backend), or one that could not be written.
    #[error("{0}")]
    Rinex(String),
    /// An RTCM stream that could not be converted (e.g. week resolution).
    #[error("{0}")]
    Rtcm(String),
    /// A UBX stream that could not be converted.
    #[error("{0}")]
    Ubx(String),
    /// An invalid decimation interval.
    #[error("{0}")]
    Interval(String),
    /// An underlying I/O error.
    #[error(transparent)]
    Io(#[from] std::io::Error),
}

/// A `Result` with the crate [`Error`].
pub type Result<T> = std::result::Result<T, Error>;

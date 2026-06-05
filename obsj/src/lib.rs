//! obsj — the observation model and JSON-lines format at the centre of the
//! convobs conversions.
//!
//! The crate is a dependency-light leaf: the core model, serde (de)serialization,
//! the loss-of-lock `arc` machinery, and a semantic [`diff`] all build on nothing
//! heavier than serde. Converters (RTCM, UBX) and the self-contained RINEX
//! backend live behind cargo features so consumers pay only for what they use.

pub mod arc;
pub mod diff;
pub mod freq;
pub mod json;
pub mod obs;
pub mod sink;

#[cfg(feature = "rinexobs")]
pub mod rinexobs;

#[cfg(feature = "rtcm")]
pub mod rtcm;

#[cfg(feature = "ubx")]
pub mod ubx;

pub use arc::{ArcToLl, LossOfLockSink};
pub use json::{read_obsj, ObsJsonSink};
pub use obs::{GpsTime, Metadata, SatId, SigId, SignalObservation, SignalValues};
pub use sink::Sink;

//! convobs — a Rust port of satpulse's `convobs`. Converts GNSS raw observation
//! inputs (RTCM MSM7, UBX RXM-RAWX, RINEX, obsj) to RINEX or obsj output, with
//! semantically identical results to the Go tool.

pub mod cli;
pub mod crc24q;
pub mod diff;
pub mod freq;
pub mod obs;
pub mod rinexobs;
pub mod obsj;
pub mod packetlog;
pub mod rinexio;
pub mod rtcm;
pub mod sink;
pub mod ubx;

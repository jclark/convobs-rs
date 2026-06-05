//! Carrier-frequency table for RINEX system/signal identifiers, including
//! the GLONASS FDMA channel arithmetic.

use crate::obs::SigId;

/// Carrier frequency in Hz for a system letter and signal id. `frq` is the
/// GLONASS FDMA channel and must be `Some` for GLONASS. Returns `None` when the
/// system, band, or channel is unrecognised.
pub fn signal_frequency_hz(sys: u8, sig: SigId, frq: Option<i8>) -> Option<f64> {
    signal_frequency_mhz(sys, sig, frq).map(|f| f * 1e6)
}

fn signal_frequency_mhz(sys: u8, sig: SigId, frq: Option<i8>) -> Option<f64> {
    let band = sig.band();
    match sys {
        b'G' => match band {
            b'1' => Some(1575.420),
            b'2' => Some(1227.600),
            b'5' => Some(1176.450),
            _ => None,
        },
        b'R' => {
            let k = frq? as f64;
            match band {
                b'1' => Some(1602.000 + k * 0.5625),
                b'2' => Some(1246.000 + k * 0.4375),
                _ => None,
            }
        }
        b'E' => match band {
            b'1' => Some(1575.420),
            b'5' => Some(1176.450),
            b'6' => Some(1278.750),
            b'7' => Some(1207.140),
            b'8' => Some(1191.795),
            _ => None,
        },
        b'S' => match band {
            b'1' => Some(1575.420),
            b'5' => Some(1176.450),
            _ => None,
        },
        b'J' => match band {
            b'1' => Some(1575.420),
            b'2' => Some(1227.600),
            b'5' => Some(1176.450),
            b'6' => Some(1278.750),
            _ => None,
        },
        b'C' => match band {
            b'1' => Some(1575.420),
            b'2' => Some(1561.098),
            b'5' => Some(1176.450),
            b'6' => Some(1268.520),
            b'7' => Some(1207.140),
            _ => None,
        },
        b'I' => match band {
            b'5' => Some(1176.450),
            b'9' => Some(2492.028),
            _ => None,
        },
        _ => None,
    }
}

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

#[cfg(test)]
mod tests {
    use super::*;

    fn sig(s: &[u8; 2]) -> SigId {
        SigId(*s)
    }

    #[test]
    fn fixed_band_frequencies() {
        // A representative entry from each fixed (non-GLONASS) system/band.
        assert_eq!(signal_frequency_hz(b'G', sig(b"1C"), None), Some(1575.420e6));
        assert_eq!(signal_frequency_hz(b'G', sig(b"2L"), None), Some(1227.600e6));
        assert_eq!(signal_frequency_hz(b'G', sig(b"5I"), None), Some(1176.450e6));
        assert_eq!(signal_frequency_hz(b'E', sig(b"5Q"), None), Some(1176.450e6));
        assert_eq!(signal_frequency_hz(b'E', sig(b"6B"), None), Some(1278.750e6));
        assert_eq!(signal_frequency_hz(b'C', sig(b"2I"), None), Some(1561.098e6));
        assert_eq!(signal_frequency_hz(b'J', sig(b"1C"), None), Some(1575.420e6));
        assert_eq!(signal_frequency_hz(b'S', sig(b"5I"), None), Some(1176.450e6));
        assert_eq!(signal_frequency_hz(b'I', sig(b"9A"), None), Some(2492.028e6));
    }

    #[test]
    fn glonass_fdma_channel_arithmetic() {
        // G1: 1602 + k*0.5625 MHz; G2: 1246 + k*0.4375 MHz.
        assert_eq!(
            signal_frequency_hz(b'R', sig(b"1C"), Some(0)),
            Some(1602.000e6)
        );
        assert_eq!(
            signal_frequency_hz(b'R', sig(b"1C"), Some(-7)),
            Some((1602.000 + -7.0 * 0.5625) * 1e6)
        );
        assert_eq!(
            signal_frequency_hz(b'R', sig(b"1C"), Some(6)),
            Some((1602.000 + 6.0 * 0.5625) * 1e6)
        );
        assert_eq!(
            signal_frequency_hz(b'R', sig(b"2C"), Some(3)),
            Some((1246.000 + 3.0 * 0.4375) * 1e6)
        );
    }

    #[test]
    fn glonass_requires_channel() {
        // GLONASS without the FDMA channel has no defined frequency.
        assert_eq!(signal_frequency_hz(b'R', sig(b"1C"), None), None);
        assert_eq!(signal_frequency_hz(b'R', sig(b"2C"), None), None);
    }

    #[test]
    fn unknown_system_or_band() {
        assert_eq!(signal_frequency_hz(b'Z', sig(b"1C"), None), None);
        // GPS has no band 9.
        assert_eq!(signal_frequency_hz(b'G', sig(b"9X"), None), None);
        // GLONASS has no band 5.
        assert_eq!(signal_frequency_hz(b'R', sig(b"5I"), Some(0)), None);
    }
}

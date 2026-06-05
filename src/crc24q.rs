//! CRC-24Q (the RTCM/Qualcomm CRC), matching `github.com/jclark/crc24q`.

const POLY: u32 = 0x0186_4CFB;

const TABLE: [u32; 256] = make_table();

const fn make_table() -> [u32; 256] {
    let mut table = [0u32; 256];
    let mut i = 0usize;
    while i < 256 {
        let mut crc = (i as u32) << 16;
        let mut j = 0;
        while j < 8 {
            crc <<= 1;
            if crc & 0x0100_0000 != 0 {
                crc ^= POLY;
            }
            j += 1;
        }
        table[i] = crc & 0x00FF_FFFF;
        i += 1;
    }
    table
}

/// CRC-24Q over `data`.
pub fn checksum(data: &[u8]) -> u32 {
    let mut crc: u32 = 0;
    for &b in data {
        crc = (crc << 8) ^ TABLE[(b ^ (crc >> 16) as u8) as usize];
    }
    crc & 0x00FF_FFFF
}

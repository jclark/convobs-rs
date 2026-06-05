//! SatPulse packet-log (JSONL) parsing. Each line is one logged packet; for
//! conversion we need its timestamp, tag, payload (`bin` hex or `ascii`), and
//! the `out` direction flag.

use serde::Deserialize;
use std::borrow::Cow;

#[derive(Deserialize)]
pub struct Entry<'a> {
    #[serde(default, borrow)]
    pub t: Option<Cow<'a, str>>,
    #[serde(default, borrow)]
    pub tag: Option<Cow<'a, str>>,
    #[serde(default, borrow)]
    pub bin: Option<Cow<'a, str>>,
    #[serde(default, borrow)]
    pub ascii: Option<Cow<'a, str>>,
    #[serde(default)]
    pub out: bool,
}

impl<'a> Entry<'a> {
    pub fn parse(line: &'a str) -> Result<Entry<'a>, String> {
        serde_json::from_str(line).map_err(|e| e.to_string())
    }

    pub fn tag_str(&self) -> &str {
        self.tag.as_deref().unwrap_or("")
    }
}

/// Decodes a lowercase/uppercase hex string into `out` (cleared first).
/// Returns an error on odd length or non-hex characters.
pub fn hex_decode(s: &str, out: &mut Vec<u8>) -> Result<(), String> {
    out.clear();
    let b = s.as_bytes();
    if b.len() % 2 != 0 {
        return Err("odd-length hex string".to_string());
    }
    out.reserve(b.len() / 2);
    let mut i = 0;
    while i < b.len() {
        let hi = hex_val(b[i])?;
        let lo = hex_val(b[i + 1])?;
        out.push((hi << 4) | lo);
        i += 2;
    }
    Ok(())
}

fn hex_val(c: u8) -> Result<u8, String> {
    match c {
        b'0'..=b'9' => Ok(c - b'0'),
        b'a'..=b'f' => Ok(c - b'a' + 10),
        b'A'..=b'F' => Ok(c - b'A' + 10),
        _ => Err(format!("invalid hex byte {:?}", c as char)),
    }
}

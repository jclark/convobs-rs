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

/// Decodes a lowercase/uppercase hex string into `out` (cleared first), using
/// `faster-hex`'s SIMD decoder — the dominant cost when converting packet logs.
pub fn hex_decode(s: &str, out: &mut Vec<u8>) -> Result<(), String> {
    let b = s.as_bytes();
    if !b.len().is_multiple_of(2) {
        return Err("odd-length hex string".to_string());
    }
    out.clear();
    out.resize(b.len() / 2, 0);
    faster_hex::hex_decode(b, out).map_err(|e| format!("invalid hex string: {e}"))
}

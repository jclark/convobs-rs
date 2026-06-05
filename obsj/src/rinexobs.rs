//! A self-contained RINEX observation reader and writer.
//!
//! Unlike the `rinex` crate, this carries an *optional* observation value, so a
//! loss-of-lock marker on a pseudorange-only signal (a blank carrier-phase
//! field that exists only to hold the LLI) round-trips faithfully. It buffers
//! observations and emits the file on flush; decimation upstream keeps the
//! buffer small.

use crate::arc::{ArcToLl, LossOfLockSink};
use crate::obs::*;
use crate::sink::Sink;
use std::collections::{HashMap, HashSet};
use std::fmt::Write as _;
use std::io::{self, BufRead, Write};

const OBS_VALUE_WIDTH: usize = 14;
const DEFAULT_RINEX_VERSION: &str = "3.04";
const SYSTEM_ORDER: [u8; 7] = [b'G', b'R', b'E', b'J', b'C', b'I', b'S'];

// ---------------------------------------------------------------------------
// Writer
// ---------------------------------------------------------------------------

/// Buffers observations and writes a RINEX file on flush.
pub struct RinexSink<W: Write> {
    w: W,
    meta: Metadata,
    obs: Vec<SignalObservation>,
}

impl<W: Write> RinexSink<W> {
    pub fn new(w: W) -> Self {
        RinexSink {
            w,
            meta: Metadata::default(),
            obs: Vec::new(),
        }
    }
}

impl<W: Write> Sink for RinexSink<W> {
    fn metadata(&mut self, m: &Metadata) -> io::Result<()> {
        self.meta.merge(m);
        Ok(())
    }
    fn observation(&mut self, o: &SignalObservation) -> io::Result<()> {
        self.obs.push(*o);
        Ok(())
    }
    fn flush(&mut self) -> io::Result<()> {
        let mut meta = self.meta.clone();
        write_observation_file(&mut self.w, &mut meta, &self.obs)?;
        self.w.flush()
    }
}

struct ObsField {
    val: Option<f64>,
    lli: u8,
}

struct Epoch {
    t: GpsTime,
    sats: Vec<SatId>,
    obs: HashMap<SatId, HashMap<ObsCode, ObsField>>,
}

struct ObsFile {
    codes: HashMap<u8, Vec<ObsCode>>,
    epochs: Vec<Epoch>,
    frq: HashMap<SatId, i8>,
    first: GpsTime,
    last: GpsTime,
}

/// Writes a complete RINEX observation file.
pub fn write_observation_file<W: Write>(
    w: &mut W,
    meta: &mut Metadata,
    obs: &[SignalObservation],
) -> io::Result<()> {
    let f = build_obs_file(obs).map_err(|e| io::Error::new(io::ErrorKind::InvalidData, e))?;
    if meta.version.is_empty() {
        meta.version = DEFAULT_RINEX_VERSION.to_string();
    }
    let mut out = String::with_capacity(64 * 1024);
    write_header(&mut out, meta, &f);
    flush_str(w, &mut out)?;
    write_epochs(w, &mut out, &f)?;
    flush_str(w, &mut out)?;
    Ok(())
}

fn flush_str<W: Write>(w: &mut W, s: &mut String) -> io::Result<()> {
    w.write_all(s.as_bytes())?;
    s.clear();
    Ok(())
}

fn build_obs_file(obs: &[SignalObservation]) -> Result<ObsFile, String> {
    if obs.is_empty() {
        return Err("rinex: no observations".to_string());
    }
    let mut sorted: Vec<SignalObservation> = obs.to_vec();
    sorted.sort_by_key(|o| o.t.0); // stable

    let mut epoch_index: HashMap<i64, usize> = HashMap::new();
    let mut epochs: Vec<Epoch> = Vec::new();
    let mut frq: HashMap<SatId, i8> = HashMap::new();
    let mut arc = ArcToLl::new();
    let mut seen_codes: HashMap<u8, Vec<ObsCode>> = HashMap::new();
    let mut seen_code_set: HashMap<u8, HashSet<ObsCode>> = HashMap::new();
    let mut first = sorted[0].t;
    let mut last = sorted[0].t;

    for o in &sorted {
        if !o.sat.is_valid() {
            return Err(format!("rinex: invalid satellite {:?}", o.sat.as_str()));
        }
        if !o.sig.is_valid() {
            return Err(format!("rinex: invalid signal {:?}", o.sig.as_str()));
        }
        if o.t.0 < first.0 {
            first = o.t;
        }
        if o.t.0 > last.0 {
            last = o.t;
        }
        let ei = *epoch_index.entry(o.t.0).or_insert_with(|| {
            epochs.push(Epoch {
                t: o.t,
                sats: Vec::new(),
                obs: HashMap::new(),
            });
            epochs.len() - 1
        });
        let e = &mut epochs[ei];
        if !e.obs.contains_key(&o.sat) {
            e.sats.push(o.sat);
            e.obs.insert(o.sat, HashMap::new());
        }
        if let Some(v) = o.v.frq {
            frq.insert(o.sat, v);
        }
        let changed = arc.lli(
            SignalKey {
                sat: o.sat,
                sig: o.sig,
            },
            o.v.arc,
        );
        let dst = e.obs.get_mut(&o.sat).unwrap();
        add_signal_observation(dst, o, changed);
        add_writer_codes(&mut seen_codes, &mut seen_code_set, o, changed);
    }

    for codes in seen_codes.values_mut() {
        sort_observation_codes(codes);
    }
    Ok(ObsFile {
        codes: seen_codes,
        epochs,
        frq,
        first,
        last,
    })
}

fn writer_codes(o: &SignalObservation, arc_changed: bool) -> [Option<ObsCode>; 4] {
    let mut out = [None; 4];
    if o.v.pr.is_some() {
        out[0] = Some(o.sig.code(TYPE_CODE));
    }
    if o.v.pr.is_some()
        || o.v.cp.is_some()
        || o.v.dop.is_some()
        || o.v.cn0.is_some()
        || o.v.rinex_lli(arc_changed) != 0
    {
        out[1] = Some(o.sig.code(TYPE_PHASE));
    }
    if o.v.dop.is_some() {
        out[2] = Some(o.sig.code(TYPE_DOPPLER));
    }
    if o.v.cn0.is_some() {
        out[3] = Some(o.sig.code(TYPE_SIGNAL_STRENGTH));
    }
    out
}

fn add_writer_codes(
    seen: &mut HashMap<u8, Vec<ObsCode>>,
    seen_set: &mut HashMap<u8, HashSet<ObsCode>>,
    o: &SignalObservation,
    arc_changed: bool,
) {
    let sys = o.system();
    if sys == 0 {
        return;
    }
    let set = seen_set.entry(sys).or_default();
    let list = seen.entry(sys).or_default();
    for code in writer_codes(o, arc_changed).into_iter().flatten() {
        if set.insert(code) {
            list.push(code);
        }
    }
}

fn add_signal_observation(
    dst: &mut HashMap<ObsCode, ObsField>,
    o: &SignalObservation,
    arc_changed: bool,
) {
    if let Some(pr) = o.v.pr {
        add_obs_field(dst, o.sig.code(TYPE_CODE), Some(pr), 0);
    }
    let lli = o.v.rinex_lli(arc_changed);
    if let Some(cp) = o.v.cp {
        add_obs_field(dst, o.sig.code(TYPE_PHASE), Some(cp), lli);
    } else if lli != 0 {
        add_obs_field(dst, o.sig.code(TYPE_PHASE), None, lli);
    }
    if let Some(dop) = o.v.dop {
        add_obs_field(dst, o.sig.code(TYPE_DOPPLER), Some(dop), 0);
    }
    if let Some(cn0) = o.v.cn0 {
        add_obs_field(dst, o.sig.code(TYPE_SIGNAL_STRENGTH), Some(cn0 as f64), 0);
    }
}

fn add_obs_field(dst: &mut HashMap<ObsCode, ObsField>, code: ObsCode, val: Option<f64>, lli: u8) {
    dst.entry(code).or_insert(ObsField { val, lli });
}

// ---- header ----

fn write_header(out: &mut String, meta: &Metadata, f: &ObsFile) {
    let sys = header_system(&f.codes);
    let mut line = String::new();
    let _ = write!(
        line,
        "{:>9.9}           OBSERVATION DATA    {:<20.20}",
        meta.version, sys
    );
    header_line(out, &line, "RINEX VERSION / TYPE");

    let date = match meta.run.date {
        Some(d) => format_run_date(d),
        None => String::new(),
    };
    line.clear();
    let _ = write!(
        line,
        "{:<20.20}{:<20.20}{:<20.20}",
        meta.run.program, meta.run.by, date
    );
    header_line(out, &line, "PGM / RUN BY / DATE");

    for c in &meta.comment {
        header_line(out, c, "COMMENT");
    }
    header_line(out, &meta.marker.name, "MARKER NAME");
    header_line(out, &meta.marker.number, "MARKER NUMBER");
    header_line(out, &meta.marker.type_, "MARKER TYPE");
    line.clear();
    let _ = write!(line, "{:<20.20}{:<40.40}", meta.observer, meta.agency);
    header_line(out, &line, "OBSERVER / AGENCY");
    line.clear();
    let _ = write!(
        line,
        "{:<20.20}{:<20.20}{:<20.20}",
        meta.receiver.number, meta.receiver.type_, meta.receiver.version
    );
    header_line(out, &line, "REC # / TYPE / VERS");
    line.clear();
    let _ = write!(
        line,
        "{:<20.20}{:<20.20}",
        meta.antenna.number, meta.antenna.type_
    );
    header_line(out, &line, "ANT # / TYPE");

    let pos = meta.approx_position.unwrap_or([0.0; 3]);
    line.clear();
    let _ = write!(line, "{:14.4}{:14.4}{:14.4}", pos[0], pos[1], pos[2]);
    header_line(out, &line, "APPROX POSITION XYZ");
    let delta = meta.antenna_delta.unwrap_or([0.0; 3]);
    line.clear();
    let _ = write!(line, "{:14.4}{:14.4}{:14.4}", delta[0], delta[1], delta[2]);
    header_line(out, &line, "ANTENNA: DELTA H/E/N");

    for sys in ordered_systems(&f.codes) {
        write_obs_types(out, sys, &f.codes[&sys]);
    }
    if let Some(interval) = meta.interval {
        line.clear();
        let _ = write!(line, "{:10.3}", interval);
        header_line(out, &line, "INTERVAL");
    }
    write_time_header(out, f.first, "TIME OF FIRST OBS");
    write_time_header(out, f.last, "TIME OF LAST OBS");
    if !f.frq.is_empty() {
        write_glonass_freq(out, &f.frq);
    }
    if f.codes.get(&b'R').is_some_and(|c| !c.is_empty()) {
        header_line(
            out,
            " C1C    0.000 C1P    0.000 C2C    0.000 C2P    0.000",
            "GLONASS COD/PHS/BIS",
        );
    }
    if let Some(ls) = meta.leap_seconds {
        line.clear();
        let _ = write!(line, "{:6}", ls);
        header_line(out, &line, "LEAP SECONDS");
    }
    header_line(out, "", "END OF HEADER");
}

fn header_line(out: &mut String, content: &str, label: &str) {
    let content = if content.len() > 60 {
        &content[..60]
    } else {
        content
    };
    let _ = writeln!(out, "{:<60}{:<20}", content, label);
}

fn header_system(codes: &HashMap<u8, Vec<ObsCode>>) -> &'static str {
    let systems = ordered_systems(codes);
    if systems.len() != 1 {
        return "M: Mixed";
    }
    match systems[0] {
        b'G' => "G: GPS",
        b'R' => "R: GLONASS",
        b'E' => "E: Galileo",
        b'J' => "J: QZSS",
        b'C' => "C: BDS",
        b'I' => "I: NavIC",
        b'S' => "S: SBAS",
        _ => "M: Mixed",
    }
}

fn ordered_systems(codes: &HashMap<u8, Vec<ObsCode>>) -> Vec<u8> {
    let mut systems = Vec::new();
    for &sys in &SYSTEM_ORDER {
        if codes.get(&sys).is_some_and(|c| !c.is_empty()) {
            systems.push(sys);
        }
    }
    let mut others: Vec<u8> = codes
        .iter()
        .filter(|(s, c)| !c.is_empty() && !SYSTEM_ORDER.contains(s))
        .map(|(s, _)| *s)
        .collect();
    others.sort_unstable();
    systems.extend(others);
    systems
}

fn write_obs_types(out: &mut String, sys: u8, codes: &[ObsCode]) {
    let mut i = 0;
    while i < codes.len() {
        let mut content = String::new();
        if i == 0 {
            let _ = write!(content, "{}{:5}", sys as char, codes.len());
        } else {
            content.push_str("      ");
        }
        let end = (i + 13).min(codes.len());
        for code in &codes[i..end] {
            let _ = write!(content, " {:<3}", code.as_str());
        }
        header_line(out, &content, "SYS / # / OBS TYPES");
        i += 13;
    }
}

fn write_time_header(out: &mut String, t: GpsTime, label: &str) {
    let c = t.civil();
    let sec = c.second as f64 + c.nanos as f64 / 1e9;
    let mut content = String::new();
    let _ = write!(
        content,
        "{:6}{:>6}{:>6}{:>6}{:>6}{:>13}     GPS",
        c.year,
        two_digit(c.month),
        two_digit(c.day),
        two_digit(c.hour),
        two_digit(c.minute),
        second_field(sec)
    );
    header_line(out, &content, label);
}

fn two_digit(n: u32) -> String {
    format!("{:02}", n)
}

fn second_field(sec: f64) -> String {
    format!("{:010.7}", sec)
}

fn write_glonass_freq(out: &mut String, frq: &HashMap<SatId, i8>) {
    let mut sats: Vec<SatId> = frq.keys().copied().filter(|s| s.system() == b'R').collect();
    sats.sort_unstable();
    let mut i = 0;
    while i < sats.len() {
        let mut content = String::new();
        if i == 0 {
            let _ = write!(content, "{:3}", sats.len());
        } else {
            content.push_str("   ");
        }
        let end = (i + 8).min(sats.len());
        for sat in &sats[i..end] {
            let _ = write!(content, " {:<3}{:3}", sat.as_str(), frq[sat]);
        }
        header_line(out, &content, "GLONASS SLOT / FRQ #");
        i += 8;
    }
}

fn format_run_date(d: Instant) -> String {
    let c = d.civil();
    format!(
        "{:04}{:02}{:02} {:02}{:02}{:02} UTC",
        c.year, c.month, c.day, c.hour, c.minute, c.second
    )
}

// ---- epochs ----

fn write_epochs<W: Write>(w: &mut W, line: &mut String, f: &ObsFile) -> io::Result<()> {
    for e in &f.epochs {
        let c = e.t.civil();
        let sec = c.second as f64 + c.nanos as f64 / 1e9;
        line.clear();
        let _ = write!(
            line,
            "> {:04} {:02} {:02} {:02} {:02} {}  0{:3}",
            c.year,
            c.month,
            c.day,
            c.hour,
            c.minute,
            second_field(sec),
            e.sats.len()
        );
        while line.len() < 56 {
            line.push(' ');
        }
        line.push('\n');
        let codes_default: Vec<ObsCode> = Vec::new();
        for sat in &e.sats {
            line.push_str(sat.as_str());
            let codes = f.codes.get(&sat.system()).unwrap_or(&codes_default);
            let fields = &e.obs[sat];
            for code in codes {
                match fields.get(code) {
                    None => line.push_str("                "),
                    Some(field) => append_obs_field(line, field),
                }
            }
            line.push('\n');
        }
        w.write_all(line.as_bytes())?;
        line.clear();
    }
    Ok(())
}

fn append_obs_field(line: &mut String, field: &ObsField) {
    let lli = if field.lli != 0 {
        (b'0' + field.lli) as char
    } else {
        ' '
    };
    let mut text = String::new();
    if let Some(v) = field.val {
        let _ = write!(text, "{:.3}", v);
    }
    for _ in text.len()..OBS_VALUE_WIDTH {
        line.push(' ');
    }
    line.push_str(&text);
    line.push(lli);
    line.push(' ');
}

fn sort_observation_codes(codes: &mut [ObsCode]) {
    // stable insertion sort: by signal (chars 1..3), then C/L/D/S order
    for i in 1..codes.len() {
        let mut j = i;
        while j > 0 && less_observation_code(codes[j], codes[j - 1]) {
            codes.swap(j, j - 1);
            j -= 1;
        }
    }
}

fn less_observation_code(a: ObsCode, b: ObsCode) -> bool {
    let asig = &a.0[1..];
    let bsig = &b.0[1..];
    if asig != bsig {
        return asig < bsig;
    }
    observation_type_index(a.0[0]) < observation_type_index(b.0[0])
}

fn observation_type_index(b: u8) -> usize {
    match b {
        b'C' => 0,
        b'L' => 1,
        b'D' => 2,
        b'S' => 3,
        _ => 4,
    }
}

// ---------------------------------------------------------------------------
// Reader
// ---------------------------------------------------------------------------

struct ObsHeader {
    meta: Metadata,
    codes: HashMap<u8, Vec<ObsCode>>,
    frq: HashMap<SatId, i8>,
    sys: u8,
    time_system: String,
}

/// Reads a RINEX observation file into the obsj model.
pub fn read_observation_file(
    r: impl BufRead,
) -> Result<(Metadata, Vec<SignalObservation>), crate::error::Error> {
    read_observation_file_impl(r).map_err(crate::error::Error::Rinex)
}

/// The reader proper; its many fixed-column parse checks use plain `String`
/// messages, folded into [`crate::error::Error::Rinex`] at the public boundary.
fn read_observation_file_impl(
    r: impl BufRead,
) -> Result<(Metadata, Vec<SignalObservation>), String> {
    let mut lines = LineReader::new(r);
    let h = read_header(&mut lines)?;
    let obs = read_epochs(&mut lines, &h)?;
    Ok((h.meta, accumulate_arc(obs)))
}

/// Collecting sink, used to drive the [`LossOfLockSink`] over a buffered read.
#[derive(Default)]
struct ObsCollector {
    obs: Vec<SignalObservation>,
}

impl Sink for ObsCollector {
    fn metadata(&mut self, _m: &Metadata) -> io::Result<()> {
        Ok(())
    }
    fn observation(&mut self, o: &SignalObservation) -> io::Result<()> {
        self.obs.push(*o);
        Ok(())
    }
    fn flush(&mut self) -> io::Result<()> {
        Ok(())
    }
}

/// Turns the per-observation `ll` flag the reader sets from RINEX LLI back into
/// the monotonic `arc`, using the same accumulator the converters feed.
fn accumulate_arc(obs: Vec<SignalObservation>) -> Vec<SignalObservation> {
    let mut acc = LossOfLockSink::new(ObsCollector::default());
    for o in &obs {
        let _ = acc.observation(o);
    }
    acc.into_inner().obs
}

struct LineReader<R: BufRead> {
    r: R,
    buf: Vec<u8>,
}

impl<R: BufRead> LineReader<R> {
    fn new(r: R) -> Self {
        LineReader {
            r,
            buf: Vec::with_capacity(256),
        }
    }
    /// Returns the next line (without trailing \r\n), or None at EOF.
    fn next_line(&mut self) -> Result<Option<String>, String> {
        self.buf.clear();
        let n = self
            .r
            .read_until(b'\n', &mut self.buf)
            .map_err(|e| e.to_string())?;
        if n == 0 {
            return Ok(None);
        }
        while matches!(self.buf.last(), Some(b'\n') | Some(b'\r')) {
            self.buf.pop();
        }
        Ok(Some(String::from_utf8_lossy(&self.buf).into_owned()))
    }
}

fn pad_line(s: &str, n: usize) -> String {
    if s.len() >= n {
        s.to_string()
    } else {
        let mut out = String::with_capacity(n);
        out.push_str(s);
        for _ in s.len()..n {
            out.push(' ');
        }
        out
    }
}

fn read_header<R: BufRead>(lines: &mut LineReader<R>) -> Result<ObsHeader, String> {
    let mut h = ObsHeader {
        meta: Metadata::default(),
        codes: HashMap::new(),
        frq: HashMap::new(),
        sys: 0,
        time_system: String::new(),
    };
    let mut obs_type_count: HashMap<u8, usize> = HashMap::new();
    while let Some(raw) = lines.next_line()? {
        let line = pad_line(&raw, 80);
        let content = &line[..60];
        let label = line[60..80].trim();
        match label {
            "RINEX VERSION / TYPE" => {
                if let Some(field) = content[..20].split_whitespace().next() {
                    h.meta.version = field.to_string();
                }
            }
            "PGM / RUN BY / DATE" => {
                h.meta.run.program = content[..20].trim().to_string();
                h.meta.run.by = content[20..40].trim().to_string();
                h.meta.run.date = parse_run_date(&content[40..60]);
            }
            "COMMENT" => {
                h.meta
                    .comment
                    .push(content.trim_end_matches(' ').to_string());
            }
            "MARKER NAME" => h.meta.marker.name = content.trim().to_string(),
            "MARKER NUMBER" => h.meta.marker.number = content.trim().to_string(),
            "MARKER TYPE" => h.meta.marker.type_ = content.trim().to_string(),
            "OBSERVER / AGENCY" => {
                h.meta.observer = content[..20].trim().to_string();
                h.meta.agency = content[20..].trim().to_string();
            }
            "REC # / TYPE / VERS" => {
                h.meta.receiver.number = content[..20].trim().to_string();
                h.meta.receiver.type_ = content[20..40].trim().to_string();
                h.meta.receiver.version = content[40..].trim().to_string();
            }
            "ANT # / TYPE" => {
                h.meta.antenna.number = content[..20].trim().to_string();
                h.meta.antenna.type_ = content[20..40].trim().to_string();
            }
            "APPROX POSITION XYZ" => {
                if let Some(v) = parse_float_triple(content) {
                    h.meta.approx_position = Some(v);
                }
            }
            "ANTENNA: DELTA H/E/N" => {
                if let Some(v) = parse_float_triple(content) {
                    h.meta.antenna_delta = Some(v);
                }
            }
            "SYS / # / OBS TYPES" => {
                h.sys = read_obs_types_header(content, h.sys, &mut h.codes, &mut obs_type_count)?;
            }
            "SYS / SCALE FACTOR" => {
                return Err("rinex: unsupported SYS / SCALE FACTOR header".to_string());
            }
            "SIGNAL STRENGTH UNIT" => read_signal_strength_unit(content)?,
            "INTERVAL" => {
                if let Some(field) = content.split_whitespace().next() {
                    let v: f64 = field
                        .parse()
                        .map_err(|_| format!("rinex: invalid interval {:?}", field))?;
                    h.meta.interval = Some(v);
                }
            }
            "GLONASS SLOT / FRQ #" => read_glonass_freq_header(content, &mut h.frq)?,
            "TIME OF FIRST OBS" => {
                if let Some(sys) = read_time_system(content) {
                    h.time_system = sys;
                }
            }
            "TIME OF LAST OBS" => {
                if h.time_system.is_empty() {
                    if let Some(sys) = read_time_system(content) {
                        h.time_system = sys;
                    }
                }
            }
            "LEAP SECONDS" => {
                if let Some(field) = content.split_whitespace().next() {
                    let n: i64 = field
                        .parse()
                        .map_err(|_| format!("rinex: invalid leap seconds {:?}", field))?;
                    h.meta.leap_seconds = Some(n as i16);
                }
            }
            "END OF HEADER" => {
                if h.time_system.is_empty() {
                    h.time_system = file_time_system(&h.codes);
                }
                return Ok(h);
            }
            _ => {}
        }
    }
    Err("rinex: missing END OF HEADER".to_string())
}

fn read_epochs<R: BufRead>(
    lines: &mut LineReader<R>,
    h: &ObsHeader,
) -> Result<Vec<SignalObservation>, String> {
    let mut obs = Vec::new();
    let leap_seconds = gps_utc_seconds(&h.meta);
    while let Some(line) = lines.next_line()? {
        if line.trim().is_empty() {
            continue;
        }
        if !line.starts_with('>') {
            return Err(format!("rinex: expected epoch line, got {:?}", line));
        }
        let (flag, count, t) = parse_epoch_line(&line)?;
        match flag {
            0 | 1 => {}
            2 | 3 | 5 => {
                skip_records(lines, count, "event records")?;
                continue;
            }
            4 => return Err("rinex: unsupported mid-file header update".to_string()),
            6 => {
                skip_records(lines, count, "cycle-slip records")?;
                continue;
            }
            _ => return Err(format!("rinex: unsupported epoch flag {}", flag)),
        }
        let t = file_to_gps_time(t, &h.time_system, leap_seconds);
        for _ in 0..count {
            let line = lines.next_line()?.ok_or("rinex: unexpected EOF in epoch")?;
            parse_satellite_observation_line(t, &line, h, &mut obs)?;
        }
    }
    Ok(obs)
}

fn skip_records<R: BufRead>(
    lines: &mut LineReader<R>,
    n: usize,
    context: &str,
) -> Result<(), String> {
    for _ in 0..n {
        if lines.next_line()?.is_none() {
            return Err(format!("rinex: unexpected EOF in {}", context));
        }
    }
    Ok(())
}

fn parse_run_date(s: &str) -> Option<Instant> {
    let fields: Vec<&str> = s.split_whitespace().collect();
    if fields.len() < 2 {
        return None;
    }
    let d = fields[0];
    let tm = fields[1];
    if d.len() != 8 || tm.len() != 6 {
        return None;
    }
    let year: i64 = d[0..4].parse().ok()?;
    let month: u32 = d[4..6].parse().ok()?;
    let day: u32 = d[6..8].parse().ok()?;
    let hour: u32 = tm[0..2].parse().ok()?;
    let minute: u32 = tm[2..4].parse().ok()?;
    let second: u32 = tm[4..6].parse().ok()?;
    let c = Civil {
        year,
        month,
        day,
        hour,
        minute,
        second,
        nanos: 0,
    };
    if !c.is_valid() {
        return None;
    }
    Some(Instant::from_civil(c))
}

fn parse_float_triple(s: &str) -> Option<[f64; 3]> {
    let fields: Vec<&str> = s.split_whitespace().collect();
    if fields.len() < 3 {
        return None;
    }
    let mut out = [0.0; 3];
    for i in 0..3 {
        out[i] = fields[i].parse().ok()?;
    }
    Some(out)
}

fn read_obs_types_header(
    content: &str,
    prev: u8,
    codes: &mut HashMap<u8, Vec<ObsCode>>,
    counts: &mut HashMap<u8, usize>,
) -> Result<u8, String> {
    let fields: Vec<&str> = content.split_whitespace().collect();
    if fields.is_empty() {
        return Ok(prev);
    }
    let mut sys = prev;
    let mut i = 0;
    if fields[0].len() == 1 && b"GRESJCI".contains(&fields[0].as_bytes()[0]) {
        sys = fields[0].as_bytes()[0];
        i = 1;
    }
    if sys == 0 {
        return Err("rinex: observation type continuation without system".to_string());
    }
    if fields.len() > i {
        if let Ok(n) = fields[i].parse::<usize>() {
            counts.insert(sys, n);
            i += 1;
        }
    }
    while i < fields.len() {
        let code = fields[i].as_bytes();
        if code.len() != 3 {
            return Err(format!("rinex: invalid observation code {:?}", fields[i]));
        }
        codes
            .entry(sys)
            .or_default()
            .push(ObsCode([code[0], code[1], code[2]]));
        i += 1;
    }
    Ok(sys)
}

fn read_signal_strength_unit(content: &str) -> Result<(), String> {
    let field = content
        .split_whitespace()
        .next()
        .ok_or("rinex: missing signal strength unit")?;
    let unit = field.to_uppercase();
    if unit == "DBHZ" || unit == "DB-HZ" {
        Ok(())
    } else {
        Err(format!(
            "rinex: unsupported signal strength unit {:?}",
            field
        ))
    }
}

fn read_glonass_freq_header(content: &str, frq: &mut HashMap<SatId, i8>) -> Result<(), String> {
    let fields: Vec<&str> = content.split_whitespace().collect();
    if fields.is_empty() {
        return Ok(());
    }
    let mut i = 0;
    if fields[0].parse::<i64>().is_ok() {
        i = 1;
    }
    while i + 1 < fields.len() {
        let n: i64 = fields[i + 1]
            .parse()
            .map_err(|_| format!("rinex: invalid GLONASS frequency {:?}", fields[i + 1]))?;
        if let Ok(sat) = fields[i].parse::<SatId>() {
            frq.insert(sat, n as i8);
        }
        i += 2;
    }
    Ok(())
}

fn read_time_system(content: &str) -> Option<String> {
    let fields: Vec<&str> = content.split_whitespace().collect();
    if fields.len() >= 7 {
        Some(fields[6].to_string())
    } else {
        None
    }
}

fn file_time_system(codes: &HashMap<u8, Vec<ObsCode>>) -> String {
    let systems = ordered_systems(codes);
    if systems.len() == 1 {
        system_time_system(systems[0]).to_string()
    } else {
        "GPS".to_string()
    }
}

fn system_time_system(sys: u8) -> &'static str {
    match sys {
        b'R' => "GLO",
        b'E' => "GAL",
        b'J' => "QZS",
        b'C' => "BDT",
        b'I' => "IRN",
        _ => "GPS",
    }
}

fn gps_utc_seconds(meta: &Metadata) -> i64 {
    match meta.leap_seconds {
        Some(v) => v as i64,
        None => DEFAULT_GPS_UTC_SECONDS as i64,
    }
}

fn file_to_gps_time(t: GpsTime, time_system: &str, leap_seconds: i64) -> GpsTime {
    match time_system {
        "GLO" | "UTC" => GpsTime(t.0 + leap_seconds * 1_000_000_000 / TICK_NS),
        "BDT" => GpsTime(t.0 + BDT_GPS_OFFSET_SECONDS * 1_000_000_000 / TICK_NS),
        _ => t,
    }
}

fn parse_epoch_line(line: &str) -> Result<(i32, usize, GpsTime), String> {
    let line = pad_line(line, 35);
    let flag = parse_epoch_int(line[31..32].trim(), "flag")?;
    let n = parse_epoch_int(line[32..35].trim(), "record count")?;
    if n < 0 {
        return Err(format!(
            "rinex: invalid epoch record count {:?}",
            line[32..35].trim()
        ));
    }
    let mut t = GpsTime(0);
    if flag == 0 || flag == 1 {
        t = parse_rinex_time(
            line[2..6].trim(),
            line[7..9].trim(),
            line[10..12].trim(),
            line[13..15].trim(),
            line[16..18].trim(),
            line[19..29].trim(),
        )?;
    }
    Ok((flag, n as usize, t))
}

fn parse_epoch_int(s: &str, name: &str) -> Result<i32, String> {
    if s.is_empty() {
        return Err(format!("rinex: missing epoch {}", name));
    }
    s.parse()
        .map_err(|_| format!("rinex: invalid epoch {} {:?}", name, s))
}

fn parse_rinex_time(
    year: &str,
    month: &str,
    day: &str,
    hour: &str,
    minute: &str,
    second: &str,
) -> Result<GpsTime, String> {
    let y: i64 = year
        .parse()
        .map_err(|_| format!("rinex: invalid year {:?}", year))?;
    let mo: u32 = month
        .parse()
        .map_err(|_| format!("rinex: invalid month {:?}", month))?;
    let d: u32 = day
        .parse()
        .map_err(|_| format!("rinex: invalid day {:?}", day))?;
    let h: u32 = hour
        .parse()
        .map_err(|_| format!("rinex: invalid hour {:?}", hour))?;
    let m: u32 = minute
        .parse()
        .map_err(|_| format!("rinex: invalid minute {:?}", minute))?;
    let (sec_text, frac_text) = match second.split_once('.') {
        Some((a, b)) => (a, b.to_string()),
        None => (second, String::new()),
    };
    let sec: u32 = sec_text
        .parse()
        .map_err(|_| format!("rinex: invalid second {:?}", second))?;
    let mut frac = frac_text;
    frac.push_str("0000000");
    let tick: i64 = frac[..7]
        .parse()
        .map_err(|_| format!("rinex: invalid fractional second {:?}", second))?;
    let c = Civil {
        year: y,
        month: mo,
        day: d,
        hour: h,
        minute: m,
        second: sec,
        nanos: 0,
    };
    if !c.is_valid() {
        return Err(format!(
            "rinex: invalid epoch date {:04}-{:02}-{:02} {:02}:{:02}:{:02}",
            y, mo, d, h, m, sec
        ));
    }
    let base = GpsTime::from_civil(c);
    Ok(GpsTime(base.0 + tick))
}

fn parse_satellite_observation_line(
    t: GpsTime,
    line: &str,
    h: &ObsHeader,
    out: &mut Vec<SignalObservation>,
) -> Result<(), String> {
    let line = pad_line(line, 3);
    let sat_bytes = line.as_bytes();
    let sat = SatId([sat_bytes[0], sat_bytes[1], sat_bytes[2]]);
    if !sat.is_valid() {
        return Err(format!("rinex: invalid satellite {:?}", &line[..3]));
    }
    let default_codes: Vec<ObsCode> = Vec::new();
    let codes = h.codes.get(&sat.system()).unwrap_or(&default_codes);
    let mut by_sig: HashMap<SigId, SignalObservation> = HashMap::new();
    let mut order: Vec<SigId> = Vec::new();
    for (i, code) in codes.iter().enumerate() {
        let field = rinex_field(&line, i);
        if field.trim().is_empty() {
            continue;
        }
        let sig = code.signal();
        by_sig.entry(sig).or_insert_with(|| {
            order.push(sig);
            let mut o = SignalObservation {
                t,
                sat,
                sig,
                v: SignalValues::default(),
            };
            if let Some(&v) = h.frq.get(&sat) {
                o.v.frq = Some(v);
            }
            o
        });
        let o = by_sig.get_mut(&sig).unwrap();
        add_rinex_field(o, *code, &field);
    }
    for sig in order {
        out.push(by_sig[&sig]);
    }
    Ok(())
}

fn rinex_field(line: &str, i: usize) -> String {
    let start = 3 + i * 16;
    if line.len() <= start {
        return " ".repeat(16);
    }
    let end = start + 16;
    if line.len() < end {
        pad_line(&line[start..], 16)
    } else {
        line[start..end].to_string()
    }
}

fn add_rinex_field(o: &mut SignalObservation, code: ObsCode, field: &str) {
    let bytes = field.as_bytes();
    let val = field[..14].trim();
    match code.0[0] {
        TYPE_CODE => {
            if let Some(v) = parse_obs_float(val) {
                o.v.pr = Some(v);
            }
            if let Some(ssi) = parse_indicator(bytes[15]) {
                add_ssi(o, ssi);
            }
        }
        TYPE_PHASE => {
            if let Some(v) = parse_obs_float(val) {
                o.v.cp = Some(v);
            }
            // The reader emits the per-observation loss-of-lock flag; the
            // accumulator (run after the file is read) turns it into `arc`.
            if let Some(lli) = parse_indicator(bytes[14]) {
                o.v.set_lli(lli);
            }
            if let Some(ssi) = parse_indicator(bytes[15]) {
                add_ssi(o, ssi);
            }
        }
        TYPE_DOPPLER => {
            if let Some(v) = parse_obs_float(val) {
                o.v.dop = Some(v);
            }
        }
        TYPE_SIGNAL_STRENGTH => {
            if let Some(v) = parse_obs_float(val) {
                o.v.cn0 = Some(v as f32);
            }
        }
        _ => {}
    }
}

fn add_ssi(o: &mut SignalObservation, ssi: u8) {
    if ssi == 0 || o.v.cn0.is_some() {
        return;
    }
    o.v.cn0 = Some(cn0_from_ssi(ssi));
}

fn cn0_from_ssi(ssi: u8) -> f32 {
    if ssi <= 1 {
        6.0
    } else if ssi >= 9 {
        57.0
    } else {
        (ssi * 6 + 3) as f32
    }
}

fn parse_obs_float(s: &str) -> Option<f64> {
    if s.is_empty() {
        return None;
    }
    s.parse().ok()
}

fn parse_indicator(b: u8) -> Option<u8> {
    if b.is_ascii_digit() {
        Some(b - b'0')
    } else {
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Cursor;

    fn epoch(secs: i64) -> GpsTime {
        let mut t = GpsTime::from_civil(Civil {
            year: 2026,
            month: 5,
            day: 19,
            hour: 0,
            minute: 0,
            second: 0,
            nanos: 0,
        });
        t.0 += secs * (1_000_000_000 / TICK_NS);
        t
    }

    fn pr_only(t: GpsTime, arc: u32) -> SignalObservation {
        let v = SignalValues {
            pr: Some(20_000_000.0),
            arc,
            ..Default::default()
        };
        SignalObservation {
            t,
            sat: SatId::format(b'G', 1),
            sig: SigId(*b"1C"),
            v,
        }
    }

    #[test]
    fn blank_phase_round_trips() {
        // A pseudorange-only signal that loses lock: `arc` bumps but there is no
        // carrier phase. The writer must emit a blank phase field carrying only
        // the LLI, and the reader must read it back as cp=None with arc restored.
        let obs = vec![pr_only(epoch(0), 0), pr_only(epoch(1), 1)];
        let mut meta = Metadata {
            version: "3.04".to_string(),
            ..Default::default()
        };
        let mut buf = Vec::new();
        write_observation_file(&mut buf, &mut meta, &obs).unwrap();

        let (_, back) = read_observation_file(Cursor::new(buf)).unwrap();
        let first = back.iter().find(|o| o.t == epoch(0)).unwrap();
        let second = back.iter().find(|o| o.t == epoch(1)).unwrap();
        assert_eq!(first.v.arc, 0);
        assert_eq!(second.v.cp, None, "blank phase must read back as cp=None");
        assert_eq!(second.v.arc, 1, "the slip must survive as arc, via the LLI");
    }
}

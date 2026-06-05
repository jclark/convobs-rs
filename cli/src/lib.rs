//! Command-line parsing and conversion orchestration.

pub mod error;
pub mod packetlog;
mod rinex_backend;

use crate::error::Error;
use crate::packetlog::{hex_decode, Entry};
use crate::rinex_backend::{open_rinex_input, parse_backend, read_rinex, rinex_sink};
use obsj::json::{read_obsj, stream_obsj, ObsJsonSink};
use obsj::obs::{
    Antenna, Civil, Instant, Marker, Metadata, MetadataRun, Receiver, SignalObservation,
};
use obsj::rtcm::{self, TimeInterval};
use obsj::sink::{
    decimation_interval_ticks, validate_decimation_interval, DecimationSink, RequireCpFilter, Sink,
};
use obsj::ubx;
use obsj::LossOfLockSink;
use std::fs::File;
use std::io::{self, BufRead, BufReader, BufWriter, Read, Write};
use std::time::SystemTime;

const SECOND_NS: i64 = 1_000_000_000;
const WEEK_SECS: i64 = 7 * 24 * 3600;

/// Hex prefix of an RXM-RAWX UBX frame: sync `B5 62`, class `0x02`, id `0x15`.
/// Used to skip non-RAWX UBX packet-log lines before hex-decoding them.
const UBX_RAWX_HEX_PREFIX: &[u8] = b"b5620215";

#[derive(Clone, Copy, PartialEq)]
enum InputFormat {
    Raw,
    Ubx,
    Rtcm,
    Rinex,
    ObsJson,
}

impl InputFormat {
    fn packet_input(self) -> bool {
        matches!(
            self,
            InputFormat::Raw | InputFormat::Ubx | InputFormat::Rtcm
        )
    }
    fn may_use_rtcm(self) -> bool {
        matches!(self, InputFormat::Raw | InputFormat::Rtcm)
    }
    fn may_use_ubx(self) -> bool {
        matches!(self, InputFormat::Raw | InputFormat::Ubx)
    }
}

#[derive(Clone, Copy, PartialEq)]
enum OutputFormat {
    Rinex,
    ObsJson,
}

#[derive(Clone, Copy, PartialEq)]
enum WeekMode {
    Auto,
    Recent,
    Date,
    Filename,
}

struct Config {
    from: InputFormat,
    to: OutputFormat,
    packet_log: bool,
    require_cp: bool,
    interval_ns: i64,
    week_mode: WeekMode,
    week_date: Instant,
    inputs: Vec<String>,
    output_path: Option<String>,
    rinex_backend: Option<RinexBackend>,
    rtcm_opts: rtcm::Options,
    ubx_opts: ubx::Options,
    meta: Metadata,
}

/// Entry point. Resolves the output writer (`--output`/stdout) and the wall
/// clock, then runs the conversion. Returns a typed error (without exit
/// handling).
pub fn run(args: &[String]) -> Result<(), Error> {
    let now = now_instant();
    let cfg = match parse_args(args, now).map_err(Error::Usage)? {
        Some(c) => c,
        None => return Ok(()), // --help
    };
    let writer = open_writer(cfg.output_path.as_deref())?;
    execute(&cfg, writer, now)
}

/// Test/embedding seam: runs a parsed conversion against a caller-supplied
/// writer and clock — mirrors Go's `convJob{out}.run(now)`. `--output` is
/// ignored here; the provided writer wins, and `now` is injected for
/// determinism — it sets the `run.date` metadata default and bounds RTCM week
/// resolution, so the whole run is reproducible.
pub fn run_to_writer(args: &[String], out: Box<dyn Write>, now: Instant) -> Result<(), Error> {
    let cfg = match parse_args(args, now).map_err(Error::Usage)? {
        Some(c) => c,
        None => return Ok(()), // --help
    };
    execute(&cfg, out, now)
}

/// Dispatches a parsed config to the observation- or packet-input path,
/// threading the output writer and clock through rather than resolving them
/// internally — so the same code drives both `run` and `run_to_writer`.
fn execute(cfg: &Config, writer: Box<dyn Write>, now: Instant) -> Result<(), Error> {
    match cfg.from {
        InputFormat::Rinex | InputFormat::ObsJson => convert_observation_inputs(cfg, writer),
        _ => convert_packet_inputs(cfg, writer, now),
    }
}

// ---------------------------------------------------------------------------
// Public reader API (shared with the diffobs binary)
// ---------------------------------------------------------------------------

pub use crate::rinex_backend::RinexBackend;

/// The format of an observation file, selected by an explicit option — never
/// inferred from the filename. Compression is detected from content.
#[derive(Clone, Copy, PartialEq)]
pub enum ObsFormat {
    Obsj,
    Rinex,
}

/// Parses a `--rinex-backend` value (`auto`/`diy`/`crate`).
pub fn parse_rinex_backend(s: &str) -> std::result::Result<Option<RinexBackend>, String> {
    parse_backend(s)
}

/// Reads an observation file (obsj or RINEX) into the obsj model. Handles gzip
/// from content and, for RINEX, the configured backend (CRINEX auto-engages the
/// crate backend).
pub fn read_obs_file(
    path: &str,
    format: ObsFormat,
    backend: Option<RinexBackend>,
) -> std::result::Result<(Metadata, Vec<SignalObservation>), Error> {
    let br: Box<dyn BufRead> = Box::new(BufReader::new(open_input(path)?));
    let ctx = |e: obsj::Error| Error::conversion(input_error(path, &e.to_string()));
    match format {
        ObsFormat::Obsj => read_obsj(br).map_err(ctx),
        ObsFormat::Rinex => {
            let (backend, br) = open_rinex_input(backend, br).map_err(ctx)?;
            read_rinex(backend, br).map_err(ctx)
        }
    }
}

// ---------------------------------------------------------------------------
// Argument parsing
// ---------------------------------------------------------------------------

fn build_command() -> clap::Command {
    use clap::{Arg, ArgAction, Command};
    Command::new("convobs")
        .no_binary_name(true)
        .disable_help_flag(true)
        .arg(
            Arg::new("help")
                .short('h')
                .long("help")
                .action(ArgAction::SetTrue),
        )
        .arg(Arg::new("output").short('o').long("output"))
        .arg(
            Arg::new("from")
                .short('r')
                .long("from")
                .default_value("raw"),
        )
        .arg(
            Arg::new("packet-log")
                .long("packet-log")
                .action(ArgAction::SetTrue),
        )
        .arg(Arg::new("to").long("to").default_value("rinex"))
        .arg(
            Arg::new("rinex-backend")
                .long("rinex-backend")
                .default_value("auto"),
        )
        .arg(Arg::new("date").long("date"))
        .arg(Arg::new("recent").long("recent").action(ArgAction::SetTrue))
        .arg(
            Arg::new("date-from-filename")
                .short('f')
                .long("date-from-filename")
                .action(ArgAction::SetTrue),
        )
        .arg(Arg::new("interval").long("interval").default_value("0"))
        .arg(
            Arg::new("ppp-ar")
                .short('p')
                .long("ppp-ar")
                .action(ArgAction::SetTrue),
        )
        .arg(Arg::new("header-file").short('H').long("header-file"))
        .arg(Arg::new("rinex-version").long("rinex-version"))
        .arg(Arg::new("program").long("program"))
        .arg(Arg::new("run-by").long("run-by"))
        .arg(Arg::new("antenna").long("antenna"))
        .arg(Arg::new("approx-pos").long("approx-pos"))
        .arg(
            Arg::new("comment")
                .long("comment")
                .action(ArgAction::Append),
        )
        .arg(
            Arg::new("rtcm-strict-prr")
                .long("rtcm-strict-prr")
                .action(ArgAction::SetTrue),
        )
        .arg(
            Arg::new("rtcm-omit-zero-do")
                .long("rtcm-omit-zero-do")
                .action(ArgAction::SetTrue),
        )
        .arg(
            Arg::new("ubx-slip-threshold")
                .long("ubx-slip-threshold")
                .default_value("15"),
        )
        .arg(
            Arg::new("ubx-bds-geo-half-cycle")
                .long("ubx-bds-geo-half-cycle")
                .action(ArgAction::SetTrue),
        )
        .arg(
            Arg::new("unc-omit-do-without-cp")
                .long("unc-omit-do-without-cp")
                .action(ArgAction::SetTrue),
        )
        .arg(Arg::new("inputs").action(ArgAction::Append).num_args(0..))
}

fn parse_args(args: &[String], now: Instant) -> Result<Option<Config>, String> {
    use clap::parser::ValueSource;
    let cmd = build_command();
    let m = cmd.try_get_matches_from(args).map_err(|e| e.to_string())?;
    if m.get_flag("help") {
        print!("{}", USAGE);
        return Ok(None);
    }
    let changed = |id: &str| m.value_source(id) == Some(ValueSource::CommandLine);

    let from = match m.get_one::<String>("from").unwrap().to_lowercase().as_str() {
        "raw" => InputFormat::Raw,
        "ubx" => InputFormat::Ubx,
        "rtcm" => InputFormat::Rtcm,
        "rinex" => InputFormat::Rinex,
        "obsj" => InputFormat::ObsJson,
        "uncb" | "unca" => return Err("Unicore input is not supported in this build".to_string()),
        other => return Err(format!("unsupported input format {:?}", other)),
    };
    let to = match m.get_one::<String>("to").unwrap().to_lowercase().as_str() {
        "rinex" => OutputFormat::Rinex,
        "obsj" => OutputFormat::ObsJson,
        other => return Err(format!("unsupported output format {:?}", other)),
    };
    let packet_log = m.get_flag("packet-log");
    if packet_log && !from.packet_input() {
        return Err("--packet-log is valid only with packet input formats".to_string());
    }

    let date = m
        .get_one::<String>("date")
        .map(String::as_str)
        .unwrap_or("");
    let recent = m.get_flag("recent");
    let date_from_filename = m.get_flag("date-from-filename");
    let n = (!date.is_empty()) as u8 + recent as u8 + date_from_filename as u8;
    if n > 1 {
        return Err(
            "--date, --recent, and --date-from-filename are mutually exclusive".to_string(),
        );
    }
    let mut week_mode = WeekMode::Auto;
    let mut week_date = Instant { secs: 0, nanos: 0 };
    if !date.is_empty() {
        week_date = parse_yyyymmdd(date)?;
        week_mode = WeekMode::Date;
    } else if recent {
        week_mode = WeekMode::Recent;
    } else if date_from_filename {
        week_mode = WeekMode::Filename;
    }

    if packet_log && week_mode != WeekMode::Auto {
        return Err(
            "--date, --recent, and --date-from-filename are not valid with --packet-log"
                .to_string(),
        );
    }
    if week_mode != WeekMode::Auto && !from.may_use_rtcm() {
        return Err(
            "--date, --recent, and --date-from-filename are valid only with raw or RTCM input"
                .to_string(),
        );
    }

    let strict_prr = m.get_flag("rtcm-strict-prr");
    let omit_zero_do = m.get_flag("rtcm-omit-zero-do");
    if strict_prr && !from.may_use_rtcm() {
        return Err("--rtcm-strict-prr is valid only with raw or RTCM input".to_string());
    }
    if omit_zero_do && !from.may_use_rtcm() {
        return Err("--rtcm-omit-zero-do is valid only with raw or RTCM input".to_string());
    }
    if changed("ubx-slip-threshold") && !from.may_use_ubx() {
        return Err("--ubx-slip-threshold is valid only with raw or UBX input".to_string());
    }
    if m.get_flag("unc-omit-do-without-cp") && !matches!(from, InputFormat::Raw) {
        return Err(
            "--unc-omit-do-without-cp is valid only with raw, UNCB, or UNCA input".to_string(),
        );
    }
    if changed("ubx-bds-geo-half-cycle") && !from.may_use_ubx() {
        return Err("--ubx-bds-geo-half-cycle is valid only with raw or UBX input".to_string());
    }

    let interval: f64 = m
        .get_one::<String>("interval")
        .unwrap()
        .parse()
        .map_err(|_| "--interval must be a finite non-negative number of seconds".to_string())?;
    if !interval.is_finite() || interval < 0.0 {
        return Err("--interval must be a finite non-negative number of seconds".to_string());
    }
    let interval_ns = (interval * SECOND_NS as f64).round() as i64;
    if interval > 0.0 {
        validate_decimation_interval(interval_ns).map_err(|e| e.to_string())?;
    }

    let inputs: Vec<String> = m
        .get_many::<String>("inputs")
        .map(|v| v.cloned().collect())
        .unwrap_or_default();
    if inputs.is_empty() {
        return Err("expected at least one input file".to_string());
    }
    if week_mode == WeekMode::Filename && inputs.iter().any(|p| p == "-") {
        return Err("--date-from-filename is not valid with stdin".to_string());
    }

    let slip_threshold: u8 = m
        .get_one::<String>("ubx-slip-threshold")
        .unwrap()
        .parse()
        .map_err(|_| "--ubx-slip-threshold must be a number 0-255".to_string())?;

    let rinex_backend = parse_backend(m.get_one::<String>("rinex-backend").unwrap())?;
    let touches_rinex = from == InputFormat::Rinex || to == OutputFormat::Rinex;
    if rinex_backend.is_some() && !touches_rinex {
        return Err("--rinex-backend is valid only with RINEX input or output".to_string());
    }

    let mut meta = Metadata::default();
    if let Some(path) = m.get_one::<String>("header-file") {
        let text = std::fs::read_to_string(path).map_err(|e| format!("{}: {}", path, e))?;
        meta = read_header_file(&text).map_err(|e| format!("{}: {}", path, e))?;
    }
    set_metadata_defaults(&mut meta, now, interval_ns);
    apply_metadata_flags(&mut meta, &m, changed)?;

    Ok(Some(Config {
        from,
        to,
        packet_log,
        require_cp: m.get_flag("ppp-ar"),
        interval_ns,
        week_mode,
        week_date,
        inputs,
        output_path: m.get_one::<String>("output").cloned(),
        rinex_backend,
        rtcm_opts: rtcm::Options {
            use_spec_phase_range_rate_sign: strict_prr,
            omit_zero_do,
        },
        ubx_opts: ubx::Options {
            slip_threshold,
            bds_geo_half_cycle: m.get_flag("ubx-bds-geo-half-cycle"),
        },
        meta,
    }))
}

const USAGE: &str = "usage: convobs [options] input...\n\
  -o, --output PATH        output observation file (default stdout)\n\
  -r, --from FORMAT        raw|ubx|rtcm|rinex|obsj (default raw)\n\
      --packet-log         input is a JSONL packet log\n\
      --to FORMAT          rinex|obsj (default rinex)\n\
      --date YYYYMMDD      RTCM observation civil date\n\
      --recent             infer RTCM observations within the last week\n\
  -f, --date-from-filename infer RTCM date from input filename\n\
      --interval SECONDS   observation decimation interval\n\
  -p, --ppp-ar             produce output optimized for PPP-AR\n\
  -H, --header-file PATH   TOML RINEX header metadata file\n";

fn parse_yyyymmdd(s: &str) -> Result<Instant, String> {
    if s.len() != 8 {
        return Err("--date must be in YYYYMMDD format".to_string());
    }
    let err = || "--date must be a valid YYYYMMDD date".to_string();
    let year: i64 = s[0..4].parse().map_err(|_| err())?;
    let month: u32 = s[4..6].parse().map_err(|_| err())?;
    let day: u32 = s[6..8].parse().map_err(|_| err())?;
    if !valid_date(year, month, day) {
        return Err(err());
    }
    Ok(date_instant(year, month, day))
}

fn date_instant(year: i64, month: u32, day: u32) -> Instant {
    Instant::from_civil(Civil {
        year,
        month,
        day,
        hour: 0,
        minute: 0,
        second: 0,
        nanos: 0,
    })
}

fn valid_date(year: i64, month: u32, day: u32) -> bool {
    Civil {
        year,
        month,
        day,
        hour: 0,
        minute: 0,
        second: 0,
        nanos: 0,
    }
    .is_valid()
}

// ---------------------------------------------------------------------------
// Metadata
// ---------------------------------------------------------------------------

fn set_metadata_defaults(meta: &mut Metadata, now: Instant, interval_ns: i64) {
    if meta.version.is_empty() {
        meta.version = "3.04".to_string();
    }
    if meta.run.program.is_empty() {
        meta.run.program = "convobs".to_string();
    }
    if meta.run.by.is_empty() {
        let mut by = std::env::var("USER").unwrap_or_default();
        if by.len() > 20 {
            by.truncate(20);
        }
        meta.run.by = by;
    }
    if meta.run.date.is_none() {
        meta.run.date = Some(now);
    }
    if meta.interval.is_none() && interval_ns != 0 {
        meta.interval = Some(interval_ns as f64 / SECOND_NS as f64);
    }
}

fn apply_metadata_flags(
    meta: &mut Metadata,
    m: &clap::ArgMatches,
    changed: impl Fn(&str) -> bool,
) -> Result<(), String> {
    if changed("rinex-version") {
        meta.version = m.get_one::<String>("rinex-version").unwrap().clone();
    }
    if changed("program") {
        meta.run.program = m.get_one::<String>("program").unwrap().clone();
    }
    if changed("run-by") {
        meta.run.by = m.get_one::<String>("run-by").unwrap().clone();
    }
    if changed("antenna") {
        meta.antenna.type_ = m.get_one::<String>("antenna").unwrap().clone();
    }
    if changed("approx-pos") {
        let s = m.get_one::<String>("approx-pos").unwrap();
        if s.is_empty() {
            meta.approx_position = None;
        } else {
            meta.approx_position = Some(parse_xyz(s, "--approx-pos")?);
        }
    }
    if changed("comment") {
        meta.comment = m
            .get_many::<String>("comment")
            .map(|v| v.cloned().collect())
            .unwrap_or_default();
    }
    Ok(())
}

fn parse_xyz(s: &str, opt: &str) -> Result<[f64; 3], String> {
    let parts: Vec<&str> = s.split(',').collect();
    if parts.len() != 3 {
        return Err(format!(
            "{} must contain three comma-separated numbers",
            opt
        ));
    }
    let mut out = [0.0; 3];
    for (i, p) in parts.iter().enumerate() {
        out[i] = p
            .trim()
            .parse()
            .map_err(|_| format!("{} contains invalid number {:?}", opt, p))?;
    }
    Ok(out)
}

#[derive(serde::Deserialize)]
#[serde(deny_unknown_fields)]
struct TomlMeta {
    version: Option<String>,
    run: Option<TomlRun>,
    comment: Option<TomlComment>,
    marker: Option<TomlMarker>,
    observer: Option<String>,
    agency: Option<String>,
    receiver: Option<TomlReceiver>,
    antenna: Option<TomlAntenna>,
    #[serde(rename = "approxPosition")]
    approx_position: Option<[f64; 3]>,
    #[serde(rename = "antennaDelta")]
    antenna_delta: Option<[f64; 3]>,
    interval: Option<f64>,
    #[serde(rename = "leapSeconds")]
    leap_seconds: Option<i16>,
}

#[derive(serde::Deserialize)]
#[serde(deny_unknown_fields)]
struct TomlRun {
    program: Option<String>,
    by: Option<String>,
    date: Option<toml::value::Datetime>,
}
#[derive(serde::Deserialize)]
#[serde(untagged)]
enum TomlComment {
    One(String),
    Many(Vec<String>),
}
#[derive(serde::Deserialize)]
#[serde(deny_unknown_fields)]
struct TomlMarker {
    name: Option<String>,
    number: Option<String>,
    #[serde(rename = "type")]
    type_: Option<String>,
}
#[derive(serde::Deserialize)]
#[serde(deny_unknown_fields)]
struct TomlReceiver {
    number: Option<String>,
    #[serde(rename = "type")]
    type_: Option<String>,
    version: Option<String>,
}
#[derive(serde::Deserialize)]
#[serde(deny_unknown_fields)]
struct TomlAntenna {
    number: Option<String>,
    #[serde(rename = "type")]
    type_: Option<String>,
}

fn read_header_file(text: &str) -> Result<Metadata, String> {
    let tm: TomlMeta = toml::from_str(text).map_err(|e| e.to_string())?;
    let mut meta = Metadata {
        version: tm.version.unwrap_or_default(),
        comment: match tm.comment {
            Some(TomlComment::One(s)) => s.lines().map(str::to_string).collect(),
            Some(TomlComment::Many(v)) => v,
            None => Vec::new(),
        },
        observer: tm.observer.unwrap_or_default(),
        agency: tm.agency.unwrap_or_default(),
        approx_position: tm.approx_position,
        antenna_delta: tm.antenna_delta,
        interval: tm.interval,
        leap_seconds: tm.leap_seconds,
        ..Default::default()
    };
    if let Some(run) = tm.run {
        meta.run = MetadataRun {
            program: run.program.unwrap_or_default(),
            by: run.by.unwrap_or_default(),
            date: run.date.and_then(toml_datetime_to_instant),
        };
    }
    if let Some(mk) = tm.marker {
        meta.marker = Marker {
            name: mk.name.unwrap_or_default(),
            number: mk.number.unwrap_or_default(),
            type_: mk.type_.unwrap_or_default(),
        };
    }
    if let Some(rx) = tm.receiver {
        meta.receiver = Receiver {
            number: rx.number.unwrap_or_default(),
            type_: rx.type_.unwrap_or_default(),
            version: rx.version.unwrap_or_default(),
        };
    }
    if let Some(an) = tm.antenna {
        meta.antenna = Antenna {
            number: an.number.unwrap_or_default(),
            type_: an.type_.unwrap_or_default(),
        };
    }
    Ok(meta)
}

fn toml_datetime_to_instant(d: toml::value::Datetime) -> Option<Instant> {
    let date = d.date?;
    let t = d.time.unwrap_or(toml::value::Time {
        hour: 0,
        minute: 0,
        second: 0,
        nanosecond: 0,
    });
    Some(Instant::from_civil(Civil {
        year: date.year as i64,
        month: date.month as u32,
        day: date.day as u32,
        hour: t.hour as u32,
        minute: t.minute as u32,
        second: t.second as u32,
        nanos: t.nanosecond,
    }))
}

// ---------------------------------------------------------------------------
// Output / input plumbing
// ---------------------------------------------------------------------------

fn open_writer(path: Option<&str>) -> Result<Box<dyn Write>, String> {
    match path {
        None => Ok(Box::new(io::stdout())),
        Some(p) => {
            let f = File::create(p).map_err(|e| format!("{}: {}", p, e))?;
            Ok(Box::new(f))
        }
    }
}

fn build_sink(cfg: &Config, writer: Box<dyn Write>) -> Result<Box<dyn Sink>, Error> {
    let bw: Box<dyn Write> = Box::new(BufWriter::with_capacity(256 * 1024, writer));
    let mut sink: Box<dyn Sink> = match cfg.to {
        OutputFormat::Rinex => rinex_sink(cfg.rinex_backend.unwrap_or(RinexBackend::Diy), bw)?,
        OutputFormat::ObsJson => Box::new(ObsJsonSink::new(bw)),
    };
    if cfg.interval_ns != 0 {
        let ticks = decimation_interval_ticks(cfg.interval_ns)?;
        sink = Box::new(DecimationSink::new(sink, ticks));
    }
    if cfg.require_cp {
        sink = Box::new(RequireCpFilter::new(sink));
    }
    Ok(sink)
}

fn open_input(path: &str) -> Result<Box<dyn Read>, String> {
    let raw: Box<dyn Read> = if path == "-" {
        Box::new(io::stdin())
    } else {
        Box::new(File::open(path).map_err(|e| input_error(path, &e.to_string()))?)
    };
    maybe_gunzip(raw).map_err(|e| input_error(path, &e.to_string()))
}

/// Transparently decompresses gzip input, detected from the gzip magic bytes in
/// the content (never from the filename). The peeked bytes are chained back so
/// non-gzip input is untouched.
fn maybe_gunzip(mut r: Box<dyn Read>) -> io::Result<Box<dyn Read>> {
    let mut magic = [0u8; 2];
    let mut n = 0;
    while n < magic.len() {
        let got = r.read(&mut magic[n..])?;
        if got == 0 {
            break;
        }
        n += got;
    }
    let head = std::io::Cursor::new(magic[..n].to_vec());
    if n == 2 && magic == [0x1f, 0x8b] {
        Ok(Box::new(flate2::read::GzDecoder::new(head.chain(r))))
    } else {
        Ok(Box::new(head.chain(r)))
    }
}

fn input_error(path: &str, err: &str) -> String {
    if path.is_empty() {
        err.to_string()
    } else if path == "-" {
        format!("stdin: {}", err)
    } else {
        format!("{}: {}", path, err)
    }
}

fn now_instant() -> Instant {
    match SystemTime::now().duration_since(SystemTime::UNIX_EPOCH) {
        Ok(d) => Instant {
            secs: d.as_secs() as i64,
            nanos: d.subsec_nanos(),
        },
        Err(_) => Instant { secs: 0, nanos: 0 },
    }
}

// ---------------------------------------------------------------------------
// Observation-file inputs (rinex / obsj)
// ---------------------------------------------------------------------------

fn convert_observation_inputs(cfg: &Config, writer: Box<dyn Write>) -> Result<(), Error> {
    let mut sink = build_sink(cfg, writer)?;
    for path in &cfg.inputs {
        let br: Box<dyn BufRead> = Box::new(BufReader::new(open_input(path)?));
        let ctx = |e: obsj::Error| Error::conversion(input_error(path, &e.to_string()));
        match cfg.from {
            InputFormat::Rinex => {
                // RINEX parsing is whole-file; stream its observations onward.
                let (backend, br) = open_rinex_input(cfg.rinex_backend, br).map_err(ctx)?;
                let (m, obs) = read_rinex(backend, br).map_err(ctx)?;
                sink.metadata(&m).map_err(|e| e.to_string())?;
                for o in &obs {
                    sink.observation(o).map_err(|e| e.to_string())?;
                }
            }
            // obsj streams record by record — O(1) memory.
            InputFormat::ObsJson => stream_obsj(br, &mut sink).map_err(ctx)?,
            _ => unreachable!(),
        }
    }
    // Command-line / header-file metadata is applied last so it takes precedence
    // when the sink merges (RINEX) or a reader re-merges (obsj).
    sink.metadata(&cfg.meta).map_err(|e| e.to_string())?;
    sink.flush().map_err(|e| e.to_string())?;
    Ok(())
}

// ---------------------------------------------------------------------------
// Packet inputs (rtcm / ubx / raw)
// ---------------------------------------------------------------------------

struct WeekConstraint {
    interval: TimeInterval,
    errf: &'static str,
    err: Option<String>,
}

fn convert_packet_inputs(cfg: &Config, writer: Box<dyn Write>, now: Instant) -> Result<(), Error> {
    let sink = build_sink(cfg, writer)?;
    // Converters emit a per-observation loss-of-lock flag; the accumulator turns
    // it into `arc`. It sits upstream of decimation so a slip in a dropped gap
    // still surfaces on the next kept epoch.
    let sink: Box<dyn Sink> = Box::new(LossOfLockSink::new(sink));
    let mut driver = PacketDriver::new(cfg.from, cfg.rtcm_opts, cfg.ubx_opts, sink);

    if !cfg.meta.is_zero() {
        driver.sink_metadata(&cfg.meta)?;
    }

    let mut total: u64 = 0;
    for path in &cfg.inputs {
        let r = open_input(path)?;
        let c = if cfg.packet_log {
            driver
                .convert_packet_log(r)
                .map_err(|e| input_error(path, &e))?
        } else {
            let modtime = file_modtime(path);
            let wc = file_week_constraint(cfg, path, modtime, now)?;
            driver
                .convert_packet_stream(r, &wc)
                .map_err(|e| input_error(path, &e))?
        };
        total += c;
    }
    if total == 0 {
        return Err(Error::conversion(no_observation_msg(cfg.from)));
    }
    driver.flush()?;
    Ok(())
}

fn no_observation_msg(from: InputFormat) -> String {
    match from {
        InputFormat::Ubx => "no UBX-RXM-RAWX messages found".to_string(),
        InputFormat::Rtcm => "no RTCM MSM7 messages found".to_string(),
        InputFormat::Raw => {
            "no raw observation packets found (UBX RAWX, RTCM MSM7, UNCB OBSVM, UNCA OBSVMA)"
                .to_string()
        }
        _ => "no observations found".to_string(),
    }
}

fn file_modtime(path: &str) -> Option<Instant> {
    if path == "-" {
        return None;
    }
    let meta = std::fs::metadata(path).ok()?;
    let d = meta
        .modified()
        .ok()?
        .duration_since(SystemTime::UNIX_EPOCH)
        .ok()?;
    Some(Instant {
        secs: d.as_secs() as i64,
        nanos: d.subsec_nanos(),
    })
}

fn file_week_constraint(
    cfg: &Config,
    path: &str,
    modtime: Option<Instant>,
    now: Instant,
) -> Result<WeekConstraint, String> {
    let wc = match cfg.week_mode {
        WeekMode::Recent => recent_week_constraint(now),
        WeekMode::Date => date_week_constraint(cfg.week_date, "--date"),
        WeekMode::Filename => {
            let t = date_from_filename(path)?;
            date_week_constraint(t, "--date-from-filename")
        }
        WeekMode::Auto => automatic_week_constraint(now, modtime),
    };
    // strict for plain RTCM input (not raw)
    if cfg.from == InputFormat::Rtcm {
        if let Some(e) = &wc.err {
            return Err(e.clone());
        }
    }
    Ok(wc)
}

fn recent_week_constraint(now: Instant) -> WeekConstraint {
    let start = Instant {
        secs: now.secs - WEEK_SECS,
        nanos: now.nanos,
    };
    WeekConstraint {
        interval: TimeInterval {
            start_ns: start.gps_nanos(),
            dur_ns: WEEK_SECS * SECOND_NS,
        },
        errf: "RTCM epoch does not match recent week inference: ",
        err: None,
    }
}

fn date_week_constraint(date: Instant, opt: &'static str) -> WeekConstraint {
    let start = Instant {
        secs: date.secs - 14 * 3600,
        nanos: date.nanos,
    };
    WeekConstraint {
        interval: TimeInterval {
            start_ns: start.gps_nanos(),
            dur_ns: 50 * 3600 * SECOND_NS,
        },
        errf: if opt == "--date" {
            "RTCM epoch does not match --date: "
        } else {
            "RTCM epoch does not match --date-from-filename: "
        },
        err: None,
    }
}

fn automatic_week_constraint(now: Instant, modtime: Option<Instant>) -> WeekConstraint {
    let mut wc = recent_week_constraint(now);
    let start_secs = now.secs - WEEK_SECS;
    if let Some(mt) = modtime {
        if mt.secs < start_secs {
            wc.err = Some(
                "RTCM input is older than one week; provide --date, --recent, or --date-from-filename"
                    .to_string(),
            );
        }
    }
    wc
}

fn packet_log_week_constraint(t: Instant) -> TimeInterval {
    let start = Instant {
        secs: t.secs - 60,
        nanos: t.nanos,
    };
    TimeInterval {
        start_ns: start.gps_nanos(),
        dur_ns: 2 * 60 * SECOND_NS,
    }
}

fn date_from_filename(path: &str) -> Result<Instant, String> {
    let base = std::path::Path::new(path)
        .file_name()
        .and_then(|s| s.to_str())
        .unwrap_or(path);
    let mut dates: Vec<(i64, u32, u32, Instant)> = Vec::new();
    for run in digit_runs(base) {
        if run.len() >= 8 {
            let y: i64 = run[0..4].parse().unwrap_or(-1);
            let mo: u32 = run[4..6].parse().unwrap_or(0);
            let d: u32 = run[6..8].parse().unwrap_or(0);
            if valid_date(y, mo, d) && !dates.iter().any(|e| e.0 == y && e.1 == mo && e.2 == d) {
                dates.push((y, mo, d, date_instant(y, mo, d)));
            }
        }
    }
    match dates.len() {
        0 => Err(format!("no date found in filename {:?}", path)),
        1 => Ok(dates[0].3),
        _ => Err(format!(
            "multiple conflicting dates found in filename {:?}",
            path
        )),
    }
}

fn digit_runs(s: &str) -> Vec<&str> {
    let b = s.as_bytes();
    let mut runs = Vec::new();
    let mut i = 0;
    while i < b.len() {
        if !b[i].is_ascii_digit() {
            i += 1;
            continue;
        }
        let j0 = i;
        while i < b.len() && b[i].is_ascii_digit() {
            i += 1;
        }
        runs.push(&s[j0..i]);
    }
    runs
}

// ---- the driver ----

#[derive(Clone, Copy)]
enum FamilyKind {
    Rtcm,
    Ubx,
}

/// The active converter. For `raw` it starts `Pending` (holding the sink) and is
/// resolved to a single family by the first packet seen; once resolved it does
/// not switch (mixed UBX/RTCM raw streams are out of scope).
enum Family {
    Pending(Option<Box<dyn Sink>>),
    Rtcm(rtcm::Converter<Box<dyn Sink>>),
    Ubx(ubx::Converter<Box<dyn Sink>>),
}

struct PacketDriver {
    from: InputFormat,
    family: Family,
    rtcm_opts: rtcm::Options,
    ubx_opts: ubx::Options,
    scratch: Vec<u8>,
}

impl PacketDriver {
    fn new(
        from: InputFormat,
        rtcm_opts: rtcm::Options,
        ubx_opts: ubx::Options,
        sink: Box<dyn Sink>,
    ) -> Self {
        let family = match from {
            InputFormat::Ubx => Family::Ubx(ubx::Converter::new(sink, ubx_opts)),
            InputFormat::Rtcm => Family::Rtcm(rtcm::Converter::new(sink, rtcm_opts)),
            _ => Family::Pending(Some(sink)),
        };
        PacketDriver {
            from,
            family,
            rtcm_opts,
            ubx_opts,
            scratch: Vec::with_capacity(4096),
        }
    }

    /// Takes the pending sink, if the family is still unresolved.
    fn take_pending(&mut self) -> Option<Box<dyn Sink>> {
        match &mut self.family {
            Family::Pending(sink) => sink.take(),
            _ => None,
        }
    }

    /// The RTCM converter, resolving a pending sink to the RTCM family on first
    /// use. Returns `None` if the stream is already locked to UBX.
    fn use_rtcm(&mut self) -> Option<&mut rtcm::Converter<Box<dyn Sink>>> {
        if let Some(sink) = self.take_pending() {
            self.family = Family::Rtcm(rtcm::Converter::new(sink, self.rtcm_opts));
        }
        match &mut self.family {
            Family::Rtcm(c) => Some(c),
            _ => None,
        }
    }

    /// The UBX converter, resolving a pending sink to the UBX family on first
    /// use. Returns `None` if the stream is already locked to RTCM.
    fn use_ubx(&mut self) -> Option<&mut ubx::Converter<Box<dyn Sink>>> {
        if let Some(sink) = self.take_pending() {
            self.family = Family::Ubx(ubx::Converter::new(sink, self.ubx_opts));
        }
        match &mut self.family {
            Family::Ubx(c) => Some(c),
            _ => None,
        }
    }

    fn sink_metadata(&mut self, m: &Metadata) -> Result<(), String> {
        match &mut self.family {
            Family::Pending(Some(s)) => s.metadata(m).map_err(|e| e.to_string()),
            Family::Pending(None) => Ok(()),
            Family::Rtcm(c) => c.sink_metadata(m).map_err(|e| e.to_string()),
            Family::Ubx(c) => c.sink_metadata(m).map_err(|e| e.to_string()),
        }
    }

    fn flush(&mut self) -> Result<(), String> {
        match &mut self.family {
            Family::Pending(Some(s)) => s.flush().map_err(|e| e.to_string()),
            Family::Pending(None) => Ok(()),
            Family::Rtcm(c) => c.flush().map_err(|e| e.to_string()),
            Family::Ubx(c) => c.flush().map_err(|e| e.to_string()),
        }
    }

    fn tag_matches(&self, tag: &str) -> bool {
        match self.from {
            InputFormat::Rtcm => tag == "RTCM",
            InputFormat::Ubx => tag == "UBX",
            InputFormat::Raw => matches!(tag, "RTCM" | "UBX"),
            _ => false,
        }
    }

    fn convert_packet_log(&mut self, r: Box<dyn Read>) -> Result<u64, String> {
        let mut br = BufReader::with_capacity(256 * 1024, r);
        let mut line = String::new();
        let mut count: u64 = 0;
        let mut line_no = 0usize;
        loop {
            line.clear();
            let n = br.read_line(&mut line).map_err(|e| e.to_string())?;
            if n == 0 {
                break;
            }
            line_no += 1;
            let trimmed = line.trim();
            if trimmed.is_empty() {
                continue;
            }
            let entry =
                Entry::parse(trimmed).map_err(|e| format!("packet log line {}: {}", line_no, e))?;
            if entry.out {
                continue;
            }
            let bin = entry.bin.as_deref().filter(|s| !s.is_empty());
            let ascii = entry.ascii.as_deref().filter(|s| !s.is_empty());
            if bin.is_none() && ascii.is_none() {
                continue;
            }
            let tag = entry.tag_str();
            if !self.tag_matches(tag) {
                continue;
            }
            // Most UBX traffic is not RXM-RAWX; recognise it from the frame
            // header in the hex (class 0x02, id 0x15) and skip the rest before
            // even hex-decoding the payload.
            if tag == "UBX" {
                if let Some(hex) = bin {
                    let n = UBX_RAWX_HEX_PREFIX.len();
                    if !(hex.len() >= n
                        && hex.as_bytes()[..n].eq_ignore_ascii_case(UBX_RAWX_HEX_PREFIX))
                    {
                        continue;
                    }
                }
            }

            let week = if tag == "RTCM" {
                let t = entry
                    .t
                    .as_deref()
                    .and_then(obsj::json::parse_rfc3339_public)
                    .ok_or_else(|| {
                        format!(
                            "packet log line {}: RTCM packet log line {} has no timestamp",
                            line_no, line_no
                        )
                    })?;
                packet_log_week_constraint(t)
            } else {
                TimeInterval::default()
            };

            let produced = if let Some(hex) = bin {
                hex_decode(hex, &mut self.scratch)
                    .map_err(|e| format!("packet log line {}: {}", line_no, e))?;
                let scratch = std::mem::take(&mut self.scratch);
                let res = self.convert_payload(tag, &scratch, week);
                self.scratch = scratch;
                res.map_err(|e| format!("packet log line {}: {}", line_no, e))?
            } else {
                let bytes = ascii.unwrap().as_bytes().to_vec();
                self.convert_payload(tag, &bytes, week)
                    .map_err(|e| format!("packet log line {}: {}", line_no, e))?
            };
            if produced {
                count += 1;
            }
        }
        Ok(count)
    }

    fn convert_packet_stream(
        &mut self,
        r: Box<dyn Read>,
        wc: &WeekConstraint,
    ) -> Result<u64, String> {
        let mut data = Vec::new();
        BufReader::new(r)
            .read_to_end(&mut data)
            .map_err(|e| e.to_string())?;
        // `raw` picks a single family from whichever valid frame comes first.
        let kind = match self.from {
            InputFormat::Ubx => Some(FamilyKind::Ubx),
            InputFormat::Rtcm => Some(FamilyKind::Rtcm),
            InputFormat::Raw => detect_raw_family(&data),
            _ => None,
        };
        match kind {
            Some(FamilyKind::Ubx) => match self.use_ubx() {
                Some(c) => c.convert_chunk(&data).map_err(|e| e.to_string()),
                None => Ok(0),
            },
            Some(FamilyKind::Rtcm) => self.convert_rtcm_stream(&data, wc),
            None => Ok(0),
        }
    }

    fn convert_rtcm_stream(&mut self, data: &[u8], wc: &WeekConstraint) -> Result<u64, String> {
        let c = match self.use_rtcm() {
            Some(c) => c,
            None => return Ok(0),
        };
        // The first frame carries the week constraint; the rest resolve by
        // continuity.
        let mut count = 0;
        let mut week_used = false;
        for frame in rtcm::frames(data) {
            let is7 = rtcm::is_msm7_frame(frame);
            let (interval, wrap) = if !week_used {
                week_used = true;
                (wc.interval, true)
            } else {
                (TimeInterval::default(), false)
            };
            c.convert_frame(frame, interval).map_err(|e| {
                if wrap {
                    format!("{}{}", wc.errf, e)
                } else {
                    e.to_string()
                }
            })?;
            if is7 {
                count += 1;
            }
        }
        Ok(count)
    }

    /// Converts the frames inside one packet-log payload. `week` (with its errf
    /// wrap baked in by the caller) constrains RTCM epoch resolution.
    fn convert_payload(
        &mut self,
        tag: &str,
        payload: &[u8],
        week: TimeInterval,
    ) -> Result<bool, String> {
        let mut produced = false;
        match tag {
            "RTCM" => {
                if let Some(c) = self.use_rtcm() {
                    // A packet-log RTCM payload is a single framed message;
                    // convert it directly instead of re-scanning and re-checking
                    // the CRC via frames().
                    produced = rtcm::is_msm7_frame(payload);
                    c.convert_frame(payload, week).map_err(|e| {
                        format!("RTCM epoch does not match packet-log timestamp: {}", e)
                    })?;
                }
            }
            "UBX"
                // Each packet-log payload is one UBX frame; most are not
                // RXM-RAWX, so skip those without a full decode.
                if ubx::is_rawx_frame(payload) => {
                    if let Some(c) = self.use_ubx() {
                        if c.convert_chunk(payload).map_err(|e| e.to_string())? > 0 {
                            produced = true;
                        }
                    }
                }
            _ => {}
        }
        Ok(produced)
    }
}

/// Picks the family for a `raw` stream from whichever valid frame appears first.
fn detect_raw_family(data: &[u8]) -> Option<FamilyKind> {
    match (rtcm::first_frame_pos(data), ubx::first_frame_pos(data)) {
        (Some(r), Some(u)) => Some(if r <= u {
            FamilyKind::Rtcm
        } else {
            FamilyKind::Ubx
        }),
        (Some(_), None) => Some(FamilyKind::Rtcm),
        (None, Some(_)) => Some(FamilyKind::Ubx),
        (None, None) => None,
    }
}

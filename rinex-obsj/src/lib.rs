//! Bridge between the obsj observation model and the [`rinex`] crate.
//!
//! The crate owns the on-disk format; this module translates both ways between
//! its epoch-keyed model and obsj's flat one, going through obsj's centralized
//! loss-of-lock transform (`arc → ll` on write, `ll → arc` on read). The whole
//! mapping is the [`RinexObsj`] extension trait on [`Rinex`] plus a couple of
//! free helpers, so it can be contributed upstream as the `rinex` crate's own
//! `obsj` feature with a near-mechanical change — it touches public APIs only.
//!
//! Output is validated semantically (diffobs at 5e-4), so the crate's formatting
//! choices need not match any particular byte layout. The one thing the crate
//! cannot represent — a blank carrier phase carrying only a loss-of-lock flag —
//! is dropped on write and is what `diffobs --ignore-blank-phase` covers.

use obsj::arc::{ArcToLl, LossOfLockSink};
use obsj::obs::*;
use obsj::sink::Sink;
use rinex::observation::{EpochFlag, LliFlags, ObsKey, SignalObservation as XSig};
use rinex::prelude::{
    Constellation, Duration, Epoch, Header, Observable, Rinex, TimeScale, Version, SV,
};
use rinex::record::Record;
use std::collections::{BTreeMap, HashMap};
use std::io::{self, BufReader, BufWriter, Read, Write};
use std::str::FromStr;

/// Maps a [`Rinex`] observation record to and from the obsj model. Shaped as an
/// extension trait over public `rinex` APIs so it is upstreamable as-is.
pub trait RinexObsj {
    /// Builds a RINEX observation record from obsj metadata and observations.
    fn from_obsj(meta: &Metadata, obs: &[SignalObservation]) -> Result<Rinex, String>;
    /// Extracts the obsj model (metadata + observations) from this record.
    fn to_obsj(&self) -> (Metadata, Vec<SignalObservation>);
}

impl RinexObsj for Rinex {
    fn from_obsj(meta: &Metadata, obs: &[SignalObservation]) -> Result<Rinex, String> {
        build_rinex(meta, obs)
    }

    fn to_obsj(&self) -> (Metadata, Vec<SignalObservation>) {
        (
            metadata_from_header(&self.header),
            observations_from_record(self),
        )
    }
}

// ---------------------------------------------------------------------------
// Writing
// ---------------------------------------------------------------------------

/// Buffers observations and writes a RINEX file on flush.
pub struct RinexSink<W: Write> {
    writer: W,
    meta: Metadata,
    obs: Vec<SignalObservation>,
}

impl<W: Write> RinexSink<W> {
    pub fn new(writer: W) -> Self {
        RinexSink {
            writer,
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
        let rinex = Rinex::from_obsj(&self.meta, &self.obs)
            .map_err(|e| io::Error::new(io::ErrorKind::InvalidData, e))?;
        let mut bw = BufWriter::new(&mut self.writer);
        rinex
            .format(&mut bw)
            .map_err(|e| io::Error::other(e.to_string()))?;
        bw.flush()
    }
}

/// Converts our GPS-scale time label to a `rinex` epoch on the GPS time scale.
fn to_epoch(t: GpsTime) -> Epoch {
    let c = t.civil();
    Epoch::from_gregorian(
        c.year as i32,
        c.month as u8,
        c.day as u8,
        c.hour as u8,
        c.minute as u8,
        c.second as u8,
        c.nanos,
        TimeScale::GPST,
    )
}

fn to_epoch_label(e: Epoch) -> GpsTime {
    let (year, month, day, hour, minute, second, nanos) = e.to_gregorian(TimeScale::GPST);
    GpsTime::from_civil(Civil {
        year: year as i64,
        month: month as u32,
        day: day as u32,
        hour: hour as u32,
        minute: minute as u32,
        second: second as u32,
        nanos,
    })
}

fn observable(typ: u8, sig: SigId) -> Observable {
    let code = [typ, sig.0[0], sig.0[1]];
    Observable::from_str(std::str::from_utf8(&code).unwrap()).expect("valid observation code")
}

fn build_rinex(meta: &Metadata, obs: &[SignalObservation]) -> Result<Rinex, String> {
    if obs.is_empty() {
        return Err("no observations".to_string());
    }

    let mut sorted = obs.to_vec();
    sorted.sort_by_key(|o| o.t.0);
    let first = sorted.first().unwrap().t;
    let last = sorted.last().unwrap().t;

    let mut arc = ArcToLl::new();
    let mut record: Record = Record::ObsRecord(BTreeMap::new());
    let signals_record = record.as_mut_obs().unwrap();
    let mut codes: HashMap<Constellation, Vec<Observable>> = HashMap::new();
    let mut code_seen: HashMap<Constellation, std::collections::HashSet<Observable>> =
        HashMap::new();
    let mut glo_channels: HashMap<SV, i8> = HashMap::new();

    for o in &sorted {
        let sv =
            SV::from_str(o.sat.as_str()).map_err(|_| format!("invalid satellite {}", o.sat))?;
        let constellation = sv.constellation;
        let changed = arc.lli(
            SignalKey {
                sat: o.sat,
                sig: o.sig,
            },
            o.v.arc,
        );
        let lli_bits = o.v.rinex_lli(changed);

        if let Some(frq) = o.v.frq {
            glo_channels.insert(sv, frq);
        }

        let entry = signals_record
            .entry(ObsKey {
                epoch: to_epoch(o.t),
                flag: EpochFlag::Ok,
            })
            .or_default();

        let mut push = |typ: u8, value: f64, lli: Option<LliFlags>| {
            let obs_code = observable(typ, o.sig);
            if code_seen
                .entry(constellation)
                .or_default()
                .insert(obs_code.clone())
            {
                codes
                    .entry(constellation)
                    .or_default()
                    .push(obs_code.clone());
            }
            entry.signals.push(XSig {
                sv,
                observable: obs_code,
                value,
                lli,
                snr: None,
            });
        };

        let lli = (lli_bits != 0).then(|| LliFlags::from_bits_truncate(lli_bits));
        if let Some(pr) = o.v.pr {
            push(TYPE_CODE, pr, None);
        }
        // The `rinex` crate's observation value is a mandatory f64, so a blank
        // carrier phase that exists only to carry a loss-of-lock indicator (a
        // pseudorange-only signal that lost lock) cannot be represented and is
        // dropped — this is the documented limitation `--ignore-blank-phase`
        // skips when validating this backend against convbin/Go goldens.
        if let Some(cp) = o.v.cp {
            push(TYPE_PHASE, cp, lli);
        }
        if let Some(dop) = o.v.dop {
            push(TYPE_DOPPLER, dop, None);
        }
        if let Some(cn0) = o.v.cn0 {
            push(TYPE_SIGNAL_STRENGTH, cn0 as f64, None);
        }
    }

    let mut header = Header::basic_obs();
    header.version = parse_version(&meta.version);
    header.program = non_empty(&meta.run.program);
    header.run_by = non_empty(&meta.run.by);
    header.date = meta.run.date.map(format_header_date);
    header.observer = non_empty(&meta.observer);
    header.agency = non_empty(&meta.agency);
    header.geodetic_marker = build_marker(meta);
    header.rcvr = build_receiver(meta);
    header.rcvr_antenna = build_antenna(meta);
    header.glo_channels = glo_channels;
    // convbin, the Go reference, and the DIY backend all emit an APPROX POSITION
    // line, zero-filled when unknown; match that so semantically-equal output
    // does not read back as a spurious metadata difference.
    let pos = meta.approx_position.unwrap_or([0.0, 0.0, 0.0]);
    header.rx_position = Some((pos[0], pos[1], pos[2]));
    if let Some(ls) = meta.leap_seconds {
        header.leap = Some(rinex::prelude::Leap {
            leap: ls as u32,
            ..Default::default()
        });
    }
    header.sampling_interval = meta.interval.map(Duration::from_seconds);

    if let Some(obs_header) = header.obs.as_mut() {
        obs_header.codes = codes;
        obs_header.timeof_first_obs = Some(to_epoch(first));
        obs_header.timeof_last_obs = Some(to_epoch(last));
    }

    Ok(Rinex {
        header,
        record,
        comments: Default::default(),
        production: Default::default(),
    })
}

fn non_empty(s: &str) -> Option<String> {
    (!s.is_empty()).then(|| s.to_string())
}

fn parse_version(s: &str) -> Version {
    let mut parts = s.split('.');
    let major = parts.next().and_then(|p| p.parse().ok()).unwrap_or(3);
    let minor = parts.next().and_then(|p| p.parse().ok()).unwrap_or(4);
    Version { major, minor }
}

fn format_header_date(d: Instant) -> String {
    let c = d.civil();
    format!(
        "{:04}{:02}{:02} {:02}{:02}{:02} UTC",
        c.year, c.month, c.day, c.hour, c.minute, c.second
    )
}

fn build_marker(meta: &Metadata) -> Option<rinex::marker::GeodeticMarker> {
    if meta.marker.is_zero() {
        return None;
    }
    let mut marker = rinex::marker::GeodeticMarker::default();
    if !meta.marker.name.is_empty() {
        marker = marker.with_name(&meta.marker.name);
    }
    if !meta.marker.number.is_empty() {
        marker = marker.with_number(&meta.marker.number);
    }
    Some(marker)
}

fn build_receiver(meta: &Metadata) -> Option<rinex::hardware::Receiver> {
    if meta.receiver.is_zero() {
        return None;
    }
    Some(
        rinex::hardware::Receiver::default()
            .with_serial_number(&meta.receiver.number)
            .with_model(&meta.receiver.type_)
            .with_firmware(&meta.receiver.version),
    )
}

fn build_antenna(meta: &Metadata) -> Option<rinex::hardware::Antenna> {
    if meta.antenna.is_zero() && meta.antenna_delta.is_none() {
        return None;
    }
    let mut antenna = rinex::hardware::Antenna::default()
        .with_model(&meta.antenna.type_)
        .with_serial_number(&meta.antenna.number);
    if let Some(d) = meta.antenna_delta {
        antenna = antenna
            .with_height(d[0])
            .with_eastern_component(d[1])
            .with_northern_component(d[2]);
    }
    Some(antenna)
}

// ---------------------------------------------------------------------------
// Reading
// ---------------------------------------------------------------------------

/// Reads a RINEX observation file into the obsj model (convenience over
/// [`Rinex::parse`] + [`RinexObsj::to_obsj`]).
pub fn read_observation_file<R: Read>(r: R) -> Result<(Metadata, Vec<SignalObservation>), String> {
    let mut reader = BufReader::new(r);
    let rinex = Rinex::parse(&mut reader).map_err(|e| e.to_string())?;
    Ok(rinex.to_obsj())
}

fn metadata_from_header(header: &Header) -> Metadata {
    let mut meta = Metadata {
        version: format!("{}.{:02}", header.version.major, header.version.minor),
        run: MetadataRun {
            program: header.program.clone().unwrap_or_default(),
            by: header.run_by.clone().unwrap_or_default(),
            date: None,
        },
        observer: header.observer.clone().unwrap_or_default(),
        agency: header.agency.clone().unwrap_or_default(),
        approx_position: header.rx_position.map(|(x, y, z)| [x, y, z]),
        interval: header.sampling_interval.map(|d| d.to_seconds()),
        leap_seconds: header.leap.as_ref().map(|l| l.leap as i16),
        ..Default::default()
    };
    if let Some(marker) = &header.geodetic_marker {
        meta.marker = Marker {
            name: marker.name.clone(),
            number: marker.number().unwrap_or_default(),
            ..Default::default()
        };
    }
    if let Some(rx) = &header.rcvr {
        meta.receiver = Receiver {
            number: rx.sn.clone(),
            type_: rx.model.clone(),
            version: rx.firmware.clone(),
        };
    }
    if let Some(ant) = &header.rcvr_antenna {
        meta.antenna = Antenna {
            number: ant.sn.clone(),
            type_: ant.model.clone(),
        };
    }
    meta
}

fn observations_from_record(rinex: &Rinex) -> Vec<SignalObservation> {
    let mut out = Vec::new();
    let record = match rinex.record.as_obs() {
        Some(r) => r,
        None => return out,
    };

    for (key, observations) in record {
        let t = to_epoch_label(key.epoch);
        // Group the epoch's signals by (sat, sig), preserving first-seen order.
        let mut order: Vec<SignalKey> = Vec::new();
        let mut by_key: HashMap<SignalKey, SignalValues> = HashMap::new();

        for sig in &observations.signals {
            let (sat, sig_id) = match (satid(sig.sv), signal_id(&sig.observable)) {
                (Some(sat), Some(sig_id)) => (sat, sig_id),
                _ => continue,
            };
            let k = SignalKey { sat, sig: sig_id };
            let values = by_key.entry(k).or_insert_with(|| {
                order.push(k);
                SignalValues::default()
            });

            match observable_kind(&sig.observable) {
                ObsKind::Code => values.pr = Some(sig.value),
                ObsKind::Phase => {
                    values.cp = Some(sig.value);
                    // Emit the per-observation loss-of-lock flag; the accumulator
                    // below turns it into `arc`, as the converters do.
                    if let Some(lli) = sig.lli {
                        values.set_lli(lli.bits());
                    }
                }
                ObsKind::Doppler => values.dop = Some(sig.value),
                ObsKind::SignalStrength => values.cn0 = Some(sig.value as f32),
                ObsKind::Other => {}
            }
        }

        for k in order {
            out.push(SignalObservation {
                t,
                sat: k.sat,
                sig: k.sig,
                v: by_key[&k],
            });
        }
    }
    accumulate_arc(out)
}

/// Collecting sink, used to drive [`LossOfLockSink`] over a buffered read.
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

/// Turns the per-observation `ll` flag (read from RINEX LLI) into `arc`, using
/// the same accumulator the converters feed.
fn accumulate_arc(obs: Vec<SignalObservation>) -> Vec<SignalObservation> {
    let mut acc = LossOfLockSink::new(ObsCollector::default());
    for o in &obs {
        let _ = acc.observation(o);
    }
    acc.into_inner().obs
}

enum ObsKind {
    Code,
    Phase,
    Doppler,
    SignalStrength,
    Other,
}

fn observable_kind(o: &Observable) -> ObsKind {
    if o.is_pseudo_range_observable() {
        ObsKind::Code
    } else if o.is_phase_range_observable() {
        ObsKind::Phase
    } else if o.is_doppler_observable() {
        ObsKind::Doppler
    } else if o.is_ssi_observable() {
        ObsKind::SignalStrength
    } else {
        ObsKind::Other
    }
}

fn satid(sv: SV) -> Option<SatId> {
    sv.to_string().parse().ok()
}

fn signal_id(o: &Observable) -> Option<SigId> {
    o.code()?.parse().ok()
}

//! The relay's replies, byte for byte, against the C++ relay's.
//!
//! `testdata/relay_corpus.json` holds a fixed corpus of requests and what
//! the C++ relay sent back to each, recorded from it by
//! `tools/record_golden.py` under three configurations: looping, not
//! looping, and `drop_mask` 0x808. This test serves the same synthetic
//! source from the in-process relay under each configuration, sends it the
//! same requests over UDP on loopback, and compares every reply: the number
//! of datagrams, the length of each and a SHA-256 over them in order, and,
//! for the first requests of each configuration, the datagrams themselves.
//!
//! The corpus is the parity reference now that the C++ relay is gone: it is
//! never re-recorded from this relay. A difference here is a change in what
//! the board would receive.

use std::error::Error;
use std::io;
use std::mem::offset_of;
use std::net::{SocketAddr, UdpSocket};
use std::sync::Arc;
use std::time::Duration;

use argus_sim_core::server::{Config, ReplayServer};
use argus_sim_core::source::ReplaySource;
use argus_sim_core::wire::{CHUNK_HEADER_LEN, decode_request};
use argus_wire::generated::argus_replay_chunk_hdr_t;
use serde_json::Value;
use sha2::{Digest, Sha256};

type TestResult<T = ()> = Result<T, Box<dyn Error>>;

/// The recording. Compiled in, so the test finds it wherever it runs from.
const CORPUS: &str = include_str!("../../../testdata/relay_corpus.json");

/// How long one expected datagram may take. The relay answers in well under
/// a millisecond on loopback; this only bounds a reply that never comes.
const REPLY_TIMEOUT: Duration = Duration::from_secs(1);

/// The differences reported in full before a configuration gives up.
const MAX_REPORTED: usize = 10;

/// What the C++ relay sent back to one request.
struct Expected {
    /// The datagrams' lengths, in order.
    lengths: Vec<usize>,
    /// SHA-256 over the datagrams, concatenated in order.
    sha256: [u8; 32],
    /// The datagrams themselves, for the first requests of a configuration.
    data: Option<Vec<Vec<u8>>>,
}

/// One relay configuration and its recorded replies, one per request.
struct Recording {
    name: String,
    looping: bool,
    drop_mask: u32,
    replies: Vec<Expected>,
}

/// The parsed corpus.
struct Corpus {
    samples: u64,
    channels: u16,
    quiet: Duration,
    requests: Vec<Vec<u8>>,
    configs: Vec<Recording>,
}

/// `s` as bytes; the corpus writes every byte string in lowercase hex.
fn unhex(s: &str) -> TestResult<Vec<u8>> {
    if !s.len().is_multiple_of(2) {
        return Err(format!("odd-length hex: {s}").into());
    }
    (0..s.len())
        .step_by(2)
        .map(|at| {
            let pair = s.get(at..at + 2).ok_or("hex")?;
            Ok(u8::from_str_radix(pair, 16)?)
        })
        .collect()
}

fn field<'a>(value: &'a Value, key: &str) -> TestResult<&'a Value> {
    value.get(key).ok_or_else(|| format!("no `{key}`").into())
}

fn string<'a>(value: &'a Value, key: &str) -> TestResult<&'a str> {
    field(value, key)?
        .as_str()
        .ok_or_else(|| format!("`{key}` is not a string").into())
}

fn number(value: &Value, key: &str) -> TestResult<u64> {
    field(value, key)?
        .as_u64()
        .ok_or_else(|| format!("`{key}` is not a number").into())
}

fn array<'a>(value: &'a Value, key: &str) -> TestResult<&'a [Value]> {
    field(value, key)?
        .as_array()
        .map(Vec::as_slice)
        .ok_or_else(|| format!("`{key}` is not an array").into())
}

/// One recorded reply. `len` holds the lengths as `[length, repeat]` runs.
fn expected(value: &Value) -> TestResult<Expected> {
    let mut lengths = Vec::new();
    for run in array(value, "len")? {
        let pair = run.as_array().ok_or("a length run is not an array")?;
        let [length, repeat] = pair.as_slice() else {
            return Err("a length run is not a pair".into());
        };
        let length = usize::try_from(length.as_u64().ok_or("length")?)?;
        let repeat = usize::try_from(repeat.as_u64().ok_or("repeat")?)?;
        lengths.extend(std::iter::repeat_n(length, repeat));
    }
    if u64::try_from(lengths.len())? != number(value, "n")? {
        return Err("`n` disagrees with `len`".into());
    }
    let sha256 = unhex(string(value, "sha256")?)?
        .try_into()
        .map_err(|_| "a SHA-256 is not 32 bytes")?;
    let data = match value.get("data") {
        None => None,
        Some(data) => Some(
            data.as_array()
                .ok_or("`data` is not an array")?
                .iter()
                .map(|d| unhex(d.as_str().ok_or("a datagram is not a string")?))
                .collect::<TestResult<_>>()?,
        ),
    };
    Ok(Expected {
        lengths,
        sha256,
        data,
    })
}

/// One configuration. Its parameters are the node's, as the recorder passed
/// them on the command line.
fn recording(value: &Value) -> TestResult<Recording> {
    let params = field(value, "params")?;
    let looping = match string(params, "loop")? {
        "true" => true,
        "false" => false,
        other => return Err(format!("loop:={other}").into()),
    };
    Ok(Recording {
        name: string(value, "name")?.to_owned(),
        looping,
        drop_mask: string(params, "drop_mask")?.parse()?,
        replies: array(value, "replies")?
            .iter()
            .map(expected)
            .collect::<TestResult<_>>()?,
    })
}

fn corpus() -> TestResult<Corpus> {
    let doc: Value = serde_json::from_str(CORPUS)?;
    let source = field(&doc, "source")?;
    let corpus = Corpus {
        samples: number(source, "synthetic_samples")?,
        channels: u16::try_from(number(source, "channels")?)?,
        quiet: Duration::from_millis(number(&doc, "quiet_ms")?),
        requests: array(&doc, "requests")?
            .iter()
            .map(|r| unhex(r.as_str().ok_or("a request is not a string")?))
            .collect::<TestResult<_>>()?,
        configs: array(&doc, "configs")?
            .iter()
            .map(recording)
            .collect::<TestResult<_>>()?,
    };
    for config in &corpus.configs {
        if config.replies.len() != corpus.requests.len() {
            return Err(format!("{}: one reply per request", config.name).into());
        }
    }
    Ok(corpus)
}

/// The header field at byte `at` of a chunk, to say where two differ.
fn chunk_field(at: usize) -> &'static str {
    type Hdr = argus_replay_chunk_hdr_t;
    // Last field first, so the first one starting at or before `at` is it.
    const FIELDS: [(usize, &str); 11] = [
        (offset_of!(Hdr, crc), "crc"),
        (offset_of!(Hdr, payload_len), "payload_len"),
        (offset_of!(Hdr, channel_count), "channel_count"),
        (offset_of!(Hdr, chunk_total), "chunk_total"),
        (offset_of!(Hdr, chunk_index), "chunk_index"),
        (offset_of!(Hdr, sample_count), "sample_count"),
        (offset_of!(Hdr, sample_offset), "sample_offset"),
        (offset_of!(Hdr, seq), "seq"),
        (offset_of!(Hdr, type_), "type"),
        (offset_of!(Hdr, version), "version"),
        (offset_of!(Hdr, magic), "magic"),
    ];
    if at >= CHUNK_HEADER_LEN {
        return "payload";
    }
    FIELDS
        .iter()
        .find(|(start, _)| at >= *start)
        .map_or("header", |(_, name)| name)
}

/// The request as the relay reads it, for a difference report.
fn describe(request: &[u8]) -> String {
    match decode_request(request) {
        Ok(r) => format!(
            "seq={} offset={} count={} flags={:#06x}",
            { r.seq },
            { r.sample_offset },
            { r.sample_count },
            { r.flags }
        ),
        Err(reject) => format!("{} bytes, rejected: {reject:?}", request.len()),
    }
}

/// The datagrams `expect` says came back, as far as they came within
/// [`REPLY_TIMEOUT`] each.
fn receive(socket: &UdpSocket, expect: usize) -> TestResult<Vec<Vec<u8>>> {
    let mut rx = [0u8; 2048];
    let mut replies = Vec::with_capacity(expect);
    socket.set_read_timeout(Some(REPLY_TIMEOUT))?;
    while replies.len() < expect {
        match socket.recv(&mut rx) {
            Ok(len) => replies.push(rx.get(..len).ok_or("datagram length")?.to_vec()),
            Err(e)
                if matches!(
                    e.kind(),
                    io::ErrorKind::WouldBlock | io::ErrorKind::TimedOut
                ) =>
            {
                break;
            }
            Err(e) => return Err(e.into()),
        }
    }
    Ok(replies)
}

/// How `got` differs from `want`, or `None` when it is the same.
fn difference(got: &[Vec<u8>], want: &Expected) -> Option<String> {
    let lengths: Vec<usize> = got.iter().map(Vec::len).collect();
    if lengths != want.lengths {
        return Some(format!(
            "{} datagrams of {lengths:?}, want {} of {:?}",
            got.len(),
            want.lengths.len(),
            want.lengths
        ));
    }
    if let Some(data) = &want.data {
        for (k, (g, w)) in got.iter().zip(data).enumerate() {
            if let Some(at) = g.iter().zip(w).position(|(a, b)| a != b) {
                return Some(format!(
                    "datagram {k} differs from byte {at} ({})",
                    chunk_field(at)
                ));
            }
        }
    }
    let mut sha = Sha256::new();
    for g in got {
        sha.update(g);
    }
    if sha.finalize().as_slice() != want.sha256.as_slice() {
        return Some("same lengths, different bytes (SHA-256)".to_owned());
    }
    None
}

/// Replays the corpus against a relay configured as `config`; returns the
/// differences, an empty list when every reply matched.
fn replay(corpus: &Corpus, config: &Recording) -> TestResult<Vec<String>> {
    let source = Arc::new(ReplaySource::synthetic(corpus.samples, corpus.channels)?);
    let mut server = ReplayServer::new(
        source,
        Config {
            bind_addr: "127.0.0.1".to_owned(),
            any_port: true,
            looping: config.looping,
            drop_mask: config.drop_mask,
            ..Config::default()
        },
    )?;
    server.start()?;
    let relay: SocketAddr = server.local_addr().ok_or("the relay is not running")?;
    let socket = UdpSocket::bind("127.0.0.1:0")?;

    // The relay answers in arrival order, and the recorder waited for quiet
    // after each request, so the datagrams for request i all arrive before
    // any for request i + 1. Receiving exactly the expected number per
    // request keeps that alignment: a datagram too many shows up as a
    // difference at the next request, and one after the last request is
    // caught by the final wait below.
    let mut diffs = Vec::new();
    for (i, (request, want)) in corpus.requests.iter().zip(&config.replies).enumerate() {
        socket.send_to(request, relay)?;
        let got = receive(&socket, want.lengths.len())?;
        if let Some(diff) = difference(&got, want) {
            diffs.push(format!(
                "{}: request {i} ({}): {diff}",
                config.name,
                describe(request)
            ));
            if diffs.len() >= MAX_REPORTED {
                return Ok(diffs);
            }
        }
    }
    socket.set_read_timeout(Some(corpus.quiet))?;
    let mut rx = [0u8; 2048];
    if let Ok(len) = socket.recv(&mut rx) {
        diffs.push(format!(
            "{}: a {len}-byte datagram after the last reply",
            config.name
        ));
    }
    server.stop();
    Ok(diffs)
}

#[test]
fn the_corpus_covers_every_recorded_configuration() -> TestResult {
    let corpus = corpus()?;
    let names: Vec<&str> = corpus.configs.iter().map(|c| c.name.as_str()).collect();
    assert_eq!(names, ["loop", "no_loop", "drop_0x808"]);
    assert_eq!((corpus.samples, corpus.channels), (30_000, 96));
    // The recorder's corpus: the fixed cases and 300 random requests.
    assert!(
        corpus.requests.len() > 300,
        "{} requests",
        corpus.requests.len()
    );
    for config in &corpus.configs {
        let whole = config
            .replies
            .iter()
            .take_while(|r| r.data.is_some())
            .count();
        assert_eq!(whole, 20, "{}: replies kept whole", config.name);
    }
    Ok(())
}

#[test]
fn every_reply_matches_the_cpp_relay() -> TestResult {
    let corpus = corpus()?;
    let mut diffs = Vec::new();
    for config in &corpus.configs {
        diffs.extend(replay(&corpus, config)?);
    }
    assert!(
        diffs.is_empty(),
        "{} difference(s) from the C++ relay:\n{}",
        diffs.len(),
        diffs.join("\n")
    );
    Ok(())
}

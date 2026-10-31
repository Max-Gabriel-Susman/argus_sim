//! The replay protocol for Python: the CRC, requests and chunks.
//!
//! Each function here is a thin wrapper over `argus_sim_core::wire`, which
//! is itself built on argus-wire's structs and CRC. What this module adds is
//! the Python side: arguments checked into wire fields, the decoded fields
//! as small immutable classes, and every refusal as a [`WireError`] that
//! says which check failed.

use argus_sim_core::wire::{self, ChunkError, ChunkFields, ChunkReject, Reject};
use pyo3::create_exception;
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use pyo3::pybacked::PyBackedBytes;
use pyo3::types::PyBytes;

use crate::args::Unsigned;

create_exception!(
    argus_sim_py,
    WireError,
    PyValueError,
    "A datagram or field the replay protocol does not allow.\n\n\
     `.kind` names the check that failed: \"size\", \"magic\", \"version\", \
     \"type\", \"crc\", \"no_samples\" or \"payload\". A ValueError, so code \
     that catches bad values catches this too."
);

/// A [`WireError`] of the given kind. The kind is set as an attribute on
/// the exception instance, since `create_exception!` makes a plain subclass
/// with no fields of its own.
fn wire_error(py: Python<'_>, kind: &str, what: &str, why: impl std::fmt::Display) -> PyErr {
    let err = WireError::new_err(format!("{what}: {why}"));
    match err.value(py).setattr("kind", kind) {
        Ok(()) => err,
        // Setting an attribute fails only when Python is out of memory;
        // that error is the one to raise then.
        Err(failed) => failed,
    }
}

/// The `.kind` of a refused request.
fn request_kind(reject: Reject) -> &'static str {
    match reject {
        Reject::Size => "size",
        Reject::Magic => "magic",
        Reject::Version => "version",
        Reject::Type => "type",
        Reject::Crc => "crc",
        Reject::NoSamples => "no_samples",
    }
}

/// The `.kind` of a refused chunk.
fn chunk_kind(reject: ChunkReject) -> &'static str {
    match reject {
        ChunkReject::Size => "size",
        ChunkReject::Magic => "magic",
        ChunkReject::Version => "version",
        ChunkReject::Type => "type",
        ChunkReject::Payload => "payload",
        ChunkReject::Crc => "crc",
    }
}

/// A replay request, as the board sends it: `sample_count` samples from
/// `sample_offset`, under the sequence number `seq`.
#[pyclass(frozen, get_all, eq, hash, module = "argus_sim_py")]
#[derive(Clone, PartialEq, Eq, Hash)]
pub struct Request {
    /// The sequence number every chunk of the reply carries back.
    seq: u32,
    /// The index in the dataset of the first sample asked for.
    sample_offset: u32,
    /// How many samples are asked for.
    sample_count: u16,
    /// F_RETRANSMIT and F_LOOP, or'd together.
    flags: u16,
    /// The CRC-16/CCITT-FALSE over the 20 bytes before it.
    crc: u16,
}

#[pymethods]
impl Request {
    fn __repr__(&self) -> String {
        format!(
            "Request(seq={}, sample_offset={}, sample_count={}, flags={:#06x}, crc={:#06x})",
            self.seq, self.sample_offset, self.sample_count, self.flags, self.crc
        )
    }
}

/// One chunk of a reply: a header and `sample_count` rows of samples.
#[pyclass(frozen, eq, hash, module = "argus_sim_py")]
#[derive(Clone, PartialEq, Eq, Hash)]
pub struct Chunk {
    /// The `seq` of the request this chunk answers.
    #[pyo3(get)]
    seq: u32,
    /// The index in the dataset of the chunk's first sample, after any wrap.
    #[pyo3(get)]
    sample_offset: u32,
    /// The samples (rows) in this chunk.
    #[pyo3(get)]
    sample_count: u16,
    /// This chunk's place in the reply, from 0.
    #[pyo3(get)]
    chunk_index: u16,
    /// The chunks in the reply.
    #[pyo3(get)]
    chunk_total: u16,
    /// The channels in a row.
    #[pyo3(get)]
    channel_count: u16,
    /// The CRC-16/CCITT-FALSE over header bytes [0, 26) and the payload.
    #[pyo3(get)]
    crc: u16,
    /// The rows: `sample_count` x `channel_count` little-endian u16 samples.
    payload: Vec<u8>,
}

#[pymethods]
impl Chunk {
    /// The length of the payload in bytes, as the header gives it.
    #[getter]
    fn payload_len(&self) -> usize {
        self.payload.len()
    }

    /// The rows: `sample_count` x `channel_count` little-endian u16 samples.
    #[getter]
    fn payload<'py>(&self, py: Python<'py>) -> Bound<'py, PyBytes> {
        PyBytes::new(py, &self.payload)
    }

    fn __repr__(&self) -> String {
        format!(
            "Chunk(seq={}, sample_offset={}, sample_count={}, chunk_index={}, \
             chunk_total={}, channel_count={}, payload_len={}, crc={:#06x})",
            self.seq,
            self.sample_offset,
            self.sample_count,
            self.chunk_index,
            self.chunk_total,
            self.channel_count,
            self.payload.len(),
            self.crc
        )
    }
}

/// The CRC-16/CCITT-FALSE of `data` (bytes or bytearray), the checksum on
/// every Argus datagram.
///
/// `crc` is the register to start from: 0xFFFF for a new checksum, or the
/// result of an earlier call to go on from it, so that
/// `crc16(b, crc16(a)) == crc16(a + b)`.
#[pyfunction]
#[pyo3(signature = (data, crc = Unsigned(0xFFFF)), text_signature = "(data, crc=0xFFFF)")]
pub fn crc16(data: PyBackedBytes, crc: Unsigned<u16>) -> u16 {
    wire::crc16_from(crc.0, &data)
}

/// The 22 bytes of a replay request for `sample_count` samples from
/// `sample_offset`, its CRC sealed, exactly as the board sends it.
///
/// `flags` is F_RETRANSMIT and F_LOOP, or'd together. A field out of range
/// raises ValueError. A `sample_count` of 0 is encoded: the relay refuses it,
/// and a test may want to send one.
#[pyfunction]
#[pyo3(
    signature = (seq, sample_offset, sample_count, flags = Unsigned(0)),
    text_signature = "(seq, sample_offset, sample_count, flags=0)"
)]
pub fn encode_request<'py>(
    py: Python<'py>,
    seq: Unsigned<u32>,
    sample_offset: Unsigned<u32>,
    sample_count: Unsigned<u16>,
    flags: Unsigned<u16>,
) -> Bound<'py, PyBytes> {
    let request = wire::encode_request(seq.0, sample_offset.0, sample_count.0, flags.0);
    PyBytes::new(py, &request)
}

/// The request in `data`, checked as the relay checks it: the length, the
/// magic, version and type, the CRC, then a nonzero sample count.
///
/// Raises WireError on the first check that fails, with `.kind` "size",
/// "magic", "version", "type", "crc" or "no_samples".
#[pyfunction]
pub fn decode_request(py: Python<'_>, data: PyBackedBytes) -> PyResult<Request> {
    let req = wire::decode_request(&data)
        .map_err(|reject| wire_error(py, request_kind(reject), "replay request", reject))?;
    Ok(Request {
        seq: req.seq,
        sample_offset: req.sample_offset,
        sample_count: req.sample_count,
        flags: req.flags,
        crc: req.crc,
    })
}

/// A replay chunk carrying `payload` (bytes or bytearray), its header filled
/// in and its CRC sealed, exactly as the relay sends it.
///
/// The payload must be whole rows of `channel_count` little-endian u16
/// samples, at most MAX_PAYLOAD bytes; `sample_count` and `payload_len` are
/// worked out from it. A payload that is not raises WireError with `.kind`
/// "payload"; a `channel_count` of 0 or a field out of range, ValueError.
#[pyfunction]
pub fn encode_chunk<'py>(
    py: Python<'py>,
    seq: Unsigned<u32>,
    sample_offset: Unsigned<u32>,
    chunk_index: Unsigned<u16>,
    chunk_total: Unsigned<u16>,
    channel_count: Unsigned<u16>,
    payload: PyBackedBytes,
) -> PyResult<Bound<'py, PyBytes>> {
    if channel_count.0 == 0 {
        return Err(PyValueError::new_err("channel_count must be positive"));
    }
    let payload_error =
        |why: &dyn std::fmt::Display| wire_error(py, "payload", "replay chunk", why);
    if payload.len() > wire::MAX_PAYLOAD {
        return Err(payload_error(&ChunkError::PayloadTooLong));
    }
    let row_bytes = usize::from(channel_count.0) * 2;
    if !payload.len().is_multiple_of(row_bytes) {
        return Err(payload_error(&format_args!(
            "{} payload bytes are not whole rows of {} channels",
            payload.len(),
            channel_count.0
        )));
    }
    // At most MAX_PAYLOAD / 2 rows, checked above, which fits a u16.
    let sample_count = u16::try_from(payload.len() / row_bytes)
        .map_err(|_| payload_error(&ChunkError::PayloadTooLong))?;

    let fields = ChunkFields {
        seq: seq.0,
        sample_offset: sample_offset.0,
        sample_count,
        chunk_index: chunk_index.0,
        chunk_total: chunk_total.0,
        channel_count: channel_count.0,
    };
    let mut packet = vec![0u8; wire::CHUNK_HEADER_LEN];
    packet.extend_from_slice(&payload);
    let len =
        wire::seal_chunk(&mut packet, &fields, payload.len()).map_err(|err| payload_error(&err))?;
    packet.truncate(len);
    Ok(PyBytes::new(py, &packet))
}

/// The chunk in `data`, checked on its own: the length, the magic, version
/// and type, the payload against the header, then the CRC.
///
/// Raises WireError on the first check that fails, with `.kind` "size",
/// "magic", "version", "type", "payload" or "crc". Whether the chunk belongs
/// to the fetch in progress (its seq, its index) is the caller's to check.
#[pyfunction]
pub fn decode_chunk(py: Python<'_>, data: PyBackedBytes) -> PyResult<Chunk> {
    let (hdr, payload) = wire::decode_chunk(&data)
        .map_err(|reject| wire_error(py, chunk_kind(reject), "replay chunk", reject))?;
    Ok(Chunk {
        seq: hdr.seq,
        sample_offset: hdr.sample_offset,
        sample_count: hdr.sample_count,
        chunk_index: hdr.chunk_index,
        chunk_total: hdr.chunk_total,
        channel_count: hdr.channel_count,
        crc: hdr.crc,
        payload: payload.to_vec(),
    })
}

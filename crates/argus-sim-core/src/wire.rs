//! Replay requests and chunks as the relay reads and writes them.
//!
//! The structs, the constants and the CRC are `argus-wire`'s, shared with the
//! firmware; this module checks a received request against them and builds
//! the chunks of a reply. Nothing in it redefines the wire format.
//!
//! A request is one 22-byte datagram from the board: "send me
//! `sample_count` samples from `sample_offset`". The reply is a burst of
//! chunks, each a 28-byte header followed by whole rows of samples. Both end
//! in a CRC-16/CCITT-FALSE: a request's covers the bytes before its `crc`
//! field; a chunk's covers its header up to the `crc` field and then the
//! payload after it.

use core::mem::{offset_of, size_of};
use std::fmt;

use argus_wire::crc::Crc16;
use argus_wire::generated::{
    ARGUS_FRAME_MAGIC, ARGUS_MAX_CHANNELS, ARGUS_MSG_REPLAY_CHUNK, ARGUS_MSG_REPLAY_REQUEST,
    ARGUS_REPLAY_F_LOOP, ARGUS_REPLAY_F_RETRANSMIT, ARGUS_REPLAY_MAX_PAYLOAD, ARGUS_REPLAY_PORT,
    ARGUS_REPLAY_SAMPLES_PER_HALF, ARGUS_REPLAY_VERSION, argus_replay_chunk_hdr_t,
    argus_replay_request_t,
};
use zerocopy::{FromBytes, IntoBytes};

/// bindgen gives every `#define` the type `u32`, but the fields they go in
/// are mostly `u16`. Narrowing here, in a `const`, means a constant that
/// ever outgrew its field would stop the build instead of wrapping.
#[expect(
    clippy::cast_possible_truncation,
    reason = "the assert above the cast rules the truncation out, at compile time"
)]
const fn narrow(value: u32) -> u16 {
    assert!(value <= u16::MAX as u32, "constant does not fit a u16");
    value as u16
}

/// `ARGUS_FRAME_MAGIC`, "ARGS" read as a little-endian `u32`: the first
/// field of every Argus datagram.
pub const MAGIC: u32 = ARGUS_FRAME_MAGIC;
/// `ARGUS_REPLAY_VERSION`, the protocol version in requests and chunks.
pub const VERSION: u16 = narrow(ARGUS_REPLAY_VERSION);
/// `ARGUS_MSG_REPLAY_REQUEST`, the `type` of a request.
pub const MSG_REQUEST: u16 = narrow(ARGUS_MSG_REPLAY_REQUEST);
/// `ARGUS_MSG_REPLAY_CHUNK`, the `type` of a chunk.
pub const MSG_CHUNK: u16 = narrow(ARGUS_MSG_REPLAY_CHUNK);
/// `ARGUS_REPLAY_F_RETRANSMIT`: the request repeats one that went
/// unanswered. Fault injection never drops a chunk of its reply.
pub const F_RETRANSMIT: u16 = narrow(ARGUS_REPLAY_F_RETRANSMIT);
/// `ARGUS_REPLAY_F_LOOP`: wrap at the end of the dataset, even when the relay
/// is not configured to loop.
pub const F_LOOP: u16 = narrow(ARGUS_REPLAY_F_LOOP);
/// `ARGUS_REPLAY_PORT`, the UDP port the board sends its requests to.
pub const REPLAY_PORT: u16 = narrow(ARGUS_REPLAY_PORT);
/// `ARGUS_REPLAY_SAMPLES_PER_HALF`: the samples in half of the board's
/// buffer, which is what it asks for at a time, and the relay's default cap
/// on one request.
pub const SAMPLES_PER_HALF: u16 = narrow(ARGUS_REPLAY_SAMPLES_PER_HALF);
/// `ARGUS_MAX_CHANNELS`: the channels in the board's telemetry frame. A
/// relay serving another count still works, but the frame will not match.
pub const MAX_CHANNELS: u16 = narrow(ARGUS_MAX_CHANNELS);
/// `ARGUS_REPLAY_MAX_PAYLOAD`: the most payload bytes one chunk carries, so
/// that header and payload fit one 1500-byte Ethernet MTU with the IP and
/// UDP headers.
pub const MAX_PAYLOAD: usize = narrow(ARGUS_REPLAY_MAX_PAYLOAD) as usize;

/// The length of a request: a datagram of any other length is not one.
pub const REQUEST_LEN: usize = size_of::<argus_replay_request_t>();
/// The bytes of a request the CRC covers: everything before the `crc` field.
pub const REQUEST_CRC_SPAN: usize = offset_of!(argus_replay_request_t, crc);
/// The length of a chunk header; the payload starts here.
pub const CHUNK_HEADER_LEN: usize = size_of::<argus_replay_chunk_hdr_t>();
/// The bytes of a chunk header the CRC covers before it goes on to the
/// payload: everything before the `crc` field.
pub const CHUNK_CRC_SPAN: usize = offset_of!(argus_replay_chunk_hdr_t, crc);
/// The longest chunk: a header and a full payload. The relay's transmit
/// buffer is this long.
pub const MAX_CHUNK_LEN: usize = CHUNK_HEADER_LEN + MAX_PAYLOAD;

// The lengths the board, the C header and the Python tools all assume. They
// come from the structs above, so these fail the build if argus-wire's
// layout ever moves.
const _: () = assert!(REQUEST_LEN == 22);
const _: () = assert!(REQUEST_CRC_SPAN == 20);
const _: () = assert!(CHUNK_HEADER_LEN == 28);
const _: () = assert!(CHUNK_CRC_SPAN == 26);
// The CRC is the last field of a request, so it covers all the rest.
const _: () = assert!(REQUEST_CRC_SPAN + size_of::<u16>() == REQUEST_LEN);

/// Why a datagram was not taken as a request.
///
/// The variants are in the order the relay checks them, which is the order
/// the C++ relay checked them in: when several apply, the first is the one
/// reported. Every one counts as a malformed request and gets no reply.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Reject {
    /// The datagram is not [`REQUEST_LEN`] bytes long.
    Size,
    /// The first field is not [`MAGIC`].
    Magic,
    /// The `version` is not [`VERSION`].
    Version,
    /// The `type` is not [`MSG_REQUEST`].
    Type,
    /// The CRC does not match the bytes before it.
    Crc,
    /// The request asks for no samples.
    NoSamples,
}

impl fmt::Display for Reject {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Self::Size => "not the length of a replay request",
            Self::Magic => "bad magic",
            Self::Version => "unsupported replay version",
            Self::Type => "not a replay request",
            Self::Crc => "CRC mismatch",
            Self::NoSamples => "asks for no samples",
        })
    }
}

impl std::error::Error for Reject {}

/// Checks a received datagram and returns the request in it.
///
/// The checks follow `ReplayServer::handle` in the C++ relay, in its order:
/// the length, then magic, version and type, then the CRC over the first
/// [`REQUEST_CRC_SPAN`] bytes, then a nonzero `sample_count`.
///
/// # Errors
///
/// The first check that fails, as a [`Reject`].
pub fn decode_request(datagram: &[u8]) -> Result<argus_replay_request_t, Reject> {
    // read_from_bytes takes exactly size_of::<argus_replay_request_t>()
    // bytes, so this is the length check.
    let req = argus_replay_request_t::read_from_bytes(datagram).map_err(|_| Reject::Size)?;

    if req.magic != MAGIC {
        return Err(Reject::Magic);
    }
    if req.version != VERSION {
        return Err(Reject::Version);
    }
    if req.type_ != MSG_REQUEST {
        return Err(Reject::Type);
    }
    let covered = datagram.get(..REQUEST_CRC_SPAN).ok_or(Reject::Size)?;
    if Crc16::new().update(covered).finish() != req.crc {
        return Err(Reject::Crc);
    }
    if req.sample_count == 0 {
        return Err(Reject::NoSamples);
    }
    Ok(req)
}

/// A request for `sample_count` samples from `sample_offset`, with its CRC:
/// the 22 bytes the board's client sends.
#[must_use]
pub fn encode_request(
    seq: u32,
    sample_offset: u32,
    sample_count: u16,
    flags: u16,
) -> [u8; REQUEST_LEN] {
    let mut req = argus_replay_request_t {
        magic: MAGIC,
        version: VERSION,
        type_: MSG_REQUEST,
        seq,
        sample_offset,
        sample_count,
        flags,
        crc: 0,
    };
    let covered = req.as_bytes().get(..REQUEST_CRC_SPAN).unwrap_or_default();
    req.crc = Crc16::new().update(covered).finish();
    // Same size on both sides, which transmute! checks at compile time.
    zerocopy::transmute!(req)
}

/// The fields of a chunk header that vary from chunk to chunk. Magic,
/// version and type are constants, and [`seal_chunk`] works out the payload
/// length and the CRC.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ChunkFields {
    /// The `seq` of the request this chunk answers.
    pub seq: u32,
    /// The index in the dataset of the chunk's first sample, after any wrap.
    pub sample_offset: u32,
    /// The samples (rows) in this chunk.
    pub sample_count: u16,
    /// This chunk's place in the burst, from 0.
    pub chunk_index: u16,
    /// The chunks in the burst, so the board can tell when one is missing.
    pub chunk_total: u16,
    /// The channels in a row. The board checks it rather than assume it.
    pub channel_count: u16,
}

/// Why a chunk could not be built.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ChunkError {
    /// The payload is longer than [`MAX_PAYLOAD`].
    PayloadTooLong,
    /// The packet buffer cannot hold the header and the payload.
    BufferTooShort,
}

impl fmt::Display for ChunkError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::PayloadTooLong => write!(f, "payload longer than {MAX_PAYLOAD} bytes"),
            Self::BufferTooShort => f.write_str("buffer too short for the chunk"),
        }
    }
}

impl std::error::Error for ChunkError {}

/// Finishes a chunk whose payload is already in place.
///
/// `packet` holds the payload's `payload_len` bytes from [`CHUNK_HEADER_LEN`]
/// on, where the relay reads the rows straight from its source. This writes
/// the header in front of them: the constants, `fields`, `payload_len`, and
/// the CRC over header bytes [0, [`CHUNK_CRC_SPAN`]) followed by the payload.
/// It returns the length of the datagram to send.
///
/// # Errors
///
/// [`ChunkError::PayloadTooLong`] when `payload_len` is over
/// [`MAX_PAYLOAD`], and [`ChunkError::BufferTooShort`] when `packet` is
/// shorter than the header and the payload.
pub fn seal_chunk(
    packet: &mut [u8],
    fields: &ChunkFields,
    payload_len: usize,
) -> Result<usize, ChunkError> {
    if payload_len > MAX_PAYLOAD {
        return Err(ChunkError::PayloadTooLong);
    }
    let len = CHUNK_HEADER_LEN + payload_len;
    let (head, rest) = packet
        .get_mut(..len)
        .ok_or(ChunkError::BufferTooShort)?
        .split_at_mut(CHUNK_HEADER_LEN);
    let payload: &[u8] = rest;

    let mut hdr = argus_replay_chunk_hdr_t {
        magic: MAGIC,
        version: VERSION,
        type_: MSG_CHUNK,
        seq: fields.seq,
        sample_offset: fields.sample_offset,
        sample_count: fields.sample_count,
        chunk_index: fields.chunk_index,
        chunk_total: fields.chunk_total,
        channel_count: fields.channel_count,
        payload_len: u16::try_from(payload_len).map_err(|_| ChunkError::PayloadTooLong)?,
        crc: 0,
    };
    // The CRC covers the header up to its own field, then the payload, which
    // is not contiguous with it: Crc16 resumes across the two pieces.
    let covered = hdr.as_bytes().get(..CHUNK_CRC_SPAN).unwrap_or_default();
    hdr.crc = Crc16::new().update(covered).update(payload).finish();
    hdr.write_to(head).map_err(|_| ChunkError::BufferTooShort)?;
    Ok(len)
}

/// Why a datagram was not taken as a chunk.
///
/// The relay never reads chunks; this is for the tools and the bindings that
/// look at its replies. The variants are in the order [`decode_chunk`] checks
/// them. The board's client checks more than this (the `seq`, the channel
/// count and the chunk's place in the fetch it is waiting for); these are only
/// the checks a chunk can fail on its own.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ChunkReject {
    /// The datagram is shorter than a chunk header.
    Size,
    /// The first field is not [`MAGIC`].
    Magic,
    /// The `version` is not [`VERSION`].
    Version,
    /// The `type` is not [`MSG_CHUNK`].
    Type,
    /// The payload does not agree with the header: `payload_len` is over
    /// [`MAX_PAYLOAD`], is not the length of the bytes after the header, or
    /// is not `sample_count` rows of `channel_count` samples.
    Payload,
    /// The CRC does not match the header and the payload.
    Crc,
}

impl fmt::Display for ChunkReject {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Self::Size => "shorter than a replay chunk header",
            Self::Magic => "bad magic",
            Self::Version => "unsupported replay version",
            Self::Type => "not a replay chunk",
            Self::Payload => "payload does not match the header",
            Self::Crc => "CRC mismatch",
        })
    }
}

impl std::error::Error for ChunkReject {}

/// Checks a received datagram and returns the chunk header and payload in it.
///
/// The length, then magic, version and type, then the payload against
/// `payload_len`, `sample_count` and `channel_count`, then the CRC over
/// header bytes [0, [`CHUNK_CRC_SPAN`]) followed by the payload. The payload
/// is checked before the CRC because its length says what the CRC covers.
///
/// # Errors
///
/// The first check that fails, as a [`ChunkReject`].
pub fn decode_chunk(datagram: &[u8]) -> Result<(argus_replay_chunk_hdr_t, &[u8]), ChunkReject> {
    let (hdr, payload) =
        argus_replay_chunk_hdr_t::read_from_prefix(datagram).map_err(|_| ChunkReject::Size)?;

    if hdr.magic != MAGIC {
        return Err(ChunkReject::Magic);
    }
    if hdr.version != VERSION {
        return Err(ChunkReject::Version);
    }
    if hdr.type_ != MSG_CHUNK {
        return Err(ChunkReject::Type);
    }
    let payload_len = usize::from(hdr.payload_len);
    // Widened before multiplying: 65535 rows of 65535 channels overflows a
    // u16 many times over, and must not wrap into a match.
    let rows_len = u32::from(hdr.sample_count) * u32::from(hdr.channel_count) * 2;
    if payload_len > MAX_PAYLOAD
        || payload.len() != payload_len
        || u32::from(hdr.payload_len) != rows_len
    {
        return Err(ChunkReject::Payload);
    }
    let covered = datagram.get(..CHUNK_CRC_SPAN).ok_or(ChunkReject::Size)?;
    if Crc16::new().update(covered).update(payload).finish() != hdr.crc {
        return Err(ChunkReject::Crc);
    }
    Ok((hdr, payload))
}

/// The CRC-16/CCITT-FALSE of `data`, starting from the register value
/// `register` instead of the usual 0xFFFF.
///
/// This is how a checksum over pieces is carried from one call to the next
/// by a caller that only has the number, not a [`Crc16`]: the CRC of `a`
/// then `b` is `crc16_from(crc16_from(0xFFFF, a), b)`.
///
/// argus-wire's [`Crc16`] starts at 0xFFFF and has no way to be set to
/// another value, on purpose, and the CRC is not written a second time
/// here. Instead, this finds the two bytes that take a fresh register to
/// `register`, and feeds `data` after them. Two bytes always do it, and
/// exactly one pair does: fed to the register, they are XORed into all 16
/// bits of it before 16 shift steps, and each step can be undone (a set low
/// bit after the step means the polynomial was XORed in), so the pairs map
/// one to one onto the 65536 register values. The search costs at most 65536
/// two-byte updates, a fraction of a millisecond, and is skipped when the
/// register is the usual start.
#[must_use]
pub fn crc16_from(register: u16, data: &[u8]) -> u16 {
    let start = Crc16::new();
    let resumed = if start.finish() == register {
        start
    } else {
        (0..=u16::MAX)
            .map(|prefix| start.update(&prefix.to_be_bytes()))
            .find(|crc| crc.finish() == register)
            // Never taken: every register value has its pair, as above, and a
            // test walks all of them.
            .unwrap_or(start)
    };
    resumed.update(data).finish()
}

#[cfg(test)]
mod tests {
    use super::*;

    use argus_difftest::c::CClient;
    use argus_difftest::relay::{self, Relay};

    type TestResult = Result<(), Box<dyn std::error::Error>>;

    /// A valid request with every field distinct, so a field read from the
    /// wrong offset shows.
    fn valid() -> [u8; REQUEST_LEN] {
        encode_request(0x0102_0304, 0x0A0B_0C0D, 147, F_LOOP)
    }

    /// Flips the bits of `mask` in byte `at`.
    fn flip(bytes: &mut [u8], at: usize, mask: u8) -> TestResult {
        *bytes.get_mut(at).ok_or("flip past the end")? ^= mask;
        Ok(())
    }

    /// Recomputes the CRC of a request after a test has changed its bytes,
    /// so the change is the only thing wrong with it.
    fn reseal(req: &mut [u8; REQUEST_LEN]) -> TestResult {
        let (covered, crc) = req.split_at_mut(REQUEST_CRC_SPAN);
        crc.copy_from_slice(&argus_wire::crc::checksum(covered).to_le_bytes());
        Ok(())
    }

    /// Why `decode_request` refused a datagram, if it did. The request
    /// struct has no `PartialEq`, so the tests compare this instead.
    fn rejected(datagram: &[u8]) -> Option<Reject> {
        decode_request(datagram).err()
    }

    /// The `n`th chunk of a packet list.
    fn nth(chunks: &[Vec<u8>], n: usize) -> Result<&[u8], String> {
        chunks
            .get(n)
            .map(Vec::as_slice)
            .ok_or(format!("no chunk {n}"))
    }

    #[test]
    fn a_valid_request_round_trips() -> TestResult {
        let req = decode_request(&valid())?;
        assert_eq!({ req.magic }, MAGIC);
        assert_eq!({ req.version }, 1);
        assert_eq!({ req.type_ }, 0x0010);
        assert_eq!({ req.seq }, 0x0102_0304);
        assert_eq!({ req.sample_offset }, 0x0A0B_0C0D);
        assert_eq!({ req.sample_count }, 147);
        assert_eq!({ req.flags }, F_LOOP);
        assert_eq!(req.as_bytes(), valid().as_slice());
        Ok(())
    }

    #[test]
    fn a_request_is_laid_out_as_argus_wire_h_says() {
        // Written out by hand from argus_wire.h, independent of the structs:
        // packed, little-endian, the CRC last.
        let mut want = Vec::new();
        want.extend_from_slice(b"SGRA"); // 0x41524753
        want.extend_from_slice(&1u16.to_le_bytes());
        want.extend_from_slice(&0x10u16.to_le_bytes());
        want.extend_from_slice(&7u32.to_le_bytes());
        want.extend_from_slice(&1000u32.to_le_bytes());
        want.extend_from_slice(&147u16.to_le_bytes());
        want.extend_from_slice(&3u16.to_le_bytes());
        let crc = argus_wire::crc::checksum(&want);
        want.extend_from_slice(&crc.to_le_bytes());
        assert_eq!(
            encode_request(7, 1000, 147, F_LOOP | F_RETRANSMIT).as_slice(),
            want.as_slice()
        );
    }

    #[test]
    fn rejects_a_wrong_length() {
        let req = valid();
        let (short, _) = req.split_at(REQUEST_LEN - 1);
        assert_eq!(rejected(short), Some(Reject::Size));
        let mut long = req.to_vec();
        long.push(0);
        assert_eq!(rejected(&long), Some(Reject::Size));
        assert_eq!(rejected(&[]), Some(Reject::Size));
    }

    #[test]
    fn rejects_a_wrong_magic_version_or_type() -> TestResult {
        // Each field at its offset in argus_replay_request_t, with the CRC
        // made good again so the field is the only fault.
        for (offset, reject) in [(0, Reject::Magic), (4, Reject::Version), (6, Reject::Type)] {
            let mut req = valid();
            flip(&mut req, offset, 0x01)?;
            reseal(&mut req)?;
            assert_eq!(rejected(&req), Some(reject), "byte {offset}");
        }
        Ok(())
    }

    #[test]
    fn rejects_a_wrong_crc() -> TestResult {
        let mut req = valid();
        flip(&mut req, REQUEST_CRC_SPAN, 0x01)?;
        assert_eq!(rejected(&req), Some(Reject::Crc));
        // A flipped bit in any field the CRC covers after the type.
        for at in 8..REQUEST_CRC_SPAN {
            let mut req = valid();
            flip(&mut req, at, 0x80)?;
            assert_eq!(rejected(&req), Some(Reject::Crc), "byte {at}");
        }
        Ok(())
    }

    #[test]
    fn rejects_a_request_for_no_samples() {
        assert_eq!(
            rejected(&encode_request(1, 0, 0, 0)),
            Some(Reject::NoSamples)
        );
    }

    #[test]
    fn reports_the_first_fault_in_the_cpp_order() -> TestResult {
        // Everything wrong at once. Each step mends the fault just reported,
        // and the next one in the order shows.
        let mut req = encode_request(1, 0, 0, 0);
        flip(&mut req, 0, 0x01)?; // magic
        flip(&mut req, 4, 0x01)?; // version
        flip(&mut req, 6, 0x01)?; // type
        flip(&mut req, REQUEST_CRC_SPAN, 0x01)?; // CRC
        let mut long = req.to_vec();
        long.push(0);
        assert_eq!(rejected(&long), Some(Reject::Size));
        assert_eq!(rejected(&req), Some(Reject::Magic));
        flip(&mut req, 0, 0x01)?;
        assert_eq!(rejected(&req), Some(Reject::Version));
        flip(&mut req, 4, 0x01)?;
        assert_eq!(rejected(&req), Some(Reject::Type));
        flip(&mut req, 6, 0x01)?;
        assert_eq!(rejected(&req), Some(Reject::Crc));
        flip(&mut req, REQUEST_CRC_SPAN, 0x01)?;
        assert_eq!(rejected(&req), Some(Reject::NoSamples));
        Ok(())
    }

    #[test]
    fn a_chunk_carries_its_fields_and_crc() -> TestResult {
        let fields = ChunkFields {
            seq: 0x1122_3344,
            sample_offset: 29_999,
            sample_count: 3,
            chunk_index: 4,
            chunk_total: 21,
            channel_count: 96,
        };
        let payload_len = 3 * 96 * 2;
        let mut packet = [0u8; MAX_CHUNK_LEN];
        let (_, payload) = packet.split_at_mut(CHUNK_HEADER_LEN);
        for (i, byte) in payload.iter_mut().take(payload_len).enumerate() {
            *byte = u8::try_from(i % 251)?;
        }
        let len = seal_chunk(&mut packet, &fields, payload_len)?;
        assert_eq!(len, CHUNK_HEADER_LEN + payload_len);

        let (hdr, rest) =
            argus_replay_chunk_hdr_t::read_from_prefix(&packet).map_err(|_| "no header")?;
        assert_eq!({ hdr.magic }, MAGIC);
        assert_eq!({ hdr.version }, 1);
        assert_eq!({ hdr.type_ }, 0x0011);
        assert_eq!({ hdr.seq }, 0x1122_3344);
        assert_eq!({ hdr.sample_offset }, 29_999);
        assert_eq!({ hdr.sample_count }, 3);
        assert_eq!({ hdr.chunk_index }, 4);
        assert_eq!({ hdr.chunk_total }, 21);
        assert_eq!({ hdr.channel_count }, 96);
        assert_eq!(usize::from(hdr.payload_len), payload_len);

        // The CRC from scratch, over one contiguous copy of what it covers.
        let mut covered = packet.get(..CHUNK_CRC_SPAN).ok_or("short")?.to_vec();
        covered.extend_from_slice(rest.get(..payload_len).ok_or("short")?);
        assert_eq!({ hdr.crc }, argus_wire::crc::checksum(&covered));
        Ok(())
    }

    #[test]
    fn a_chunk_refuses_what_does_not_fit() {
        let fields = ChunkFields {
            seq: 0,
            sample_offset: 0,
            sample_count: 0,
            chunk_index: 0,
            chunk_total: 1,
            channel_count: 96,
        };
        let mut packet = [0u8; MAX_CHUNK_LEN + 2];
        assert_eq!(
            seal_chunk(&mut packet, &fields, MAX_PAYLOAD),
            Ok(MAX_CHUNK_LEN)
        );
        assert_eq!(
            seal_chunk(&mut packet, &fields, MAX_PAYLOAD + 2),
            Err(ChunkError::PayloadTooLong)
        );
        let (short, _) = packet.split_at_mut(CHUNK_HEADER_LEN + 9);
        assert_eq!(
            seal_chunk(short, &fields, 10),
            Err(ChunkError::BufferTooShort)
        );
        let (short, _) = packet.split_at_mut(CHUNK_HEADER_LEN - 1);
        assert_eq!(
            seal_chunk(short, &fields, 0),
            Err(ChunkError::BufferTooShort)
        );
    }

    /// A sealed chunk of `sample_count` rows of `channel_count` samples, its
    /// payload bytes counting up from 0.
    fn sealed(
        sample_count: u16,
        channel_count: u16,
    ) -> Result<Vec<u8>, Box<dyn std::error::Error>> {
        let fields = ChunkFields {
            seq: 0x1122_3344,
            sample_offset: 29_999,
            sample_count,
            chunk_index: 4,
            chunk_total: 21,
            channel_count,
        };
        let payload_len = usize::from(sample_count) * usize::from(channel_count) * 2;
        let mut packet = vec![0u8; CHUNK_HEADER_LEN + payload_len];
        let (_, payload) = packet.split_at_mut(CHUNK_HEADER_LEN);
        for (i, byte) in payload.iter_mut().enumerate() {
            *byte = u8::try_from(i % 251)?;
        }
        let len = seal_chunk(&mut packet, &fields, payload_len)?;
        assert_eq!(len, packet.len());
        Ok(packet)
    }

    /// Why `decode_chunk` refused a datagram, if it did.
    fn chunk_rejected(datagram: &[u8]) -> Option<ChunkReject> {
        decode_chunk(datagram).err()
    }

    #[test]
    fn a_sealed_chunk_decodes_to_its_fields() -> TestResult {
        let packet = sealed(3, 96)?;
        let (hdr, payload) = decode_chunk(&packet)?;
        assert_eq!({ hdr.seq }, 0x1122_3344);
        assert_eq!({ hdr.sample_offset }, 29_999);
        assert_eq!({ hdr.sample_count }, 3);
        assert_eq!({ hdr.chunk_index }, 4);
        assert_eq!({ hdr.chunk_total }, 21);
        assert_eq!({ hdr.channel_count }, 96);
        assert_eq!(usize::from(hdr.payload_len), 3 * 96 * 2);
        assert_eq!(Some(payload), packet.get(CHUNK_HEADER_LEN..));
        // A full chunk and an empty one decode too.
        decode_chunk(&sealed(7, 96)?)?;
        decode_chunk(&sealed(0, 96)?)?;
        Ok(())
    }

    #[test]
    fn a_chunk_is_refused_in_order() -> TestResult {
        let good = sealed(3, 96)?;
        assert_eq!(
            chunk_rejected(good.get(..CHUNK_HEADER_LEN - 1).ok_or("short")?),
            Some(ChunkReject::Size)
        );
        assert_eq!(chunk_rejected(&[]), Some(ChunkReject::Size));
        for (offset, reject) in [
            (0, ChunkReject::Magic),
            (4, ChunkReject::Version),
            (6, ChunkReject::Type),
        ] {
            let mut bad = good.clone();
            flip(&mut bad, offset, 0x01)?;
            assert_eq!(chunk_rejected(&bad), Some(reject), "byte {offset}");
        }
        // A byte missing from the payload, a byte too many, and a header
        // whose rows (sample_count at 16, channel_count at 22) no longer
        // match payload_len.
        let short = good.get(..good.len() - 1).ok_or("short")?;
        assert_eq!(chunk_rejected(short), Some(ChunkReject::Payload));
        let mut long = good.clone();
        long.push(0);
        assert_eq!(chunk_rejected(&long), Some(ChunkReject::Payload));
        for at in [16, 22] {
            let mut bad = good.clone();
            flip(&mut bad, at, 0x01)?;
            assert_eq!(
                chunk_rejected(&bad),
                Some(ChunkReject::Payload),
                "byte {at}"
            );
        }
        // A bit flipped where the CRC looks: the seq, the offset, the chunk
        // numbers, the CRC itself and the payload.
        for at in [
            8,
            12,
            18,
            20,
            CHUNK_CRC_SPAN,
            CHUNK_HEADER_LEN,
            good.len() - 1,
        ] {
            let mut bad = good.clone();
            flip(&mut bad, at, 0x80)?;
            assert_eq!(chunk_rejected(&bad), Some(ChunkReject::Crc), "byte {at}");
        }
        Ok(())
    }

    #[test]
    fn a_chunk_over_the_mtu_is_refused() -> TestResult {
        // 8 rows of 96 channels: one row more than fits. seal_chunk will not
        // build it, so the header is written by hand; every other check
        // would pass.
        let payload_len: u16 = 8 * 96 * 2;
        let mut hdr = argus_replay_chunk_hdr_t {
            magic: MAGIC,
            version: VERSION,
            type_: MSG_CHUNK,
            seq: 1,
            sample_offset: 0,
            sample_count: 8,
            chunk_index: 0,
            chunk_total: 1,
            channel_count: 96,
            payload_len,
            crc: 0,
        };
        let payload = vec![0u8; usize::from(payload_len)];
        let covered = hdr.as_bytes().get(..CHUNK_CRC_SPAN).ok_or("short")?;
        hdr.crc = Crc16::new().update(covered).update(&payload).finish();
        let mut packet = hdr.as_bytes().to_vec();
        packet.extend_from_slice(&payload);
        assert_eq!(chunk_rejected(&packet), Some(ChunkReject::Payload));
        Ok(())
    }

    #[test]
    fn the_crc_resumes_from_any_register() {
        let data = b"123456789";
        for split in 0..=data.len() {
            let (head, tail) = data.split_at(split);
            let resumed = crc16_from(crc16_from(0xFFFF, head), tail);
            assert_eq!(resumed, 0x29B1, "split at {split}");
        }
        assert_eq!(crc16_from(0xFFFF, data), argus_wire::crc::checksum(data));
        for register in [0x0000, 0x0001, 0x29B1, 0x8000, 0xFFFE] {
            assert_eq!(crc16_from(register, &[]), register);
        }
    }

    #[test]
    fn two_bytes_take_a_fresh_register_to_every_value() {
        // What crc16_from's search relies on: the 65536 two-byte prefixes
        // leave 65536 different registers.
        let mut reached = vec![false; 65536];
        for prefix in 0..=u16::MAX {
            let register = Crc16::new().update(&prefix.to_be_bytes()).finish();
            if let Some(seen) = reached.get_mut(usize::from(register)) {
                *seen = true;
            }
        }
        assert!(reached.iter().all(|&seen| seen));
    }

    /// The burst argus-difftest's relay sends for `request`, built here: the
    /// same fields, and payloads of its `relay::value` pattern.
    fn burst(relay: &Relay, request: &[u8]) -> Result<Vec<Vec<u8>>, Box<dyn std::error::Error>> {
        let req = decode_request(request)?;
        let per_chunk = relay.samples_per_chunk;
        let total = req.sample_count.div_ceil(per_chunk);
        let mut burst = Vec::new();
        for index in 0..total {
            let done = index * per_chunk;
            let count = per_chunk.min(req.sample_count - done);
            let first = req.sample_offset + u32::from(done);
            let mut packet = [0u8; MAX_CHUNK_LEN];
            let (_, payload) = packet.split_at_mut(CHUNK_HEADER_LEN);
            let values = (0..u32::from(count))
                .flat_map(|s| (0..relay.channel_count).map(move |c| relay::value(first + s, c)));
            let mut payload_len = 0;
            for (slot, value) in payload.chunks_exact_mut(2).zip(values) {
                slot.copy_from_slice(&value.to_le_bytes());
                payload_len += 2;
            }
            let fields = ChunkFields {
                seq: req.seq,
                sample_offset: first,
                sample_count: count,
                chunk_index: index,
                chunk_total: total,
                channel_count: relay.channel_count,
            };
            let len = seal_chunk(&mut packet, &fields, payload_len)?;
            burst.push(packet.get(..len).ok_or("short")?.to_vec());
        }
        Ok(burst)
    }

    #[test]
    fn chunks_match_argus_difftest_and_the_c_client_takes_them() -> TestResult {
        let relay = Relay::for_channels(96);
        for (offset, count) in [
            (0, 147),
            (1000, 147),
            (29_927, 147),
            (5, 1),
            (70, 8),
            (0, 7),
        ] {
            // The request as the C client in argus_replay_client.h sends it.
            let mut client = CClient::new(96, 200, 5).map_err(|rc| format!("init {rc}"))?;
            assert_eq!(client.fetch(offset, count, 0), 0);
            let snap = client.snapshot();
            let sent = snap
                .sent
                .get(..usize::try_from(snap.sent_len)?)
                .ok_or("sent_len past the snapshot")?;
            let req = decode_request(sent)?;
            assert_eq!(
                ({ req.sample_offset }, { req.sample_count }),
                (offset, count)
            );

            let reference = relay.answer(sent);
            let ours = burst(&relay, sent)?;
            assert_eq!(ours.len(), reference.len(), "offset {offset} count {count}");
            assert!(!ours.is_empty());
            for i in 0..ours.len() {
                assert_eq!(
                    nth(&ours, i)?,
                    nth(&reference, i)?,
                    "offset {offset} chunk {i}"
                );
            }

            for chunk in &ours {
                client.on_packet(chunk);
            }
            let snap = client.snapshot();
            assert_eq!(snap.state, 2, "offset {offset}: the fetch completes");
            assert_eq!(usize::try_from(snap.chunks_accepted)?, ours.len());
            assert_eq!(snap.chunks_rejected, 0);
        }
        Ok(())
    }
}

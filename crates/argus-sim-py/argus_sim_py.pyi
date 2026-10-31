"""The Argus replay relay, for Python.

The board fetches a recorded dataset from the workstation half a buffer at a
time: a 22-byte replay request over UDP, answered with a burst of chunks of
whole sample rows. This module is that protocol and its relay, built from the
same Rust as the dataset_relay_node executable.
"""

from typing import Final, final

__version__: Final[str]
"""The package version, the same as the crate's and package.xml's."""

MAGIC: Final[int]
"""The first field of every Argus datagram: "ARGS" read as a little-endian
32-bit integer."""

VERSION: Final[int]
"""The replay protocol version, in requests and chunks."""

MSG_REQUEST: Final[int]
"""The `type` of a replay request."""

MSG_CHUNK: Final[int]
"""The `type` of a replay chunk."""

F_RETRANSMIT: Final[int]
"""Request flag: the request repeats one that went unanswered. The relay never
withholds a chunk of its reply."""

F_LOOP: Final[int]
"""Request flag: wrap at the end of the dataset, even when the relay is not
configured to loop."""

REPLAY_PORT: Final[int]
"""The UDP port the board sends its requests to."""

SAMPLES_PER_HALF: Final[int]
"""The samples in half of the board's buffer: what it asks for at a time, and
the relay's cap on one request."""

MAX_CHANNELS: Final[int]
"""The channels in the board's telemetry frame."""

MAX_PAYLOAD: Final[int]
"""The most payload bytes one chunk carries, so that a chunk fits one
1500-byte Ethernet MTU with its IP and UDP headers."""

REQUEST_LEN: Final[int]
"""The length of a request in bytes."""

CHUNK_HEADER_LEN: Final[int]
"""The length of a chunk header in bytes; the payload follows it."""

MAX_CHUNK_LEN: Final[int]
"""The longest chunk: a header and a full payload."""

class WireError(ValueError):
    """A datagram or field the replay protocol does not allow.

    `.kind` names the check that failed: "size", "magic", "version", "type",
    "crc", "no_samples" or "payload". A ValueError, so code that catches bad
    values catches this too.
    """

    kind: str
    """The check that failed."""

@final
class Request:
    """A replay request, as the board sends it: `sample_count` samples from
    `sample_offset`, under the sequence number `seq`."""

    @property
    def seq(self) -> int:
        """The sequence number every chunk of the reply carries back."""
    @property
    def sample_offset(self) -> int:
        """The index in the dataset of the first sample asked for."""
    @property
    def sample_count(self) -> int:
        """How many samples are asked for."""
    @property
    def flags(self) -> int:
        """F_RETRANSMIT and F_LOOP, or'd together."""
    @property
    def crc(self) -> int:
        """The CRC-16/CCITT-FALSE over the 20 bytes before it."""
    def __eq__(self, other: object) -> bool: ...
    def __hash__(self) -> int: ...

@final
class Chunk:
    """One chunk of a reply: a header and `sample_count` rows of samples."""

    @property
    def seq(self) -> int:
        """The `seq` of the request this chunk answers."""
    @property
    def sample_offset(self) -> int:
        """The index in the dataset of the chunk's first sample, after any wrap."""
    @property
    def sample_count(self) -> int:
        """The samples (rows) in this chunk."""
    @property
    def chunk_index(self) -> int:
        """This chunk's place in the reply, from 0."""
    @property
    def chunk_total(self) -> int:
        """The chunks in the reply."""
    @property
    def channel_count(self) -> int:
        """The channels in a row."""
    @property
    def payload_len(self) -> int:
        """The length of the payload in bytes, as the header gives it."""
    @property
    def crc(self) -> int:
        """The CRC-16/CCITT-FALSE over header bytes [0, 26) and the payload."""
    @property
    def payload(self) -> bytes:
        """The rows: `sample_count` x `channel_count` little-endian u16 samples."""
    def __eq__(self, other: object) -> bool: ...
    def __hash__(self) -> int: ...

def crc16(data: bytes | bytearray, crc: int = 0xFFFF) -> int:
    """The CRC-16/CCITT-FALSE of `data`, the checksum on every Argus datagram.

    `crc` is the register to start from: 0xFFFF for a new checksum, or the
    result of an earlier call to go on from it, so that
    `crc16(b, crc16(a)) == crc16(a + b)`.
    """

def encode_request(seq: int, sample_offset: int, sample_count: int, flags: int = 0) -> bytes:
    """The 22 bytes of a replay request for `sample_count` samples from
    `sample_offset`, its CRC sealed, exactly as the board sends it.

    `flags` is F_RETRANSMIT and F_LOOP, or'd together. A field out of range
    raises ValueError. A `sample_count` of 0 is encoded: the relay refuses it,
    and a test may want to send one.
    """

def decode_request(data: bytes | bytearray) -> Request:
    """The request in `data`, checked as the relay checks it: the length, the
    magic, version and type, the CRC, then a nonzero sample count.

    Raises WireError on the first check that fails, with `.kind` "size",
    "magic", "version", "type", "crc" or "no_samples".
    """

def encode_chunk(
    seq: int,
    sample_offset: int,
    chunk_index: int,
    chunk_total: int,
    channel_count: int,
    payload: bytes | bytearray,
) -> bytes:
    """A replay chunk carrying `payload`, its header filled in and its CRC
    sealed, exactly as the relay sends it.

    The payload must be whole rows of `channel_count` little-endian u16
    samples, at most MAX_PAYLOAD bytes; `sample_count` and `payload_len` are
    worked out from it. A payload that is not raises WireError with `.kind`
    "payload"; a `channel_count` of 0 or a field out of range, ValueError.
    """

def decode_chunk(data: bytes | bytearray) -> Chunk:
    """The chunk in `data`, checked on its own: the length, the magic, version
    and type, the payload against the header, then the CRC.

    Raises WireError on the first check that fails, with `.kind` "size",
    "magic", "version", "type", "payload" or "crc". Whether the chunk belongs
    to the fetch in progress (its seq, its index) is the caller's to check.
    """

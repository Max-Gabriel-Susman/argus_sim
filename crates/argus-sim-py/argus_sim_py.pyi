"""The Argus replay relay, for Python.

The board fetches a recorded dataset from the workstation half a buffer at a
time: a 22-byte replay request over UDP, answered with a burst of chunks of
whole sample rows. This module is that protocol and its relay, built from the
same Rust as the dataset_relay_node executable.
"""

import os
from types import TracebackType
from typing import Final, TypedDict, final

from typing_extensions import Self

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

@final
class ReplaySource:
    """A replay dataset: `sample_count` rows of `channel_count` little-endian
    u16 samples, read from a recorded file or generated as the identity
    pattern, exactly as the relay serves it."""

    @staticmethod
    def from_file(path: str | os.PathLike[str], channel_count: int = 96) -> ReplaySource:
        """Opens a replay file: little-endian u16 samples, sample-major, with
        no header, `channel_count` samples per row. The file is read on
        demand, not loaded.

        Raises ValueError when `channel_count` is 0, and OSError when the file
        cannot be opened (FileNotFoundError, PermissionError, ... with
        `.errno` set) or is empty or ends partway through a row.
        """
    @staticmethod
    def synthetic(n_samples: int, channel_count: int = 96) -> ReplaySource:
        """The identity pattern the fabric's self-test expects: `n_samples`
        rows of `channel_count` samples, each the chip (`channel // 32`, two
        bits) in bits 15..14, the channel within the chip in bits 13..8 (0
        to 31, so bit 13 stays 0) and the sample index modulo 256 in bits
        7..0.

        Raises ValueError when either is 0, and MemoryError when the rows
        cannot be held in memory.
        """
    @property
    def sample_count(self) -> int:
        """The rows in the source; never 0."""
    @property
    def channel_count(self) -> int:
        """The samples in a row; never 0."""
    def read(self, sample_offset: int, count: int = 147) -> bytes:
        """`count` rows from row `sample_offset`, as bytes: the rows the relay
        would send for that request. The offset is taken modulo
        `sample_count` and the rows wrap at the end, as many times as it
        takes, so `count` may be more than the whole source.

        Raises ValueError for a negative argument, MemoryError when the rows
        would not fit in memory, and OSError when the file cannot be read (it
        shrank after it was opened, say).
        """

class _RelayStats(TypedDict):
    """The counters `Relay.stats()` returns: a plain dict at run time."""

    requests_served: int
    """Requests answered, retransmits included."""
    retransmits_served: int
    """Of those, the ones flagged F_RETRANSMIT."""
    chunks_sent: int
    """Chunks handed to the socket."""
    chunks_dropped: int
    """Chunks withheld by `drop_mask`."""
    bytes_sent: int
    """Bytes handed to the socket, chunk headers included."""
    bad_packets: int
    """Malformed requests rejected: wrong size, magic, version, type or CRC,
    or a `sample_count` of 0."""
    wraps: int
    """Requests that ran past the end of the source and went on from its
    start."""
    last_offset: int
    """The `sample_offset` of the last request served, after any wrap."""
    last_peer: str | None
    """The dotted quad of the last requester; None before the first."""

@final
class Relay:
    """A replay relay serving a ReplaySource over UDP, on its own thread: the
    server the dataset_relay_node executable runs, answering requests byte for
    byte as the C++ relay did."""

    def __init__(
        self,
        source: ReplaySource,
        port: int = 0,
        bind_addr: str = "127.0.0.1",
        loop: bool = True,
        drop_mask: int = 0,
    ) -> None:
        """A relay for `source`, not yet started. `port` 0 binds a port the
        kernel picks, so that two relays never collide; `.port` tells which.
        `loop` wraps every request at the end of the source (a request can
        also ask for that itself, with F_LOOP); bit N of `drop_mask` withholds
        chunk N of every first attempt, to exercise a client's retransmits.

        Raises ValueError when a row of the source does not fit one chunk, or
        an argument is out of range.
        """
    def start(self) -> None:
        """Binds the socket and starts the relay's thread; it answers requests
        from then on. Does nothing when the relay is already running. A relay
        that was stopped can be started again, on a new socket.

        Raises ValueError for a bind address that is not a dotted quad, and
        OSError when the socket cannot be bound (OSError with errno EADDRINUSE
        for a port in use) or the thread cannot be started.
        """
    def stop(self) -> None:
        """Stops the relay's thread and closes its socket, so the port is free
        again when this returns. That takes up to 200 ms, the thread's receive
        timeout, with the GIL released. Does nothing when the relay is not
        running.
        """
    @property
    def port(self) -> int | None:
        """The UDP port the relay is bound to while it runs, None when it is
        stopped."""
    @property
    def samples_per_chunk(self) -> int:
        """The sample rows in one chunk: as many as fit MAX_PAYLOAD bytes (7
        at 96 channels)."""
    def stats(self) -> _RelayStats:
        """The relay's counters since it was made, as a dict: the requests
        served (retransmits included) and the retransmits among them, the
        chunks sent and withheld by `drop_mask`, the bytes sent, the malformed
        requests rejected, the requests that wrapped, the offset of the last
        request after any wrap, and the address of the last requester, None
        before the first. The counters go on across a stop and a start.
        """
    def __enter__(self) -> Self:
        """Starts the relay; `with Relay(source) as relay:` serves inside the
        block."""
    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc_value: BaseException | None,
        traceback: TracebackType | None,
    ) -> bool:
        """Stops the relay, however the block ended; an exception from the
        block goes on."""

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

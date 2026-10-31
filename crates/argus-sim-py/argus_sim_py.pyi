"""The Argus replay relay, for Python.

The board fetches a recorded dataset from the workstation half a buffer at a
time: a 22-byte replay request over UDP, answered with a burst of chunks of
whole sample rows. This module is that protocol and its relay, built from the
same Rust as the dataset_relay_node executable.
"""

from typing import Final

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

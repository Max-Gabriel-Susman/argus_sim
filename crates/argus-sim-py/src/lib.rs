//! Python bindings for the Argus replay relay.
//!
//! The extension module `argus_sim_py`, built by maturin. It exposes what
//! `argus-sim-core` does to Python, and adds no wire or server logic of its
//! own: every constant here is the one the relay and the board use.
//!
//! The bindings write no unsafe code. PyO3's macros expand to some, which is
//! why this crate cannot `forbid(unsafe_code)` as the others do.

use pyo3::pymodule;

/// The Argus replay relay, for Python.
///
/// The board fetches a recorded dataset from the workstation half a buffer
/// at a time: a 22-byte replay request over UDP, answered with a burst of
/// chunks of whole sample rows. This module is that protocol and its relay,
/// built from the same Rust as the `dataset_relay_node` executable.
#[pymodule]
mod argus_sim_py {
    use argus_sim_core::wire;

    /// The package version, the same as the crate's and package.xml's.
    #[pymodule_export]
    #[expect(
        non_upper_case_globals,
        reason = "Python looks for the version under this exact name"
    )]
    const __version__: &str = env!("CARGO_PKG_VERSION");

    /// The first field of every Argus datagram: "ARGS" read as a
    /// little-endian 32-bit integer.
    #[pymodule_export]
    const MAGIC: u32 = wire::MAGIC;

    /// The replay protocol version, in requests and chunks.
    #[pymodule_export]
    const VERSION: u16 = wire::VERSION;

    /// The `type` of a replay request.
    #[pymodule_export]
    const MSG_REQUEST: u16 = wire::MSG_REQUEST;

    /// The `type` of a replay chunk.
    #[pymodule_export]
    const MSG_CHUNK: u16 = wire::MSG_CHUNK;

    /// Request flag: the request repeats one that went unanswered. The
    /// relay never withholds a chunk of its reply.
    #[pymodule_export]
    const F_RETRANSMIT: u16 = wire::F_RETRANSMIT;

    /// Request flag: wrap at the end of the dataset, even when the relay is
    /// not configured to loop.
    #[pymodule_export]
    const F_LOOP: u16 = wire::F_LOOP;

    /// The UDP port the board sends its requests to.
    #[pymodule_export]
    const REPLAY_PORT: u16 = wire::REPLAY_PORT;

    /// The samples in half of the board's buffer: what it asks for at a
    /// time, and the relay's cap on one request.
    #[pymodule_export]
    const SAMPLES_PER_HALF: u16 = wire::SAMPLES_PER_HALF;

    /// The channels in the board's telemetry frame.
    #[pymodule_export]
    const MAX_CHANNELS: u16 = wire::MAX_CHANNELS;

    /// The most payload bytes one chunk carries, so that a chunk fits one
    /// 1500-byte Ethernet MTU with its IP and UDP headers.
    #[pymodule_export]
    const MAX_PAYLOAD: usize = wire::MAX_PAYLOAD;

    /// The length of a request in bytes.
    #[pymodule_export]
    const REQUEST_LEN: usize = wire::REQUEST_LEN;

    /// The length of a chunk header in bytes; the payload follows it.
    #[pymodule_export]
    const CHUNK_HEADER_LEN: usize = wire::CHUNK_HEADER_LEN;

    /// The longest chunk: a header and a full payload.
    #[pymodule_export]
    const MAX_CHUNK_LEN: usize = wire::MAX_CHUNK_LEN;
}

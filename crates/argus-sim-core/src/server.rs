//! The UDP server that answers replay requests.
//!
//! The firmware's replay client asks for one half-buffer of samples at a
//! time and plays them into the fabric as if the simulated chips were
//! sampling electrodes. [`ReplayServer`] answers each request with those
//! samples as a burst of CRC'd chunks, each sized to fit one Ethernet frame,
//! and answers a retransmit request for any chunk the client missed.
//!
//! [`ReplayServer::start`] binds the socket and spawns the thread; the loop
//! blocks in `recv_from` with a 200 ms timeout, so [`ReplayServer::stop`] is
//! noticed without a wake-up packet. Every request is answered from its own
//! fields alone (offset, count, flags), so the server holds no per-client
//! state and a board reset mid-run needs nothing from it. Requests are
//! answered in the order they arrive, and none expires: one that waited in
//! the socket while the process was stopped is answered like any other.
//!
//! The loop never runs on a ROS executor; the node says why.

use std::fmt;
use std::io;
use std::net::{IpAddr, Ipv4Addr, SocketAddr, SocketAddrV4, UdpSocket};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, PoisonError};
use std::thread::{self, JoinHandle};
use std::time::Duration;

use socket2::{Domain, Socket, Type};

use crate::os_error::strerror;
use crate::source::ReplaySource;
use crate::wire::{
    self, CHUNK_HEADER_LEN, ChunkFields, F_LOOP, F_RETRANSMIT, MAX_CHUNK_LEN, MAX_PAYLOAD,
    REPLAY_PORT, SAMPLES_PER_HALF,
};

/// The receive buffer. A request is 22 bytes; a longer datagram is cut
/// short here and then rejected for its length, as the C++ relay did.
const RX_LEN: usize = 512;

/// How long one `recv_from` waits before the loop looks at the stop flag
/// again: the longest [`ReplayServer::stop`] waits for the thread.
const RECV_TIMEOUT: Duration = Duration::from_millis(200);

/// A burst is about 21 packets back to back. Give the send buffer headroom
/// so a slow drain blocks rather than silently dropping.
const SEND_BUFFER_BYTES: usize = 1 << 20;

/// The name of the server thread, as `top -H` and a debugger show it.
const THREAD_NAME: &str = "replay_server";

/// Everything the server takes besides its source.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Config {
    /// The IPv4 address to bind, dotted quad; the default is every interface.
    pub bind_addr: String,
    /// UDP port; 0 means [`REPLAY_PORT`], as in the C++ relay.
    pub port: u16,
    /// Bind a port the kernel picks instead of [`Config::port`];
    /// [`ReplayServer::local_addr`] tells which. Not in the C++: the tests
    /// and the Python bindings use it so that runs never share a port.
    pub any_port: bool,
    /// Continue from the start when a request runs past the end of the
    /// source. A request can also ask for this itself with [`F_LOOP`].
    pub looping: bool,
    /// Test only: bit N withholds chunk N of a first-attempt request, so the
    /// client's retransmit path gets exercised. Retransmits always land.
    pub drop_mask: u32,
    /// Cap on a request's `sample_count`; 0 means [`SAMPLES_PER_HALF`].
    pub max_samples_per_request: u16,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            bind_addr: "0.0.0.0".to_owned(),
            port: 0,
            any_port: false,
            looping: true,
            drop_mask: 0,
            max_samples_per_request: 0,
        }
    }
}

/// Counters the server keeps across requests. [`ReplayServer::stats`]
/// returns a snapshot; the node prints them as its status line.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct ReplayStats {
    /// Requests rejected: wrong size, magic, version, type or CRC, or a
    /// `sample_count` of 0.
    pub bad_packets: u64,
    /// Bytes handed to `send_to`, chunk headers included.
    pub bytes_sent: u64,
    /// Chunks withheld by [`Config::drop_mask`]. Zero outside
    /// fault-injection tests.
    pub chunks_dropped: u64,
    /// Chunks handed to `send_to`.
    pub chunks_sent: u64,
    /// `sample_offset` of the last request served, after any wrap.
    pub last_offset: u64,
    /// Requests that passed validation, retransmits included.
    pub requests_served: u64,
    /// Of those, the ones flagged [`F_RETRANSMIT`].
    pub retransmits_served: u64,
    /// Requests that ran past the end of the source and continued from its
    /// start.
    pub wraps: u64,
    /// The address of the last requester, normally the board; `None` before
    /// the first request. Its `Display` is the dotted quad the C++ kept as a
    /// string. Kept as an address so that serving a request allocates
    /// nothing.
    pub last_peer: Option<IpAddr>,
}

/// Why the server could not be built or started. The texts are the C++
/// relay's, because the node prints them as its FATAL line.
#[derive(Debug)]
pub enum ServerError {
    /// The source has no samples.
    EmptySource,
    /// One row is longer than a chunk's payload.
    ChannelsTooWide {
        /// The source's channel count.
        channels: u16,
    },
    /// The socket could not be created.
    Socket(io::Error),
    /// [`Config::bind_addr`] is not an IPv4 address.
    BadBindAddress(String),
    /// The socket could not be bound.
    Bind {
        /// [`Config::bind_addr`].
        addr: String,
        /// The port it tried: [`REPLAY_PORT`] for a [`Config::port`] of 0,
        /// and 0 when the kernel was asked to pick one.
        port: u16,
        /// What `bind` reported.
        error: io::Error,
    },
    /// The server thread could not be started.
    Spawn(io::Error),
}

impl fmt::Display for ServerError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::EmptySource => f.write_str("replay source is empty"),
            Self::ChannelsTooWide { channels } => write!(
                f,
                "{channels} channels do not fit one MTU; jumbo frames or channel splitting required"
            ),
            Self::Socket(error) => write!(f, "socket: {}", strerror(error)),
            Self::BadBindAddress(addr) => write!(f, "bad bind address: {addr}"),
            Self::Bind { addr, port, error } => {
                write!(f, "bind {addr}:{port}: {}", strerror(error))
            }
            Self::Spawn(error) => write!(f, "spawn {THREAD_NAME}: {}", strerror(error)),
        }
    }
}

impl std::error::Error for ServerError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Socket(error) | Self::Bind { error, .. } | Self::Spawn(error) => Some(error),
            Self::EmptySource | Self::ChannelsTooWide { .. } | Self::BadBindAddress(_) => None,
        }
    }
}

/// Serves a [`ReplaySource`] over UDP, one request at a time, on its own
/// thread. See the module documentation.
#[derive(Debug)]
pub struct ReplayServer {
    source: Arc<ReplaySource>,
    config: Config,
    samples_per_chunk: u16,
    running: Arc<AtomicBool>,
    stats: Arc<Mutex<ReplayStats>>,
    thread: Option<JoinHandle<()>>,
    local_addr: Option<SocketAddr>,
}

impl ReplayServer {
    /// Sizes the chunks for `source`; nothing is bound until
    /// [`start`](Self::start). A zero port or request cap in `config` takes
    /// the wire header's default.
    ///
    /// # Errors
    ///
    /// [`ServerError::EmptySource`] when the source has no samples, and
    /// [`ServerError::ChannelsTooWide`] when one row does not fit a chunk
    /// payload.
    pub fn new(source: Arc<ReplaySource>, mut config: Config) -> Result<Self, ServerError> {
        if source.sample_count() == 0 {
            return Err(ServerError::EmptySource);
        }
        if config.port == 0 {
            config.port = REPLAY_PORT;
        }
        if config.max_samples_per_request == 0 {
            config.max_samples_per_request = SAMPLES_PER_HALF;
        }

        let fit = MAX_PAYLOAD / source.row_bytes();
        let samples_per_chunk = match u16::try_from(fit) {
            Ok(0) | Err(_) => {
                return Err(ServerError::ChannelsTooWide {
                    channels: source.channel_count(),
                });
            }
            Ok(fit) => fit,
        };

        Ok(Self {
            source,
            config,
            samples_per_chunk,
            running: Arc::new(AtomicBool::new(false)),
            stats: Arc::new(Mutex::new(ReplayStats::default())),
            thread: None,
            local_addr: None,
        })
    }

    /// Binds the socket and starts the server thread. A no-op when already
    /// running.
    ///
    /// # Errors
    ///
    /// [`ServerError::Socket`], [`ServerError::BadBindAddress`] or
    /// [`ServerError::Bind`] when the socket cannot be made or bound, in that
    /// order of checking, and [`ServerError::Spawn`] when the thread cannot
    /// be started. The server is then still stopped.
    pub fn start(&mut self) -> Result<(), ServerError> {
        if self.running.load(Ordering::Relaxed) {
            return Ok(());
        }

        let socket = self.bind()?;
        let local_addr = socket.local_addr().ok();
        let worker = Worker {
            socket,
            source: Arc::clone(&self.source),
            looping: self.config.looping,
            drop_mask: self.config.drop_mask,
            max_samples: self.config.max_samples_per_request,
            samples_per_chunk: self.samples_per_chunk,
            running: Arc::clone(&self.running),
            stats: Arc::clone(&self.stats),
            tx: vec![0u8; MAX_CHUNK_LEN],
        };

        self.running.store(true, Ordering::Relaxed);
        let spawned = thread::Builder::new()
            .name(THREAD_NAME.to_owned())
            .spawn(move || worker.run());
        match spawned {
            Ok(handle) => {
                self.thread = Some(handle);
                self.local_addr = local_addr;
                Ok(())
            }
            Err(error) => {
                self.running.store(false, Ordering::Relaxed);
                Err(ServerError::Spawn(error))
            }
        }
    }

    /// Makes the socket as the C++ did: reuse the address, a 1 MiB send
    /// buffer, the receive timeout, then bind.
    fn bind(&self) -> Result<UdpSocket, ServerError> {
        let socket = Socket::new(Domain::IPV4, Type::DGRAM, None).map_err(ServerError::Socket)?;

        // The C++ ignored a failure of either option; so does this. Neither
        // is needed to serve, only to serve well.
        let _ = socket.set_reuse_address(true);
        let _ = socket.set_send_buffer_size(SEND_BUFFER_BYTES);
        // Bounded recv so stop() is observed without needing to poke the
        // socket. Without it the thread could block forever.
        socket
            .set_read_timeout(Some(RECV_TIMEOUT))
            .map_err(ServerError::Socket)?;

        let ip: Ipv4Addr = self
            .config
            .bind_addr
            .parse()
            .map_err(|_| ServerError::BadBindAddress(self.config.bind_addr.clone()))?;
        let port = if self.config.any_port {
            0
        } else {
            self.config.port
        };
        socket
            .bind(&SocketAddrV4::new(ip, port).into())
            .map_err(|error| ServerError::Bind {
                addr: self.config.bind_addr.clone(),
                port,
                error,
            })?;
        Ok(socket.into())
    }

    /// Stops the thread and closes the socket: within the 200 ms receive
    /// timeout, plus the burst being sent, if any. A no-op when not running; dropping
    /// the server calls it.
    pub fn stop(&mut self) {
        if !self.running.swap(false, Ordering::Relaxed) {
            return;
        }
        if let Some(handle) = self.thread.take() {
            // The socket is the thread's and closes as it ends. A thread
            // that panicked has nothing left to clean up either.
            let _ = handle.join();
        }
        self.local_addr = None;
    }

    /// A snapshot of the counters, taken under the stats lock.
    #[must_use]
    pub fn stats(&self) -> ReplayStats {
        lock(&self.stats).clone()
    }

    /// Samples per chunk: [`MAX_PAYLOAD`] divided by the row size, at
    /// least 1.
    #[must_use]
    pub fn samples_per_chunk(&self) -> u16 {
        self.samples_per_chunk
    }

    /// The address the socket is bound to while the server runs, which tells
    /// the port the kernel picked under [`Config::any_port`]; `None` when
    /// stopped.
    #[must_use]
    pub fn local_addr(&self) -> Option<SocketAddr> {
        self.local_addr
    }
}

impl Drop for ReplayServer {
    fn drop(&mut self) {
        self.stop();
    }
}

/// The counters, whether or not a thread panicked while holding them: they
/// are plain numbers, never left half-updated in a way that matters.
fn lock(stats: &Mutex<ReplayStats>) -> std::sync::MutexGuard<'_, ReplayStats> {
    stats.lock().unwrap_or_else(PoisonError::into_inner)
}

/// Whether the receive loop goes on after `recv_from` failed with `error`.
///
/// A timeout, or a signal interrupting the wait, only means it is time to
/// look at the stop flag again. Any other
/// error ends the thread silently while the node keeps running, as the C++
/// did; kept for parity.
fn recv_goes_on(error: &io::Error) -> bool {
    matches!(
        error.kind(),
        io::ErrorKind::WouldBlock | io::ErrorKind::TimedOut | io::ErrorKind::Interrupted
    )
}

/// The low 32 bits, as the C++ `static_cast<uint32_t>` took them.
fn low_u32(value: u64) -> u32 {
    let [a, b, c, d, ..] = value.to_le_bytes();
    u32::from_le_bytes([a, b, c, d])
}

/// What the server thread owns: the socket, the transmit buffer, and its
/// share of the source, the flag and the counters.
struct Worker {
    socket: UdpSocket,
    source: Arc<ReplaySource>,
    looping: bool,
    drop_mask: u32,
    max_samples: u16,
    samples_per_chunk: u16,
    running: Arc<AtomicBool>,
    stats: Arc<Mutex<ReplayStats>>,
    /// The one transmit buffer, reused for every chunk: a header and a full
    /// payload.
    tx: Vec<u8>,
}

impl Worker {
    /// The receive loop: `recv_from` with a timeout, then `handle`, until
    /// the stop flag clears.
    fn run(mut self) {
        let mut rx = [0u8; RX_LEN];
        while self.running.load(Ordering::Relaxed) {
            match self.socket.recv_from(&mut rx) {
                Ok((len, peer)) => self.handle(rx.get(..len).unwrap_or(&rx), peer),
                Err(error) if recv_goes_on(&error) => {}
                Err(_) => break,
            }
        }
    }

    /// Validates one request and answers it with a burst of chunks.
    fn handle(&mut self, datagram: &[u8], peer: SocketAddr) {
        let Ok(req) = wire::decode_request(datagram) else {
            lock(&self.stats).bad_packets += 1;
            return;
        };

        let n_samples = self.source.sample_count();
        let channels = self.source.channel_count();
        let row_bytes = self.source.row_bytes();
        let spc = self.samples_per_chunk;
        let looping = self.looping || (req.flags & F_LOOP) != 0;
        let retransmit = (req.flags & F_RETRANSMIT) != 0;

        let mut offset = u64::from(req.sample_offset);
        let mut wrapped = false;
        if offset >= n_samples {
            if !looping {
                return;
            }
            offset %= n_samples;
            wrapped = true;
        }

        let mut count = req.sample_count.min(self.max_samples);
        if !looping && offset + u64::from(count) > n_samples {
            // Less than count, so it fits.
            count = u16::try_from(n_samples - offset).unwrap_or(count);
        }

        let total_chunks = count.div_ceil(spc);

        let mut sent_bytes: u64 = 0;
        let mut sent_chunks: u16 = 0;
        let mut dropped_chunks: u16 = 0;

        for index in 0..total_chunks {
            // Fault injection. Applied only to first-attempt requests, so a
            // retransmit always lands and the client's recovery terminates.
            // Keyed off the request flag rather than server state, so the
            // server stays stateless and a PS reset mid-test still behaves.
            if self.drop_mask != 0
                && !retransmit
                && index < 32
                && ((self.drop_mask >> index) & 1) != 0
            {
                dropped_chunks += 1;
                continue;
            }

            // Below count, so no overflow.
            let start = index * spc;
            let in_chunk = spc.min(count - start);
            let payload_len = usize::from(in_chunk) * row_bytes;

            // The rows go straight into the transmit buffer behind the
            // header. read_rows wraps at the end of the dataset, so a wrap
            // needs no special case. A file that can no longer be read ends
            // the burst like a send error; the C++, reading a mapping, would
            // have died of SIGBUS instead.
            let first = offset + u64::from(start);
            let Some(payload) = self
                .tx
                .get_mut(CHUNK_HEADER_LEN..CHUNK_HEADER_LEN + payload_len)
            else {
                break;
            };
            if self.source.read_rows(first, payload).is_err() {
                break;
            }

            let fields = ChunkFields {
                seq: req.seq,
                sample_offset: low_u32(first % n_samples),
                sample_count: in_chunk,
                chunk_index: index,
                chunk_total: total_chunks,
                channel_count: channels,
            };
            // Cannot fail: the payload is at most MAX_PAYLOAD and the buffer
            // is MAX_CHUNK_LEN.
            let Ok(packet_len) = wire::seal_chunk(&mut self.tx, &fields, payload_len) else {
                break;
            };
            let Some(packet) = self.tx.get(..packet_len) else {
                break;
            };
            match self.socket.send_to(packet, peer) {
                Ok(sent) => {
                    sent_bytes += sent as u64;
                    sent_chunks += 1;
                }
                Err(_) => break,
            }
        }

        let mut stats = lock(&self.stats);
        stats.requests_served += 1;
        if retransmit {
            stats.retransmits_served += 1;
        }
        if wrapped || offset + u64::from(count) > n_samples {
            stats.wraps += 1;
        }
        stats.chunks_sent += u64::from(sent_chunks);
        stats.chunks_dropped += u64::from(dropped_chunks);
        stats.bytes_sent += sent_bytes;
        stats.last_offset = offset;
        stats.last_peer = Some(peer.ip());
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use std::time::Instant;

    use argus_wire::generated::argus_replay_chunk_hdr_t;
    use zerocopy::FromBytes;

    use crate::source::identity;
    use crate::wire::{CHUNK_CRC_SPAN, MAGIC, MSG_CHUNK, REQUEST_LEN, VERSION, encode_request};

    type TestResult = Result<(), Box<dyn std::error::Error>>;

    /// The samples in the test source, as in the original three fetches.
    const N: u64 = 30_000;

    /// How long a test waits for a chunk or a counter before failing.
    const PATIENCE: Duration = Duration::from_secs(2);

    /// How long a burst must stay quiet to count as finished: the chunks of
    /// one burst leave back to back, so anything later is a stray.
    const QUIET: Duration = Duration::from_millis(30);

    /// A config bound to an ephemeral port on loopback, as every test is.
    fn loopback() -> Config {
        Config {
            bind_addr: "127.0.0.1".to_owned(),
            any_port: true,
            ..Config::default()
        }
    }

    /// A running server over the synthetic 30000 x 96 identity pattern.
    fn serve(config: Config) -> Result<ReplayServer, Box<dyn std::error::Error>> {
        let source = Arc::new(ReplaySource::synthetic(N, 96)?);
        let mut server = ReplayServer::new(source, config)?;
        server.start()?;
        Ok(server)
    }

    /// The board's side of a test: a socket on loopback that sends to the
    /// server and reads its replies.
    struct Board {
        socket: UdpSocket,
        server: SocketAddr,
    }

    impl Board {
        fn to(server: &ReplayServer) -> Result<Self, Box<dyn std::error::Error>> {
            let socket = UdpSocket::bind("127.0.0.1:0")?;
            let server = server.local_addr().ok_or("the server is not running")?;
            Ok(Self { socket, server })
        }

        /// Sends `datagram` and returns the `expect` replies to it, after
        /// checking that no more follow.
        fn ask(
            &self,
            datagram: &[u8],
            expect: usize,
        ) -> Result<Vec<Vec<u8>>, Box<dyn std::error::Error>> {
            self.socket.send_to(datagram, self.server)?;
            self.replies(expect)
        }

        /// The next `expect` datagrams, then QUIET of nothing.
        fn replies(&self, expect: usize) -> Result<Vec<Vec<u8>>, Box<dyn std::error::Error>> {
            let mut replies = Vec::new();
            let mut buf = [0u8; 2048];
            self.socket.set_read_timeout(Some(PATIENCE))?;
            while replies.len() < expect {
                let len = self.socket.recv(&mut buf)?;
                replies.push(buf.get(..len).ok_or("long datagram")?.to_vec());
            }
            self.socket.set_read_timeout(Some(QUIET))?;
            match self.socket.recv(&mut buf) {
                Ok(len) => Err(format!("a stray reply of {len} bytes after {expect}").into()),
                Err(e) if recv_goes_on(&e) => Ok(replies),
                Err(e) => Err(e.into()),
            }
        }

        /// Sends `datagram`, which must get no reply. A valid request goes
        /// right behind it: requests are answered in arrival order, so a
        /// reply to the first would arrive before the second's and fail the
        /// check on its seq.
        fn ask_unanswered(&self, datagram: &[u8]) -> TestResult {
            self.socket.send_to(datagram, self.server)?;
            let replies = self.ask(&encode_request(0xFEED, 0, 7, 0), 1)?;
            let reply = replies.first().ok_or("no reply")?;
            let (hdr, _) =
                argus_replay_chunk_hdr_t::read_from_prefix(reply).map_err(|_| "short")?;
            assert_eq!({ hdr.seq }, 0xFEED, "the rejected request was answered");
            Ok(())
        }
    }

    /// The counters once `pred` holds of them. The server counts a request
    /// after its last chunk has gone, so a test that has the chunks may still
    /// have to wait a moment for the count.
    fn stats_when(
        server: &ReplayServer,
        pred: impl Fn(&ReplayStats) -> bool,
    ) -> Result<ReplayStats, Box<dyn std::error::Error>> {
        let deadline = Instant::now() + PATIENCE;
        loop {
            let stats = server.stats();
            if pred(&stats) {
                return Ok(stats);
            }
            if Instant::now() > deadline {
                return Err(format!("counters never settled: {stats:?}").into());
            }
            thread::sleep(Duration::from_millis(1));
        }
    }

    /// The counters once `served` requests and `bad` rejects are counted.
    fn settled(
        server: &ReplayServer,
        served: u64,
        bad: u64,
    ) -> Result<ReplayStats, Box<dyn std::error::Error>> {
        stats_when(server, |s| {
            s.requests_served >= served && s.bad_packets >= bad
        })
    }

    /// What one request should get back, at 96 channels: 7 rows of 192
    /// bytes to a chunk.
    struct Burst {
        seq: u32,
        offset: u64,
        count: u16,
        /// Chunk indices withheld by the drop mask.
        dropped: Vec<u16>,
    }

    impl Burst {
        fn of(seq: u32, offset: u64, count: u16) -> Self {
            Self {
                seq,
                offset,
                count,
                dropped: Vec::new(),
            }
        }

        fn total(&self) -> u16 {
            self.count.div_ceil(7)
        }

        fn sent(&self) -> usize {
            usize::from(self.total()) - self.dropped.len()
        }

        fn bytes(&self) -> u64 {
            (0..self.total())
                .filter(|i| !self.dropped.contains(i))
                .map(|i| 28 + 192 * u64::from(7.min(self.count - 7 * i)))
                .sum()
        }
    }

    /// Checks every chunk of `chunks` against `want`: the header fields, the
    /// CRC over header [0, 26) and the payload, and every sample of the
    /// payload against the identity pattern.
    fn check(chunks: &[Vec<u8>], want: &Burst) -> TestResult {
        let indices: Vec<u16> = (0..want.total())
            .filter(|i| !want.dropped.contains(i))
            .collect();
        assert_eq!(chunks.len(), indices.len(), "chunks in the burst");
        for (chunk, index) in chunks.iter().zip(indices) {
            let (hdr, payload) =
                argus_replay_chunk_hdr_t::read_from_prefix(chunk).map_err(|_| "short chunk")?;
            let start = index * 7;
            let rows = 7.min(want.count - start);
            let first = want.offset + u64::from(start);
            assert_eq!({ hdr.magic }, MAGIC);
            assert_eq!({ hdr.version }, VERSION);
            assert_eq!({ hdr.type_ }, MSG_CHUNK);
            assert_eq!({ hdr.seq }, want.seq);
            assert_eq!(u64::from(hdr.sample_offset), first % N, "chunk {index}");
            assert_eq!({ hdr.sample_count }, rows, "chunk {index}");
            assert_eq!({ hdr.chunk_index }, index);
            assert_eq!({ hdr.chunk_total }, want.total());
            assert_eq!({ hdr.channel_count }, 96);
            assert_eq!(usize::from(hdr.payload_len), usize::from(rows) * 192);
            assert_eq!(payload.len(), usize::from(hdr.payload_len));

            let mut covered = chunk.get(..CHUNK_CRC_SPAN).ok_or("short")?.to_vec();
            covered.extend_from_slice(payload);
            assert_eq!(
                { hdr.crc },
                argus_wire::crc::checksum(&covered),
                "chunk {index}"
            );

            for (s, row) in payload.chunks_exact(192).enumerate() {
                let sample = (first + u64::try_from(s)?) % N;
                for (c, code) in row.chunks_exact(2).enumerate() {
                    let code = u16::from_le_bytes(code.try_into()?);
                    assert_eq!(
                        code,
                        identity(sample, u16::try_from(c)?),
                        "sample {sample} channel {c}"
                    );
                }
            }
        }
        Ok(())
    }

    /// Sends one request and checks its reply against `want`.
    fn fetch(board: &Board, flags: u16, want: &Burst) -> TestResult {
        let request = encode_request(want.seq, u32::try_from(want.offset)?, want.count, flags);
        check(&board.ask(&request, want.sent())?, want)
    }

    #[test]
    fn the_three_fetches_are_answered() -> TestResult {
        let server = serve(loopback())?;
        let board = Board::to(&server)?;
        fetch(&board, 0, &Burst::of(1, 0, 147))?;
        fetch(&board, 0, &Burst::of(2, 1000, 147))?;
        let stats = settled(&server, 2, 0)?;
        assert_eq!(stats.wraps, 0);
        assert_eq!(stats.last_offset, 1000);

        // Across the end of the dataset: 73 rows before it, 74 after.
        fetch(&board, 0, &Burst::of(3, N - 73, 147))?;
        let stats = settled(&server, 3, 0)?;
        let bytes = Burst::of(1, 0, 147).bytes() * 3;
        assert_eq!(bytes, 3 * (21 * 28 + 147 * 192));
        assert_eq!(
            stats,
            ReplayStats {
                bad_packets: 0,
                bytes_sent: bytes,
                chunks_dropped: 0,
                chunks_sent: 63,
                last_offset: N - 73,
                requests_served: 3,
                retransmits_served: 0,
                wraps: 1,
                last_peer: Some(IpAddr::V4(Ipv4Addr::LOCALHOST)),
            }
        );
        Ok(())
    }

    #[test]
    fn every_malformed_class_is_rejected_without_a_reply() -> TestResult {
        let server = serve(loopback())?;
        let board = Board::to(&server)?;
        let good = encode_request(9, 0, 147, 0);

        let mut cases: Vec<(&str, Vec<u8>)> = Vec::new();
        cases.push((
            "21 bytes",
            good.get(..REQUEST_LEN - 1).ok_or("short")?.to_vec(),
        ));
        let mut long = good.to_vec();
        long.push(0);
        cases.push(("23 bytes", long));
        // A 600-byte datagram is cut to the 512-byte receive buffer, and
        // then rejected for its length.
        let mut huge = good.to_vec();
        huge.resize(600, 0);
        cases.push(("600 bytes", huge));
        // Each field at its offset, with the CRC made good again so the
        // field is the only fault.
        for (name, at) in [("magic", 0), ("version", 4), ("type", 6)] {
            let mut bad = good;
            *bad.get_mut(at).ok_or("short")? ^= 1;
            let (covered, crc) = bad.split_at_mut(20);
            crc.copy_from_slice(&argus_wire::crc::checksum(covered).to_le_bytes());
            cases.push((name, bad.to_vec()));
        }
        let mut bad_crc = good;
        *bad_crc.get_mut(20).ok_or("short")? ^= 1;
        cases.push(("crc", bad_crc.to_vec()));
        cases.push(("sample_count 0", encode_request(9, 0, 0, 0).to_vec()));

        for (i, (name, datagram)) in (1u64..).zip(&cases) {
            board.ask_unanswered(datagram)?;
            let stats = settled(&server, i, i)?;
            assert_eq!(stats.bad_packets, i, "{name}");
            // Only the follow-up requests were served.
            assert_eq!(stats.requests_served, i, "{name}");
            assert_eq!(stats.chunks_sent, i, "{name}");
        }
        Ok(())
    }

    #[test]
    fn past_the_end_without_loop_gets_nothing_and_counts_nothing() -> TestResult {
        let server = serve(Config {
            looping: false,
            ..loopback()
        })?;
        let board = Board::to(&server)?;
        for offset in [N, N + 1, u64::from(u32::MAX)] {
            let request = encode_request(4, u32::try_from(offset)?, 147, 0);
            board.socket.send_to(&request, board.server)?;
        }
        board.replies(0)?;
        // Give the thread time to have taken all three, then check that not
        // one counter moved, not even the peer.
        thread::sleep(QUIET);
        assert_eq!(server.stats(), ReplayStats::default());

        // The last row is still served.
        fetch(&board, 0, &Burst::of(5, N - 1, 1))?;
        let stats = settled(&server, 1, 0)?;
        assert_eq!((stats.wraps, stats.last_offset), (0, N - 1));
        Ok(())
    }

    #[test]
    fn the_loop_flag_wraps_a_relay_that_does_not_loop() -> TestResult {
        let server = serve(Config {
            looping: false,
            ..loopback()
        })?;
        let board = Board::to(&server)?;
        // An offset past the end is taken modulo the length...
        let request = encode_request(6, u32::try_from(2 * N + 5)?, 14, F_LOOP);
        check(&board.ask(&request, 2)?, &Burst::of(6, 5, 14))?;
        let stats = settled(&server, 1, 0)?;
        assert_eq!((stats.wraps, stats.last_offset), (1, 5));
        // ...and a request across the end continues from the start.
        fetch(&board, F_LOOP, &Burst::of(7, N - 3, 10))?;
        let stats = settled(&server, 2, 0)?;
        assert_eq!((stats.wraps, stats.last_offset), (2, N - 3));
        Ok(())
    }

    #[test]
    fn a_looping_relay_wraps_at_29927() -> TestResult {
        let server = serve(loopback())?;
        let board = Board::to(&server)?;
        fetch(&board, 0, &Burst::of(8, 29_927, 147))?;
        let stats = settled(&server, 1, 0)?;
        assert_eq!(
            (stats.wraps, stats.last_offset, stats.chunks_sent),
            (1, 29_927, 21)
        );

        // An offset of exactly n wraps to 0 and counts as a wrap.
        let request = encode_request(9, u32::try_from(N)?, 7, 0);
        check(&board.ask(&request, 1)?, &Burst::of(9, 0, 7))?;
        let stats = settled(&server, 2, 0)?;
        assert_eq!((stats.wraps, stats.last_offset), (2, 0));
        Ok(())
    }

    #[test]
    fn without_loop_a_request_is_cut_at_the_end() -> TestResult {
        let server = serve(Config {
            looping: false,
            ..loopback()
        })?;
        let board = Board::to(&server)?;
        // Ten rows are left: a chunk of 7 and one of 3. Ending exactly at
        // the end is not a wrap.
        let request = encode_request(10, u32::try_from(N - 10)?, 147, 0);
        check(&board.ask(&request, 2)?, &Burst::of(10, N - 10, 10))?;
        let stats = settled(&server, 1, 0)?;
        assert_eq!(stats.wraps, 0);
        assert_eq!(stats.chunks_sent, 2);
        assert_eq!(stats.bytes_sent, 2 * 28 + 10 * 192);
        Ok(())
    }

    #[test]
    fn a_request_is_capped_at_147_samples() -> TestResult {
        let server = serve(loopback())?;
        let board = Board::to(&server)?;
        for (seq, count) in [(11, 148), (12, 65_535)] {
            let request = encode_request(seq, 100, count, 0);
            check(&board.ask(&request, 21)?, &Burst::of(seq, 100, 147))?;
        }
        let stats = settled(&server, 2, 0)?;
        assert_eq!(stats.chunks_sent, 42);

        // Fewer than one chunk, exactly one, and one row more.
        fetch(&board, 0, &Burst::of(13, 100, 1))?;
        fetch(&board, 0, &Burst::of(14, 100, 7))?;
        fetch(&board, 0, &Burst::of(15, 100, 8))?;
        Ok(())
    }

    #[test]
    fn a_configured_cap_replaces_147() -> TestResult {
        let server = serve(Config {
            max_samples_per_request: 10,
            ..loopback()
        })?;
        let board = Board::to(&server)?;
        let request = encode_request(16, 0, 147, 0);
        check(&board.ask(&request, 2)?, &Burst::of(16, 0, 10))?;
        Ok(())
    }

    #[test]
    fn drop_mask_withholds_chunks_of_first_attempts_only() -> TestResult {
        let server = serve(Config {
            drop_mask: 0x808,
            ..loopback()
        })?;
        let board = Board::to(&server)?;

        let first = Burst {
            dropped: vec![3, 11],
            ..Burst::of(17, 0, 147)
        };
        fetch(&board, 0, &first)?;
        let stats = settled(&server, 1, 0)?;
        assert_eq!((stats.chunks_sent, stats.chunks_dropped), (19, 2));
        assert_eq!(stats.bytes_sent, first.bytes());
        assert_eq!(stats.retransmits_served, 0);

        // A retransmit always lands, whole.
        fetch(&board, F_RETRANSMIT, &Burst::of(17, 0, 147))?;
        fetch(&board, F_RETRANSMIT | F_LOOP, &Burst::of(18, 7, 7))?;
        let stats = settled(&server, 3, 0)?;
        assert_eq!((stats.chunks_sent, stats.chunks_dropped), (19 + 21 + 1, 2));
        assert_eq!(stats.retransmits_served, 2);

        // A burst too short to reach a masked chunk loses nothing.
        fetch(&board, 0, &Burst::of(19, 0, 21))?;
        Ok(())
    }

    #[test]
    fn requests_are_answered_in_arrival_order() -> TestResult {
        let server = serve(loopback())?;
        let board = Board::to(&server)?;
        for seq in 0..20u32 {
            let request = encode_request(seq, seq * 7, 14, 0);
            board.socket.send_to(&request, board.server)?;
        }
        let replies = board.replies(40)?;
        for (seq, pair) in (0u32..).zip(replies.chunks(2)) {
            check(pair, &Burst::of(seq, u64::from(seq) * 7, 14))?;
        }
        settled(&server, 20, 0)?;
        Ok(())
    }

    #[test]
    fn start_and_stop_twice() -> TestResult {
        let mut server = serve(loopback())?;
        let addr = server.local_addr().ok_or("not running")?;
        // A second start while running changes nothing.
        server.start()?;
        assert_eq!(server.local_addr(), Some(addr));
        fetch(&Board::to(&server)?, 0, &Burst::of(20, 0, 7))?;

        let begun = Instant::now();
        server.stop();
        assert!(begun.elapsed() < RECV_TIMEOUT + Duration::from_millis(150));
        assert_eq!(server.local_addr(), None);
        server.stop();

        // The counters carry over a restart, as the C++ kept them.
        server.start()?;
        fetch(&Board::to(&server)?, 0, &Burst::of(21, 7, 7))?;
        let stats = settled(&server, 2, 0)?;
        assert_eq!(stats.chunks_sent, 2);

        let begun = Instant::now();
        server.stop();
        assert!(begun.elapsed() < RECV_TIMEOUT + Duration::from_millis(150));
        server.stop();
        assert_eq!(server.local_addr(), None);
        Ok(())
    }

    #[test]
    fn a_stopped_server_answers_nothing() -> TestResult {
        let mut server = serve(loopback())?;
        let board = Board::to(&server)?;
        server.stop();
        board
            .socket
            .send_to(&encode_request(22, 0, 7, 0), board.server)?;
        board.replies(0)?;
        assert_eq!(server.stats(), ReplayStats::default());
        Ok(())
    }

    #[test]
    fn chunks_are_sized_to_the_channels() -> TestResult {
        for (channels, spc) in [(96, 7), (1, 722), (722, 1)] {
            let source = Arc::new(ReplaySource::synthetic(10, channels)?);
            let server = ReplayServer::new(source, loopback())?;
            assert_eq!(server.samples_per_chunk(), spc, "{channels} channels");
        }
        let source = Arc::new(ReplaySource::synthetic(10, 723)?);
        let err = ReplayServer::new(source, loopback())
            .err()
            .ok_or("723 channels fit")?;
        assert_eq!(
            err.to_string(),
            "723 channels do not fit one MTU; jumbo frames or channel splitting required"
        );
        Ok(())
    }

    #[test]
    fn zero_port_and_cap_take_the_wire_defaults() -> TestResult {
        let source = Arc::new(ReplaySource::synthetic(10, 96)?);
        let server = ReplayServer::new(source, Config::default())?;
        assert_eq!(server.config.port, REPLAY_PORT);
        assert_eq!(server.config.max_samples_per_request, 147);
        assert!(server.config.looping);
        assert_eq!(server.config.bind_addr, "0.0.0.0");
        Ok(())
    }

    #[test]
    fn a_bad_bind_address_is_refused() -> TestResult {
        let source = Arc::new(ReplaySource::synthetic(10, 96)?);
        for addr in ["localhost", "1.2.3", "::1", ""] {
            let config = Config {
                bind_addr: addr.to_owned(),
                ..loopback()
            };
            let mut server = ReplayServer::new(Arc::clone(&source), config)?;
            let err = server.start().err().ok_or("bound")?;
            assert_eq!(err.to_string(), format!("bad bind address: {addr}"));
            assert_eq!(server.local_addr(), None);
            // Still stopped, so a stop is a no-op.
            server.stop();
        }
        Ok(())
    }

    #[test]
    fn a_failed_bind_reports_strerror() -> TestResult {
        let source = Arc::new(ReplaySource::synthetic(10, 96)?);
        // TEST-NET-1: never an address of this host.
        let config = Config {
            bind_addr: "192.0.2.1".to_owned(),
            ..loopback()
        };
        let mut server = ReplayServer::new(source, config)?;
        let err = server.start().err().ok_or("bound")?;
        assert_eq!(
            err.to_string(),
            "bind 192.0.2.1:0: Cannot assign requested address"
        );
        assert!(std::error::Error::source(&err).is_some());
        Ok(())
    }

    #[test]
    fn only_a_timeout_or_an_interruption_keeps_the_receive_loop_going() {
        for kind in [
            io::ErrorKind::WouldBlock,
            io::ErrorKind::TimedOut,
            io::ErrorKind::Interrupted,
        ] {
            assert!(recv_goes_on(&io::Error::from(kind)), "{kind:?}");
        }
        // Any other error ends the thread, as in the C++. ECONNREFUSED is
        // what a connected UDP socket reports after an ICMP unreachable.
        assert!(!recv_goes_on(&io::Error::from_raw_os_error(111)));
        assert!(!recv_goes_on(&io::Error::from(io::ErrorKind::Other)));
    }

    #[test]
    fn low_u32_truncates_as_static_cast_did() {
        assert_eq!(low_u32(29_999), 29_999);
        assert_eq!(low_u32(u64::from(u32::MAX) + 6), 5);
    }
}

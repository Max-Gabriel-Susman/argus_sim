//! The relay over UDP on loopback, asked by the two clients that talk to it.
//!
//! One is argus-replay, the client the board runs. The other is the C
//! header `argus_replay_client.h`, compiled for the host by argus-difftest.
//! Each runs the original host test's three fetches (offset 0, offset 1000,
//! and one that straddles the end of the dataset) against its own relay
//! serving the synthetic 30000 x 96 identity pattern, on real sockets and a
//! real clock. Both must fill their destination with exactly the rows asked
//! for, and the two must end with the same destination, the same counters
//! and the same relay counters.
//!
//! With `drop_mask` 0x808 the relay withholds chunks 3 and 11 of every first
//! attempt, so each fetch completes only through two timeouts and two
//! retransmits. The C++ relay and the C++ test client reported
//! `retransmits=6 timeouts=6 accepted=63 rejected=0` for that run; both
//! clients must report the same against this relay.

use std::error::Error;
use std::io;
use std::net::{SocketAddr, UdpSocket};
use std::sync::Arc;
use std::time::{Duration, Instant};

use argus_difftest::c::{CClient, DST_BYTES};
use argus_replay::{Client, Counters, State, Transport};
use argus_sim_core::server::{Config, ReplayServer, ReplayStats};
use argus_sim_core::source::{ReplaySource, identity};

type TestResult<T = ()> = Result<T, Box<dyn Error>>;

/// Samples in the synthetic source: the node's default.
const N: u32 = 30_000;

/// The board's channel count.
const CHANNELS: u16 = 96;

/// One half-buffer, the size of every fetch here.
const HALF: u16 = 147;

/// The board's client settings: a 200 ms deadline and five retransmits per
/// fetch, as the firmware and the original test client set them.
const TIMEOUT_MS: u32 = 200;
const MAX_RETRIES: u8 = 5;

/// How long one receive waits before the client's timer is polled again,
/// as in the original test client.
const RECV_TIMEOUT: Duration = Duration::from_millis(50);

/// A fetch that has not ended by then has hung: five retransmits at 200 ms
/// take 1.2 s at most.
const FETCH_DEADLINE: Duration = Duration::from_secs(5);

/// The three fetches of the original test: from the start, at an offset,
/// and across the end of the dataset (73 rows before it, 74 after).
const FETCHES: [u32; 3] = [0, 1000, N - 73];

/// A fresh destination of the size argus-difftest compares.
fn zeroed() -> TestResult<Box<[u8; DST_BYTES]>> {
    vec![0u8; DST_BYTES]
        .into_boxed_slice()
        .try_into()
        .map_err(|_| "destination size".into())
}

/// The destination after a fetch of [`HALF`] rows from `offset`: the
/// identity pattern of rows `(offset + s) mod N` in front, as many zeros
/// behind as were there before. Checking all of it catches a misplaced
/// chunk, a wrong offset, a broken seam and a write past the fetch alike.
fn expected(offset: u32) -> TestResult<Box<[u8; DST_BYTES]>> {
    let mut dst = zeroed()?;
    let row_bytes = 2 * usize::from(CHANNELS);
    let rows = dst.chunks_exact_mut(row_bytes).take(usize::from(HALF));
    for (s, row) in (0u64..).zip(rows) {
        let sample = (u64::from(offset) + s) % u64::from(N);
        for (c, code) in (0u16..).zip(row.chunks_exact_mut(2)) {
            code.copy_from_slice(&identity(sample, c).to_le_bytes());
        }
    }
    Ok(dst)
}

/// A relay on an ephemeral loopback port, serving the identity pattern.
fn relay(drop_mask: u32) -> TestResult<ReplayServer> {
    let source = Arc::new(ReplaySource::synthetic(u64::from(N), CHANNELS)?);
    let mut server = ReplayServer::new(
        source,
        Config {
            bind_addr: "127.0.0.1".to_owned(),
            any_port: true,
            drop_mask,
            ..Config::default()
        },
    )?;
    server.start()?;
    Ok(server)
}

/// The board's end of the network: a socket that sends to the relay and
/// receives with a short timeout, so a lost burst shows up as a client
/// timeout rather than a hang.
struct Udp {
    socket: UdpSocket,
    relay: SocketAddr,
}

impl Udp {
    fn to(server: &ReplayServer) -> TestResult<Self> {
        let socket = UdpSocket::bind("127.0.0.1:0")?;
        socket.set_read_timeout(Some(RECV_TIMEOUT))?;
        let relay = server.local_addr().ok_or("the relay is not running")?;
        Ok(Self { socket, relay })
    }

    /// The next datagram into `buf`, or `None` when none came in time.
    fn recv<'a>(&self, buf: &'a mut [u8]) -> TestResult<Option<&'a [u8]>> {
        match self.socket.recv(buf) {
            Ok(len) => Ok(buf.get(..len)),
            Err(e)
                if matches!(
                    e.kind(),
                    io::ErrorKind::WouldBlock | io::ErrorKind::TimedOut
                ) =>
            {
                Ok(None)
            }
            Err(e) => Err(e.into()),
        }
    }
}

impl Transport for Udp {
    type Error = io::Error;

    /// Sends one request; a short send is an error, as in the C callback.
    fn send(&mut self, datagram: &[u8]) -> io::Result<()> {
        let sent = self.socket.send_to(datagram, self.relay)?;
        if sent == datagram.len() {
            Ok(())
        } else {
            Err(io::Error::other("short send"))
        }
    }
}

/// What the fetch loop needs from a client, so that the same loop drives
/// both.
trait BoardClient {
    /// The socket the client's requests go out on and its chunks come in on.
    fn net(&self) -> &Udp;
    /// Starts a fetch and sends its request.
    fn fetch(&mut self, offset: u32, count: u16, now_ms: u32) -> TestResult;
    /// Feeds one received datagram.
    fn on_packet(&mut self, datagram: &[u8]);
    /// Runs the client's timer, which may send a retransmit.
    fn poll(&mut self, now_ms: u32) -> TestResult;
    /// `Some(true)` once the fetch completed, `Some(false)` once it failed.
    fn outcome(&mut self) -> Option<bool>;
    /// Chunks in the current fetch's burst.
    fn total_chunks(&mut self) -> u16;
    /// The five status counters.
    fn counters(&mut self) -> Counters;
    /// The first byte of the destination that differs from `want`.
    fn dst_diff(&mut self, want: &[u8; DST_BYTES]) -> Option<usize>;
}

/// argus-replay, the board's client, writing into a buffer in RAM.
struct RustBoard {
    client: Client,
    net: Udp,
    dst: Box<[u8; DST_BYTES]>,
}

impl RustBoard {
    fn to(server: &ReplayServer) -> TestResult<Self> {
        Ok(Self {
            client: Client::new(CHANNELS, TIMEOUT_MS, MAX_RETRIES).map_err(|e| format!("{e:?}"))?,
            net: Udp::to(server)?,
            dst: zeroed()?,
        })
    }
}

impl BoardClient for RustBoard {
    fn net(&self) -> &Udp {
        &self.net
    }

    fn fetch(&mut self, offset: u32, count: u16, now_ms: u32) -> TestResult {
        self.client
            .fetch(&mut self.net, offset, count, now_ms)
            .map_err(|e| format!("fetch() rejected offset={offset}: {e:?}").into())
    }

    fn on_packet(&mut self, datagram: &[u8]) {
        self.client.on_packet(datagram, self.dst.as_mut_slice());
    }

    fn poll(&mut self, now_ms: u32) -> TestResult {
        Ok(self.client.poll(&mut self.net, now_ms)?)
    }

    fn outcome(&mut self) -> Option<bool> {
        match self.client.state() {
            State::Complete => Some(true),
            State::Failed => Some(false),
            State::Idle | State::Waiting => None,
        }
    }

    fn total_chunks(&mut self) -> u16 {
        self.client.total_chunks()
    }

    fn counters(&mut self) -> Counters {
        self.client.counters()
    }

    fn dst_diff(&mut self, want: &[u8; DST_BYTES]) -> Option<usize> {
        self.dst.iter().zip(want.iter()).position(|(a, b)| a != b)
    }
}

/// The C header's client. Its send callback only records the request, so
/// whenever the count of sends grows, the request it recorded goes out over
/// the socket here.
struct CBoard {
    client: CClient,
    net: Udp,
    sends_seen: u32,
}

impl CBoard {
    fn to(server: &ReplayServer) -> TestResult<Self> {
        Ok(Self {
            client: CClient::new(CHANNELS, TIMEOUT_MS, MAX_RETRIES)
                .map_err(|rc| format!("argus_replay_client_init returned {rc}"))?,
            net: Udp::to(server)?,
            sends_seen: 0,
        })
    }

    /// Sends the request the C client recorded, if it made a new one. Each
    /// call into the client sends at most one.
    fn forward(&mut self) -> TestResult {
        let snap = self.client.snapshot();
        if snap.sends == self.sends_seen {
            return Ok(());
        }
        if snap.sends != self.sends_seen.wrapping_add(1) {
            return Err(format!(
                "{} sends since the last look",
                snap.sends.wrapping_sub(self.sends_seen)
            )
            .into());
        }
        self.sends_seen = snap.sends;
        let len = usize::try_from(snap.sent_len)?;
        let request = snap
            .sent
            .get(..len)
            .ok_or("request longer than the snapshot")?;
        self.net.send(request)?;
        Ok(())
    }
}

impl BoardClient for CBoard {
    fn net(&self) -> &Udp {
        &self.net
    }

    fn fetch(&mut self, offset: u32, count: u16, now_ms: u32) -> TestResult {
        if self.client.fetch(offset, count, now_ms) != 0 {
            return Err(format!("fetch() rejected offset={offset}").into());
        }
        self.forward()
    }

    fn on_packet(&mut self, datagram: &[u8]) {
        self.client.on_packet(datagram);
    }

    fn poll(&mut self, now_ms: u32) -> TestResult {
        self.client.poll(now_ms);
        self.forward()
    }

    fn outcome(&mut self) -> Option<bool> {
        match self.client.snapshot().state {
            2 => Some(true),
            3 => Some(false),
            _ => None,
        }
    }

    fn total_chunks(&mut self) -> u16 {
        u16::try_from(self.client.snapshot().total_chunks).unwrap_or(u16::MAX)
    }

    fn counters(&mut self) -> Counters {
        let snap = self.client.snapshot();
        Counters {
            requests_sent: snap.requests_sent,
            retransmits_sent: snap.retransmits_sent,
            chunks_accepted: snap.chunks_accepted,
            chunks_rejected: snap.chunks_rejected,
            timeouts: snap.timeouts,
        }
    }

    fn dst_diff(&mut self, want: &[u8; DST_BYTES]) -> Option<usize> {
        self.client.dst_diff(want)
    }
}

/// Milliseconds since `start`, the clock both clients run on.
fn now_ms(start: Instant) -> u32 {
    u32::try_from(start.elapsed().as_millis()).unwrap_or(u32::MAX)
}

/// Runs one fetch to completion, pumping the socket and the client's timer,
/// as the original test client's `fetch_block` did.
fn fetch_block(board: &mut impl BoardClient, offset: u32, start: Instant) -> TestResult {
    board.fetch(offset, HALF, now_ms(start))?;
    let deadline = Instant::now() + FETCH_DEADLINE;
    let mut rx = [0u8; 2048];
    loop {
        if let Some(complete) = board.outcome() {
            return if complete {
                Ok(())
            } else {
                Err(format!("the fetch at {offset} failed").into())
            };
        }
        if Instant::now() > deadline {
            return Err(format!("the fetch at {offset} never ended").into());
        }
        if let Some(datagram) = board.net().recv(&mut rx)? {
            board.on_packet(datagram);
        }
        board.poll(now_ms(start))?;
    }
}

/// What one client run leaves behind, for comparing the two.
struct Run {
    counters: Counters,
    relay: ReplayStats,
}

/// The status line of the original test client, minus `requests=`.
fn status(c: &Counters) -> String {
    format!(
        "retransmits={} timeouts={} accepted={} rejected={}",
        c.retransmits_sent, c.timeouts, c.chunks_accepted, c.chunks_rejected
    )
}

/// The three fetches with `board` against `server`, every byte of the
/// destination checked after each, and the relay's counters checked against
/// the client's.
fn three_fetches(board: &mut impl BoardClient, server: &ReplayServer) -> TestResult<Run> {
    let start = Instant::now();
    for offset in FETCHES {
        fetch_block(board, offset, start)?;
        // 147 samples at 7 to a chunk.
        assert_eq!(board.total_chunks(), 21, "chunk count at {offset}");
        if let Some(at) = board.dst_diff(&*expected(offset)?) {
            return Err(format!("fetch at {offset}: destination wrong from byte {at}").into());
        }
    }

    // The relay counts a request after its last chunk is sent, which can be
    // a moment after the client has it.
    let deadline = Instant::now() + Duration::from_secs(2);
    let counters = board.counters();
    let requests = u64::from(counters.requests_sent);
    let retransmits = u64::from(counters.retransmits_sent);
    let relay = loop {
        let stats = server.stats();
        if stats.requests_served >= requests + retransmits || Instant::now() > deadline {
            break stats;
        }
        std::thread::sleep(Duration::from_millis(1));
    };
    // Every request the client sent was answered once, and every chunk
    // withheld was sent again on a retransmit.
    assert_eq!(relay.requests_served, requests + retransmits);
    assert_eq!(relay.retransmits_served, retransmits);
    assert_eq!(relay.chunks_sent, u64::from(counters.chunks_accepted));
    assert_eq!(relay.chunks_dropped, retransmits);
    assert_eq!(relay.bad_packets, 0);
    Ok(Run { counters, relay })
}

/// Both clients, each against its own relay with `drop_mask`; checks that
/// they agree and returns their counters.
fn both_clients(drop_mask: u32) -> TestResult<Counters> {
    let rust_relay = relay(drop_mask)?;
    let mut rust = RustBoard::to(&rust_relay)?;
    let rust_run = three_fetches(&mut rust, &rust_relay)?;

    let c_relay = relay(drop_mask)?;
    let mut c = CBoard::to(&c_relay)?;
    let c_run = three_fetches(&mut c, &c_relay)?;

    // The same rows in the same places, all 46,208 bytes of it.
    if let Some(at) = c.dst_diff(&rust.dst) {
        return Err(format!("the destinations differ from byte {at}").into());
    }
    assert_eq!(c_run.counters, rust_run.counters, "the clients' counters");
    // Both clients sent the same requests from 127.0.0.1, so their relays
    // must have counted the same, down to the last offset and the peer.
    assert_eq!(c_run.relay, rust_run.relay, "the relays' counters");
    Ok(rust_run.counters)
}

#[test]
fn both_clients_fetch_the_identity_pattern() -> TestResult {
    let counters = both_clients(0)?;
    assert_eq!(counters.requests_sent, 3);
    assert_eq!(
        status(&counters),
        "retransmits=0 timeouts=0 accepted=63 rejected=0"
    );
    Ok(())
}

#[test]
fn both_clients_recover_dropped_chunks_as_the_cpp_pair_did() -> TestResult {
    // 0x808 = 2056: chunks 3 and 11 of every first attempt.
    let counters = both_clients(0x808)?;
    assert_eq!(counters.requests_sent, 3);
    assert_eq!(
        status(&counters),
        "retransmits=6 timeouts=6 accepted=63 rejected=0"
    );
    Ok(())
}

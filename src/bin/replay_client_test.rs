//! `replay_client_test`: the host test client for the replay relay.
//!
//! Runs argus-replay, the client the board runs, over a host UDP socket.
//! The PS wraps the same client in lwIP; everything below is transport
//! shell, and none of the protocol logic lives here.
//!
//! Its job is to make the relay a known-good peer, so that when the two ends
//! talk to each other over real hardware, only one of them is unproven.
//!
//! Verifies, beyond the checks inside the client:
//! - the burst arrives as the expected number of chunks;
//! - every sample decodes to the channel and index the identity pattern
//!   claims, so a mis-ordered or mis-offset chunk is caught rather than
//!   averaging out;
//! - a wrap at the end of the dataset produces continuous samples.
//!
//! It needs a live relay serving the synthetic pattern at its default size,
//! so it does not run in CI:
//!
//! ```text
//! ros2 run argus_sim dataset_relay_node --ros-args -p synthetic_samples:=30000
//! ros2 run argus_sim replay_client_test [host] [port]
//! ```
//!
//! What it prints and its exit status (0 on PASS, 1 otherwise) are the C++
//! client's, line for line.

#![forbid(unsafe_code)]

use std::io;
use std::net::{Ipv4Addr, SocketAddrV4, UdpSocket};
use std::process::ExitCode;
use std::time::{Duration, Instant};

use argus_replay::{Client, State, Transport};
use argus_sim_core::os_error::strerror;
use argus_sim_core::source::identity;
use argus_sim_core::wire::{MAX_CHANNELS, REPLAY_PORT, SAMPLES_PER_HALF};

/// Samples in the relay's synthetic source: the node's default.
const N_SAMPLES: u32 = 30_000;

/// The board's client settings: a 200 ms deadline and five retransmits per
/// fetch.
const TIMEOUT_MS: u32 = 200;
const MAX_RETRIES: u8 = 5;

/// Bounded receive, so a dead relay surfaces as a client timeout and retry
/// rather than a hang.
const RECV_TIMEOUT: Duration = Duration::from_millis(50);

/// The largest datagram one receive takes; a chunk is at most 1472 bytes.
const RX_BYTES: usize = 2048;

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().collect();
    let host = args.get(1).map_or("127.0.0.1", String::as_str);
    let port = match args.get(2) {
        None => REPLAY_PORT,
        Some(arg) => match parse_port(arg) {
            Some(port) => port,
            None => {
                eprintln!("replay_client_test: bad port {arg}");
                return ExitCode::FAILURE;
            }
        },
    };
    let channels = MAX_CHANNELS;

    let socket = match UdpSocket::bind((Ipv4Addr::UNSPECIFIED, 0)) {
        Ok(socket) => socket,
        Err(e) => {
            eprintln!("socket: {}", strerror(&e));
            return ExitCode::FAILURE;
        }
    };
    if let Err(e) = socket.set_read_timeout(Some(RECV_TIMEOUT)) {
        eprintln!("setsockopt: {}", strerror(&e));
        return ExitCode::FAILURE;
    }

    let Ok(addr) = host.parse::<Ipv4Addr>() else {
        println!("bad_host {host}");
        return ExitCode::FAILURE;
    };
    let net = UdpTransport {
        socket,
        peer: SocketAddrV4::new(addr, port),
    };

    let Ok(client) = Client::new(channels, TIMEOUT_MS, MAX_RETRIES) else {
        println!("{channels} channels do not fit one MTU");
        return ExitCode::FAILURE;
    };

    println!(
        "replay client -> {host}:{port}, {channels} channels, {} samples/chunk",
        client.samples_per_chunk()
    );

    let half = SAMPLES_PER_HALF;
    let channel_words = usize::from(channels);
    let mut board = Board {
        client,
        net,
        clock: Clock::new(),
        dst: vec![0u8; usize::from(half) * channel_words * 2],
    };
    let mut checks = Checks::default();

    // 1. One half from the start of the dataset.
    println!("test 1: fetch {half} samples at offset 0");
    if board.fetch_block(0, half, &mut checks) {
        let want_chunks = half.div_ceil(board.client.samples_per_chunk());
        checks.check(board.client.total_chunks() == want_chunks, "chunk count");

        let words = words(&board.dst);
        'rows: for (s, row) in (0u32..).zip(words.chunks_exact(channel_words)) {
            for (c, &got) in (0u16..).zip(row) {
                if got != identity(u64::from(s), c) {
                    checks.check(false, &sample_mismatch(s, c, got));
                    // One report is enough.
                    break 'rows;
                }
            }
        }
    } else {
        checks.check(false, "fetch did not complete");
    }

    // 2. A non-zero offset, to catch base_offset arithmetic.
    println!("test 2: fetch {half} samples at offset 1000");
    if board.fetch_block(1000, half, &mut checks) {
        let words = words(&board.dst);
        let channel0 = words.chunks_exact(channel_words).filter_map(<[u16]>::first);
        for (s, &got) in (0u32..).zip(channel0) {
            if got != identity(u64::from(1000 + s), 0) {
                checks.check(false, &format!("offset sample {s}"));
                break;
            }
        }
    } else {
        checks.check(false, "fetch did not complete");
    }

    // 3. A fetch that straddles the end of the dataset. The relay wraps per
    // sample; samples must stay continuous across the seam.
    let straddle = N_SAMPLES - u32::from(half / 2);
    println!("test 3: fetch {half} samples at offset {straddle} (wraps)");
    if board.fetch_block(straddle, half, &mut checks) {
        let words = words(&board.dst);
        let channel0 = words.chunks_exact(channel_words).filter_map(<[u16]>::first);
        for (s, &got) in (0u32..).zip(channel0) {
            let src = (straddle + s) % N_SAMPLES;
            if got != identity(u64::from(src), 0) {
                checks.check(false, &format!("wrap sample {s}"));
                break;
            }
        }
    } else {
        checks.check(false, "fetch did not complete");
    }

    let c = board.client.counters();
    println!(
        "\nrequests={} retransmits={} accepted={} rejected={} timeouts={}",
        c.requests_sent, c.retransmits_sent, c.chunks_accepted, c.chunks_rejected, c.timeouts
    );

    if checks.failures == 0 {
        println!("PASS");
        ExitCode::SUCCESS
    } else {
        println!("FAIL: {} check(s)", checks.failures);
        ExitCode::FAILURE
    }
}

/// The client's way out: a socket that sends each request to the relay.
struct UdpTransport {
    socket: UdpSocket,
    peer: SocketAddrV4,
}

impl Transport for UdpTransport {
    type Error = io::Error;

    /// Sends one request; a short send is an error, as in the C++ callback.
    fn send(&mut self, datagram: &[u8]) -> io::Result<()> {
        let sent = self.socket.send_to(datagram, self.peer)?;
        if sent == datagram.len() {
            Ok(())
        } else {
            Err(io::Error::other("short send"))
        }
    }
}

/// The client's millisecond clock. It runs from when the test started and
/// wraps at 2^32, as the C++ client's `uint32_t` steady-clock count did; the
/// client compares deadlines with wrapping arithmetic, so either is fine.
struct Clock(Instant);

impl Clock {
    fn new() -> Self {
        Self(Instant::now())
    }

    fn now_ms(&self) -> u32 {
        let ms = self.0.elapsed().as_millis() % (1u128 << 32);
        u32::try_from(ms).unwrap_or(0)
    }
}

/// The failed checks, each reported on its own line as it happens.
#[derive(Default)]
struct Checks {
    failures: u32,
}

impl Checks {
    fn check(&mut self, ok: bool, what: &str) {
        if !ok {
            println!(" FAIL {what}");
            self.failures += 1;
        }
    }
}

/// The board's end of the test: the client, the socket it talks over, its
/// clock, and the destination its fetches fill.
struct Board {
    client: Client,
    net: UdpTransport,
    clock: Clock,
    dst: Vec<u8>,
}

impl Board {
    /// Runs one fetch to completion, pumping the socket and the client's
    /// timer. True when every chunk landed.
    fn fetch_block(&mut self, offset: u32, count: u16, checks: &mut Checks) -> bool {
        let now = self.clock.now_ms();
        if self
            .client
            .fetch(&mut self.net, offset, count, now)
            .is_err()
        {
            println!(" FAIL fetch() rejected offset={offset}");
            checks.failures += 1;
            return false;
        }

        let mut rx = [0u8; RX_BYTES];
        while !self.client.is_done() {
            // A timeout, or any other receive error, is just no datagram:
            // the client's own deadline decides when to ask again.
            if let Ok(len) = self.net.socket.recv(&mut rx)
                && len > 0
                && let Some(datagram) = rx.get(..len)
            {
                self.client.on_packet(datagram, self.dst.as_mut_slice());
            }
            // A retransmit that could not be sent is still counted and
            // waited out, and the next deadline tries again; the C++ poll()
            // returned nothing for the same reason.
            let _ = self.client.poll(&mut self.net, self.clock.now_ms());
        }
        self.client.state() == State::Complete
    }
}

/// The destination as the relay's little-endian `u16` samples.
fn words(dst: &[u8]) -> Vec<u16> {
    dst.as_chunks::<2>()
        .0
        .iter()
        .map(|&pair| u16::from_le_bytes(pair))
        .collect()
}

/// The failure line for a wrong sample in test 1. The C++ printed `0x` and
/// then the value in decimal (`std::to_string`); kept so the output matches.
fn sample_mismatch(sample: u32, channel: u16, got: u16) -> String {
    format!("sample {sample} ch {channel}: got 0x{got}")
}

/// The port argument as the C++ read it: `std::stoi`, then a cast to
/// `uint16_t`. Leading white space and a sign are taken, parsing stops at
/// the first non-digit, and the value is reduced mod 65536. `None` where
/// `stoi` would have thrown: no digits, or outside an `int`.
fn parse_port(arg: &str) -> Option<u16> {
    let text = arg.trim_start_matches([' ', '\t', '\n', '\x0b', '\x0c', '\r']);
    let (negative, rest) = match text.strip_prefix('-') {
        Some(rest) => (true, rest),
        None => (false, text.strip_prefix('+').unwrap_or(text)),
    };
    let end = rest
        .find(|c: char| !c.is_ascii_digit())
        .unwrap_or(rest.len());
    let magnitude: i64 = rest.get(..end)?.parse().ok()?;
    let value = i32::try_from(if negative { -magnitude } else { magnitude }).ok()?;
    let [low, high, ..] = value.to_le_bytes();
    Some(u16::from_le_bytes([low, high]))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn port_is_read_as_stoi_reads_it() {
        assert_eq!(parse_port("26010"), Some(26010));
        assert_eq!(parse_port("  +26010x"), Some(26010));
        assert_eq!(parse_port("0x10"), Some(0));
    }

    #[test]
    fn port_narrows_as_the_cast_did() {
        assert_eq!(parse_port("70000"), Some(4464));
        assert_eq!(parse_port("-1"), Some(65535));
        assert_eq!(parse_port("-2147483648"), Some(0));
    }

    #[test]
    fn port_stoi_would_throw_on_is_refused() {
        assert_eq!(parse_port(""), None);
        assert_eq!(parse_port("abc"), None);
        assert_eq!(parse_port("+-5"), None);
        assert_eq!(parse_port("2147483648"), None);
        assert_eq!(parse_port("99999999999999999999"), None);
    }

    #[test]
    fn mismatch_line_prints_0x_before_a_decimal_value() {
        // 0x4105 is 16645: the C++ printed "0x16645".
        assert_eq!(sample_mismatch(5, 1, 0x4105), "sample 5 ch 1: got 0x16645");
    }

    #[test]
    fn words_are_little_endian() {
        assert_eq!(words(&[0x05, 0x41, 0xFF, 0x00]), vec![0x4105, 0x00FF]);
    }

    #[test]
    fn clock_starts_near_zero() {
        assert!(Clock::new().now_ms() < 1000);
    }
}

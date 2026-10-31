//! The relays under the board's load, and the Rust relay through stalls.
//!
//! The board asks for one half buffer, 147 samples, about 204 times a
//! second, and needs each answer back well inside the 4.9 ms that half
//! lasts. This example sends the same requests at 204/s and at twice that,
//! 408/s, for 30 s against the C++ relay and then the Rust one, and times
//! each request from its send to the arrival of its last chunk.
//!
//! ```text
//! cargo run --release -p argus-sim-core --example replay_load -- \
//!     --cpp <c++ prefix>/lib/argus_sim/dataset_relay_node \
//!     --rust <rust prefix>/lib/argus_sim/dataset_relay_node \
//!     --dataset <file>.bin
//! ```
//!
//! Run it with the ROS environment both nodes need already sourced, in a
//! ROS domain of its own. The nodes are named `relay_cpp` and `relay_rs`,
//! bind 127.0.0.1 on ports 26030 and 26031 (`--cpp-port`, `--rust-port`),
//! serve the file looping, and run one at a time, each under its own
//! `timeout`, so neither outlives the example by much even if it dies.
//!
//! Then the stall drill, the board's stall test on the host: while requests
//! go on at 204/s, the Rust relay is stopped with SIGSTOP for 50 ms every
//! 5 s for 30 s (`--seconds`) and resumed with SIGCONT. The board keeps
//! asking through a stall, so the requests of those 50 ms wait in the
//! relay's socket, and there is always one in flight; nothing expires, so
//! every one of them must be answered, in the order they were sent.
//!
//! The example prints the latency table and exits 0 when, at each rate, the
//! Rust p99 is at most the C++ p99 + 0.5 ms, the Rust max is under 4 ms,
//! every request of every run was answered completely and in order, and the
//! Rust relay exits 0 on SIGINT afterwards. Otherwise it exits 1.

#![forbid(unsafe_code)]

use std::error::Error;
use std::fs::File;
use std::io;
use std::net::{SocketAddr, UdpSocket};
use std::path::{Path, PathBuf};
use std::process::{Child, Command, ExitCode, ExitStatus, Stdio};
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread;
use std::time::{Duration, Instant};

use argus_sim_core::wire::{MAX_CHUNK_LEN, MSG_CHUNK, SAMPLES_PER_HALF, encode_request};
use argus_wire::generated::argus_replay_chunk_hdr_t;
use zerocopy::FromBytes;

type Result<T = (), E = Box<dyn Error>> = std::result::Result<T, E>;

/// The channels the dataset has, and the nodes' default.
const CHANNELS: u64 = 96;

/// The request rates: the board's, and twice it for headroom.
const RATES: [u32; 2] = [204, 408];

/// How long a node may take to log that it is serving.
const STARTUP: Duration = Duration::from_secs(15);
/// The `timeout` each node runs under. The Rust node's runs take about two
/// minutes; this only ends a node the example failed to stop.
const NODE_LIFETIME_S: &str = "400";

/// The stall drill: as board.sh's stall injector does it.
const STALL: Duration = Duration::from_millis(50);
const STALL_EVERY: Duration = Duration::from_secs(5);
/// The drill's rate: the board's.
const STALL_RATE: u32 = 204;

/// How long the receiver waits for the last chunks after the last request.
const DRAIN: Duration = Duration::from_millis(500);

/// What the Rust relay must reach, against the C++ relay's numbers.
const P99_MARGIN: Duration = Duration::from_micros(500);
const MAX_LATENCY: Duration = Duration::from_millis(4);

/// What the command line gives.
struct Options {
    cpp_node: PathBuf,
    rust_node: PathBuf,
    dataset: PathBuf,
    cpp_port: u16,
    rust_port: u16,
    seconds: u32,
}

impl Options {
    fn parse() -> Result<Self> {
        let mut cpp = None;
        let mut rust = None;
        let mut dataset = None;
        let mut cpp_port = 26_030;
        let mut rust_port = 26_031;
        let mut seconds = 30;
        let mut args = std::env::args().skip(1);
        while let Some(flag) = args.next() {
            let mut value = || args.next().ok_or_else(|| format!("{flag} needs a value"));
            match flag.as_str() {
                "--cpp" => cpp = Some(PathBuf::from(value()?)),
                "--rust" => rust = Some(PathBuf::from(value()?)),
                "--dataset" => dataset = Some(PathBuf::from(value()?)),
                "--cpp-port" => cpp_port = value()?.parse()?,
                "--rust-port" => rust_port = value()?.parse()?,
                "--seconds" => seconds = value()?.parse()?,
                _ => return Err(format!("unknown argument {flag}").into()),
            }
        }
        let usage = "usage: replay_load --cpp NODE --rust NODE --dataset FILE \
                     [--cpp-port P] [--rust-port P] [--seconds S]";
        Ok(Self {
            cpp_node: cpp.ok_or(usage)?,
            rust_node: rust.ok_or(usage)?,
            dataset: dataset.ok_or(usage)?,
            cpp_port,
            rust_port,
            seconds,
        })
    }
}

/// One node under `timeout`, its output in a file.
struct Node {
    name: &'static str,
    /// The `timeout` process; the node is its child.
    child: Child,
    log: PathBuf,
    port: u16,
    stopped: Option<ExitStatus>,
}

impl Node {
    /// Starts `bin` as node `name` on 127.0.0.1:`port` serving `dataset`,
    /// and waits until it logs that it is serving.
    fn start(name: &'static str, bin: &Path, port: u16, dataset: &str) -> Result<Self> {
        let log =
            std::env::temp_dir().join(format!("replay_load-{}-{name}.log", std::process::id()));
        let out = File::create(&log)?;
        // SIGINT on expiry, as on Ctrl-C, and SIGKILL 5 s later if that is
        // not enough. --preserve-status gives back the node's own status.
        let child = Command::new("timeout")
            .args(["--preserve-status", "-s", "INT", "-k", "5", NODE_LIFETIME_S])
            .arg(bin)
            .args(["--ros-args", "-r", &format!("__node:={name}")])
            .args(["-p", "bind_addr:=127.0.0.1", "-p", &format!("port:={port}")])
            .args(["-p", &format!("dataset_path:={dataset}")])
            .stdin(Stdio::null())
            .stdout(out.try_clone()?)
            .stderr(out)
            .spawn()?;
        let mut node = Self {
            name,
            child,
            log,
            port,
            stopped: None,
        };
        node.wait_for_banner()?;
        Ok(node)
    }

    fn wait_for_banner(&mut self) -> Result {
        let deadline = Instant::now() + STARTUP;
        while Instant::now() < deadline {
            if self.child.try_wait()?.is_some() {
                break;
            }
            if std::fs::read_to_string(&self.log)?.contains("replay server on") {
                return Ok(());
            }
            thread::sleep(Duration::from_millis(50));
        }
        Err(format!(
            "{} did not come up:\n{}",
            self.name,
            std::fs::read_to_string(&self.log)?
        )
        .into())
    }

    fn addr(&self) -> SocketAddr {
        SocketAddr::from(([127, 0, 0, 1], self.port))
    }

    /// The relay's own pid: the one child of the `timeout` this example
    /// started, checked by its kernel name. Stopping `timeout` instead would
    /// leave the relay running.
    fn relay_pid(&self) -> Result<u32> {
        let parent = self.child.id();
        let children = std::fs::read_to_string(format!("/proc/{parent}/task/{parent}/children"))?;
        let mut pids = children.split_whitespace();
        let pid: u32 = pids.next().ok_or("timeout has no child")?.parse()?;
        if pids.next().is_some() {
            return Err("timeout has more than one child".into());
        }
        let comm = std::fs::read_to_string(format!("/proc/{pid}/comm"))?;
        if comm.trim_end() != "dataset_relay_n" {
            return Err(format!("timeout's child {pid} is {comm:?}, not the relay").into());
        }
        Ok(pid)
    }

    /// SIGINT to `timeout`, which passes it on to the node: the way the
    /// node is stopped on the board. Returns how the node exited.
    fn stop(&mut self) -> Result<ExitStatus> {
        if let Some(status) = self.stopped {
            return Ok(status);
        }
        if self.child.try_wait()?.is_none() {
            signal("-INT", self.child.id())?;
        }
        let status = self.child.wait()?;
        self.stopped = Some(status);
        Ok(status)
    }
}

impl Drop for Node {
    fn drop(&mut self) {
        // On an early return the node still gets its SIGINT; the log goes
        // either way. Nothing to report from here, so errors are dropped.
        let _ = self.stop();
        let _ = std::fs::remove_file(&self.log);
    }
}

/// `kill <sig> <pid>`, for a process this example started.
fn signal(sig: &str, pid: u32) -> Result {
    let status = Command::new("kill")
        .args([sig, &pid.to_string()])
        .status()?;
    if !status.success() {
        return Err(format!("kill {sig} {pid}: {status}").into());
    }
    Ok(())
}

/// A relay stopped with SIGSTOP. Dropping it sends SIGCONT, so no early
/// return leaves the relay stopped.
struct Stopped(u32);

impl Stopped {
    fn new(pid: u32) -> Result<Self> {
        signal("-STOP", pid)?;
        Ok(Self(pid))
    }
}

impl Drop for Stopped {
    fn drop(&mut self) {
        // Nothing to report from here; the drill's results show a relay
        // that never came back.
        let _ = signal("-CONT", self.0);
    }
}

/// The stall drill's thread: the stalls it made, or what went wrong. A
/// `String` error, as a thread's result must be `Send`.
type Stall = dyn Fn() -> std::result::Result<u32, String> + Sync;

/// What came back for one request.
#[derive(Clone, Copy, Default)]
struct Answer {
    /// Chunks received.
    chunks: u16,
    /// The burst length the chunks gave.
    total: u16,
    /// When the last of them arrived.
    done: Option<Instant>,
}

/// What one run of requests found.
struct Run {
    /// Request to last chunk, of every request answered completely, sorted.
    latencies: Vec<Duration>,
    sent: usize,
    /// Requests without every chunk of their burst.
    incomplete: usize,
    /// Chunks that did not follow the one before: the next chunk of the same
    /// request, or the first chunk of the next request.
    out_of_order: usize,
    /// Datagrams that were not a chunk of a request this run sent.
    stray: usize,
    /// The stalls made alongside, if any.
    stalls: Option<u32>,
}

impl Run {
    /// The latency at quantile `q`, the nearest rank.
    fn quantile(&self, q: f64) -> Duration {
        // The rank of a 30 s run fits a usize and an f64 exactly.
        #[allow(
            clippy::cast_possible_truncation,
            reason = "a rank of at most a few tens of thousands"
        )]
        let rank = ((q * self.latencies.len() as f64).ceil() as usize).saturating_sub(1);
        self.latencies.get(rank).copied().unwrap_or_default()
    }

    fn max(&self) -> Duration {
        self.latencies.last().copied().unwrap_or_default()
    }

    /// Every request answered, completely and in order.
    fn complete(&self) -> bool {
        self.incomplete == 0 && self.out_of_order == 0 && self.stray == 0
    }
}

/// Receives chunks until `sending` is clear and `DRAIN` passes without one,
/// and returns what came back for each of the `count` requests (seq 0 up).
fn receive(
    socket: &UdpSocket,
    count: usize,
    sending: &AtomicBool,
) -> io::Result<(Vec<Answer>, usize, usize)> {
    let mut answers = vec![Answer::default(); count];
    let mut out_of_order = 0;
    let mut stray = 0;
    // The chunk that should come next: (seq, chunk_index).
    let mut expected = (0usize, 0u16);
    let mut rx = [0u8; MAX_CHUNK_LEN + 1];
    socket.set_read_timeout(Some(DRAIN))?;
    loop {
        let len = match socket.recv(&mut rx) {
            Ok(len) => len,
            Err(e)
                if matches!(
                    e.kind(),
                    io::ErrorKind::WouldBlock | io::ErrorKind::TimedOut
                ) =>
            {
                if sending.load(Ordering::Acquire) {
                    continue;
                }
                return Ok((answers, out_of_order, stray));
            }
            Err(e) => return Err(e),
        };
        let now = Instant::now();
        let Some((hdr, _)) = rx
            .get(..len)
            .and_then(|d| argus_replay_chunk_hdr_t::read_from_prefix(d).ok())
        else {
            stray += 1;
            continue;
        };
        let (index, total) = ({ hdr.chunk_index }, { hdr.chunk_total });
        let Ok(seq) = usize::try_from(hdr.seq) else {
            stray += 1;
            continue;
        };
        let Some(answer) = answers.get_mut(seq).filter(|_| { hdr.type_ } == MSG_CHUNK) else {
            stray += 1;
            continue;
        };
        if (seq, index) != expected {
            out_of_order += 1;
        }
        // Saturating, so a malformed header is counted rather than panicking.
        let next = index.saturating_add(1);
        expected = if next >= total {
            (seq + 1, 0)
        } else {
            (seq, next)
        };
        answer.chunks = answer.chunks.saturating_add(1);
        answer.total = total;
        if answer.chunks == total {
            answer.done = Some(now);
        }
    }
}

/// Sends `rate` requests a second for `seconds` to `relay`, one half buffer
/// each, walking through `n` samples, and times each to its last chunk.
/// `stall`, when given, runs alongside on its own thread and returns the
/// stalls it made.
fn run(relay: SocketAddr, n: u64, rate: u32, seconds: u32, stall: Option<&Stall>) -> Result<Run> {
    let count = usize::try_from(rate * seconds)?;
    let period = Duration::from_secs(1) / rate;
    let socket = UdpSocket::bind("127.0.0.1:0")?;
    socket.connect(relay)?;
    let sending = AtomicBool::new(true);

    let (sent_at, received, stalled) = thread::scope(|s| -> Result<_> {
        let rx = socket.try_clone()?;
        let sending = &sending;
        let receiver = s.spawn(move || receive(&rx, count, sending));
        let staller = stall.map(|f| s.spawn(f));

        // Each request at its own deadline from the start, so a late wake-up
        // does not push the ones after it.
        let mut sent_at = Vec::with_capacity(count);
        let start = Instant::now();
        let send_all = |sent_at: &mut Vec<Instant>| -> Result {
            let mut offset = 0u64;
            for k in 0..count {
                let deadline = start + period * u32::try_from(k)?;
                if let Some(wait) = deadline.checked_duration_since(Instant::now()) {
                    thread::sleep(wait);
                }
                let request = encode_request(
                    u32::try_from(k)?,
                    u32::try_from(offset)?,
                    SAMPLES_PER_HALF,
                    0,
                );
                sent_at.push(Instant::now());
                socket.send(&request)?;
                offset = (offset + u64::from(SAMPLES_PER_HALF)) % n;
            }
            Ok(())
        };
        let sent = send_all(&mut sent_at);
        // Cleared before a send error is returned too: the receiver only
        // ends once it is, and the scope waits for the receiver.
        sending.store(false, Ordering::Release);
        sent?;

        let received = receiver
            .join()
            .map_err(|_| "the receiver thread panicked")??;
        let stalled = match staller {
            Some(h) => Some(h.join().map_err(|_| "the stall thread panicked")??),
            None => None,
        };
        Ok((sent_at, received, stalled))
    })?;

    let (answers, out_of_order, stray) = received;
    let mut latencies = Vec::with_capacity(count);
    let mut incomplete = 0;
    for (sent, answer) in sent_at.iter().zip(&answers) {
        match answer.done {
            Some(done) if answer.total > 0 => latencies.push(done.duration_since(*sent)),
            _ => incomplete += 1,
        }
    }
    latencies.sort_unstable();
    Ok(Run {
        latencies,
        sent: count,
        incomplete,
        out_of_order,
        stray,
        stalls: stalled,
    })
}

/// Stops the relay `pid` for `STALL` every `STALL_EVERY`, starting half a
/// period in, until `seconds` have passed. Returns the stalls made.
fn stall_drill(pid: u32, seconds: u32) -> Result<u32> {
    let start = Instant::now();
    let end = start + Duration::from_secs(u64::from(seconds));
    let mut stalls = 0;
    let mut at = start + STALL_EVERY / 2;
    while at + STALL < end {
        if let Some(wait) = at.checked_duration_since(Instant::now()) {
            thread::sleep(wait);
        }
        let stopped = Stopped::new(pid)?;
        thread::sleep(STALL);
        drop(stopped);
        stalls += 1;
        at += STALL_EVERY;
    }
    Ok(stalls)
}

fn ms(d: Duration) -> String {
    format!("{:.3}", d.as_secs_f64() * 1e3)
}

fn print_run(relay: &str, rate: u32, r: &Run) {
    println!(
        "| {relay} | {rate}/s | {} | {} | {} | {} | {} | {} |",
        r.sent,
        ms(r.quantile(0.5)),
        ms(r.quantile(0.99)),
        ms(r.max()),
        r.incomplete,
        r.out_of_order
    );
}

fn run_all() -> Result<bool> {
    let opts = Options::parse()?;
    let n = std::fs::metadata(&opts.dataset)?.len() / (2 * CHANNELS);
    let dataset = opts
        .dataset
        .to_str()
        .ok_or("the dataset path is not UTF-8")?;

    // Read the file once, so neither relay pays for a cold page cache.
    io::copy(&mut File::open(&opts.dataset)?, &mut io::sink())?;

    println!(
        "replay_load: {n} samples x {CHANNELS} channels, {} s per run",
        opts.seconds
    );
    println!();
    println!("| relay | rate | requests | p50 ms | p99 ms | max ms | incomplete | out of order |");
    println!("| --- | --- | --- | --- | --- | --- | --- | --- |");

    let mut cpp_runs = Vec::new();
    {
        let mut cpp = Node::start("relay_cpp", &opts.cpp_node, opts.cpp_port, dataset)?;
        for rate in RATES {
            let r = run(cpp.addr(), n, rate, opts.seconds, None)?;
            print_run("C++", rate, &r);
            cpp_runs.push(r);
        }
        cpp.stop()?;
    }

    let mut rust = Node::start("relay_rs", &opts.rust_node, opts.rust_port, dataset)?;
    let mut clean = true;
    for (rate, cpp_run) in RATES.into_iter().zip(&cpp_runs) {
        let r = run(rust.addr(), n, rate, opts.seconds, None)?;
        print_run("Rust", rate, &r);
        clean &= r.complete() && cpp_run.complete();
        clean &= r.quantile(0.99) <= cpp_run.quantile(0.99) + P99_MARGIN;
        clean &= r.max() < MAX_LATENCY;
    }

    let pid = rust.relay_pid()?;
    let seconds = opts.seconds;
    let drill = move || stall_drill(pid, seconds).map_err(|e| e.to_string());
    let d = run(rust.addr(), n, STALL_RATE, opts.seconds, Some(&drill))?;
    let exit = rust.stop()?;
    println!();
    println!(
        "stall drill: Rust relay stopped {} ms every {} s, {} stalls; {} requests at {STALL_RATE}/s, \
         {} answered, {} incomplete, {} out of order, {} stray; p50 {} ms, p99 {} ms, max {} ms",
        STALL.as_millis(),
        STALL_EVERY.as_secs(),
        d.stalls.unwrap_or_default(),
        d.sent,
        d.latencies.len(),
        d.incomplete,
        d.out_of_order,
        d.stray,
        ms(d.quantile(0.5)),
        ms(d.quantile(0.99)),
        ms(d.max())
    );
    println!("relay_rs exit: {exit}");
    clean &= d.complete() && exit.success();

    println!("replay_load: {}", if clean { "PASS" } else { "FAIL" });
    Ok(clean)
}

fn main() -> ExitCode {
    match run_all() {
        Ok(true) => ExitCode::SUCCESS,
        Ok(false) => ExitCode::FAILURE,
        Err(e) => {
            eprintln!("replay_load: {e}");
            ExitCode::FAILURE
        }
    }
}

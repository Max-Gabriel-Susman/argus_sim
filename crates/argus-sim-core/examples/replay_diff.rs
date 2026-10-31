//! The C++ relay and the Rust relay side by side, live.
//!
//! `tests/golden.rs` checks the Rust relay against replies recorded from the
//! C++ one. This example checks the two running nodes against each other:
//! it starts both, sends every request to both, and compares what comes
//! back, datagram for datagram. Then it compares their status lines, which
//! the recording has no place for.
//!
//! ```text
//! cargo run --release -p argus-sim-core --example replay_diff -- \
//!     --cpp <c++ prefix>/lib/argus_sim/dataset_relay_node \
//!     --rust <rust prefix>/lib/argus_sim/dataset_relay_node \
//!     --dataset <file>.bin
//! ```
//!
//! Run it with the ROS environment both nodes need already sourced, in a
//! ROS domain of its own: it reads the status topics with `ros2 topic echo`.
//! The nodes are named `relay_cpp` and `relay_rs`, bind 127.0.0.1 on ports
//! 26020 and 26021 (`--cpp-port`, `--rust-port`), and each runs under its own
//! `timeout`, so neither outlives the example by much even if it dies.
//!
//! Three rounds, each with a fresh pair of nodes:
//!
//! - `synthetic`: the nodes' defaults, the identity pattern of 30000 samples
//!   x 96 channels, looping;
//! - `dataset`: the file, looping;
//! - `dataset_no_loop`: the file, not looping, so the end of the real
//!   dataset is checked both ways.
//!
//! Each round sends the corpus of `testdata/relay_corpus.json`, the
//! requests around the end of that round's source, and 500 seeded random
//! requests over it. A reply is every datagram that arrives until 50 ms
//! (`--quiet-ms`) pass without one, as the recorder took it. The example
//! prints one summary per round and exits 0 when there are no differences
//! and the status lines are equal, 1 otherwise.

#![forbid(unsafe_code)]

use std::error::Error;
use std::fs::File;
use std::io;
use std::net::{SocketAddr, UdpSocket};
use std::path::{Path, PathBuf};
use std::process::{Child, Command, ExitCode, ExitStatus, Stdio};
use std::thread;
use std::time::{Duration, Instant};

use argus_sim_core::wire::{F_LOOP, F_RETRANSMIT, decode_request, encode_request};
use serde_json::Value;

type Result<T = (), E = Box<dyn Error>> = std::result::Result<T, E>;

/// The corpus golden.rs replays; its requests are sent here too.
const CORPUS: &str = include_str!("../../../testdata/relay_corpus.json");

/// The channels both nodes serve: their default, and the dataset's.
const CHANNELS: u64 = 96;
/// The samples in the nodes' default synthetic source.
const SYNTHETIC_SAMPLES: u64 = 30_000;

/// The random part of each round. Fixed, so a difference can be reproduced.
const SEED: u64 = 20_161_005;
const RANDOM_REQUESTS: usize = 500;

/// How long a node may take to log that it is serving.
const STARTUP: Duration = Duration::from_secs(15);
/// The `timeout` each node runs under. A round takes about a minute; this
/// only ends a node the example failed to stop.
const NODE_LIFETIME_S: &str = "300";
/// The status period the nodes are given, so a fresh line comes soon after
/// the last request.
const STATUS_PERIOD_S: &str = "0.5";

/// The differences per round reported in full.
const MAX_REPORTED: usize = 10;

/// The flag combinations every edge case is tried under.
const FLAG_SETS: [u16; 4] = [0, F_LOOP, F_RETRANSMIT, F_LOOP | F_RETRANSMIT];

/// A node parameter and its value, as `-p name:=value` passes it.
type Param = (&'static str, String);

/// What the command line gives.
struct Options {
    cpp_node: PathBuf,
    rust_node: PathBuf,
    dataset: PathBuf,
    cpp_port: u16,
    rust_port: u16,
    quiet: Duration,
}

impl Options {
    fn parse() -> Result<Self> {
        let mut cpp = None;
        let mut rust = None;
        let mut dataset = None;
        let mut cpp_port = 26_020;
        let mut rust_port = 26_021;
        let mut quiet = Duration::from_millis(50);
        let mut args = std::env::args().skip(1);
        while let Some(flag) = args.next() {
            let mut value = || args.next().ok_or_else(|| format!("{flag} needs a value"));
            match flag.as_str() {
                "--cpp" => cpp = Some(PathBuf::from(value()?)),
                "--rust" => rust = Some(PathBuf::from(value()?)),
                "--dataset" => dataset = Some(PathBuf::from(value()?)),
                "--cpp-port" => cpp_port = value()?.parse()?,
                "--rust-port" => rust_port = value()?.parse()?,
                "--quiet-ms" => quiet = Duration::from_millis(value()?.parse()?),
                _ => return Err(format!("unknown argument {flag}").into()),
            }
        }
        let usage = "usage: replay_diff --cpp NODE --rust NODE --dataset FILE \
                     [--cpp-port P] [--rust-port P] [--quiet-ms MS]";
        Ok(Self {
            cpp_node: cpp.ok_or(usage)?,
            rust_node: rust.ok_or(usage)?,
            dataset: dataset.ok_or(usage)?,
            cpp_port,
            rust_port,
            quiet,
        })
    }
}

/// SplitMix64: a few lines, no crate, and the same sequence everywhere.
struct Rng(u64);

impl Rng {
    fn next(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    /// A value in `low..high`. The modulo's bias is far below what matters
    /// for picking test requests.
    fn range(&mut self, low: u64, high: u64) -> u64 {
        low + self.next() % (high - low)
    }

    /// True with probability `percent` / 100.
    fn chance(&mut self, percent: u64) -> bool {
        self.next() % 100 < percent
    }
}

/// A request field from a value the generator keeps in range.
fn u16_of(v: u64) -> Result<u16> {
    Ok(u16::try_from(v)?)
}

fn u32_of(v: u64) -> Result<u32> {
    Ok(u32::try_from(v)?)
}

/// The corpus's requests, as recorded.
fn corpus_requests() -> Result<Vec<Vec<u8>>> {
    let doc: Value = serde_json::from_str(CORPUS)?;
    doc.get("requests")
        .and_then(Value::as_array)
        .ok_or("the corpus has no requests")?
        .iter()
        .map(|r| unhex(r.as_str().ok_or("a request is not a string")?))
        .collect()
}

fn unhex(s: &str) -> Result<Vec<u8>> {
    (0..s.len())
        .step_by(2)
        .map(|at| {
            let pair = s.get(at..at + 2).ok_or("odd-length hex")?;
            Ok(u8::from_str_radix(pair, 16)?)
        })
        .collect()
}

/// The requests for a source of `n` samples: the corpus, the end of this
/// source under every count and flag that matters there, and the random
/// ones. The corpus's own edge cases sit at the end of the synthetic source;
/// the second part puts the same cases at the end of the file.
fn requests(n: u64) -> Result<Vec<Vec<u8>>> {
    let mut reqs = corpus_requests()?;

    let mut seq = 0x00A0_0000;
    for offset in [n - 147, n - 1, n, n + 1, 2 * n + 5, u64::from(u32::MAX)] {
        for count in [1, 7, 8, 147, 148] {
            for flags in FLAG_SETS {
                reqs.push(encode_request(seq, u32_of(offset)?, count, flags).to_vec());
                seq += 1;
            }
        }
    }

    // As the recorder drew them: mostly inside the source and up to the
    // board's half buffer, some past the end or larger than the cap.
    let mut rng = Rng(SEED);
    for _ in 0..RANDOM_REQUESTS {
        let offset = if rng.chance(70) {
            rng.range(0, n)
        } else {
            rng.range(n - 200, 3 * n)
        };
        let count = if rng.chance(90) {
            rng.range(1, 148)
        } else {
            rng.range(148, 65_536)
        };
        let flags = *FLAG_SETS
            .get(usize::try_from(rng.range(0, 4))?)
            .ok_or("flag set")?;
        let seq = u32_of(rng.next() >> 32)?;
        reqs.push(encode_request(seq, u32_of(offset)?, u16_of(count)?, flags).to_vec());
    }
    Ok(reqs)
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
    /// Starts `bin` as node `name` on 127.0.0.1:`port` with `params`, and
    /// waits until it logs that it is serving.
    fn start(name: &'static str, bin: &Path, port: u16, params: &[Param]) -> Result<Self> {
        let log =
            std::env::temp_dir().join(format!("replay_diff-{}-{name}.log", std::process::id()));
        let out = File::create(&log)?;
        let mut cmd = Command::new("timeout");
        // SIGINT on expiry, as on Ctrl-C, and SIGKILL 5 s later if that is
        // not enough. --preserve-status gives back the node's own status.
        cmd.args(["--preserve-status", "-s", "INT", "-k", "5", NODE_LIFETIME_S])
            .arg(bin)
            .args(["--ros-args", "-r", &format!("__node:={name}")])
            .args(["-p", "bind_addr:=127.0.0.1", "-p", &format!("port:={port}")])
            .args(["-p", &format!("status_period_s:={STATUS_PERIOD_S}")]);
        for (key, value) in params {
            cmd.args(["-p", &format!("{key}:={value}")]);
        }
        let child = cmd
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

    /// SIGINT to `timeout`, which passes it on to the node: the way the
    /// node is stopped on the board. Returns how the node exited.
    fn stop(&mut self) -> Result<ExitStatus> {
        if let Some(status) = self.stopped {
            return Ok(status);
        }
        if self.child.try_wait()?.is_none() {
            Command::new("kill")
                .args(["-INT", &self.child.id().to_string()])
                .status()?;
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

/// Every datagram until `quiet` passes without one.
fn collect(socket: &UdpSocket, quiet: Duration) -> io::Result<Vec<Vec<u8>>> {
    let mut rx = [0u8; 2048];
    let mut replies = Vec::new();
    socket.set_read_timeout(Some(quiet))?;
    loop {
        match socket.recv(&mut rx) {
            Ok(len) => replies.push(rx.get(..len).unwrap_or_default().to_vec()),
            Err(e)
                if matches!(
                    e.kind(),
                    io::ErrorKind::WouldBlock | io::ErrorKind::TimedOut
                ) =>
            {
                return Ok(replies);
            }
            Err(e) => return Err(e),
        }
    }
}

/// The request as the relays read it, for a difference report.
fn describe(request: &[u8]) -> String {
    match decode_request(request) {
        Ok(r) => format!(
            "seq={} offset={} count={} flags={:#06x}",
            { r.seq },
            { r.sample_offset },
            { r.sample_count },
            { r.flags }
        ),
        Err(reject) => format!("{} bytes, rejected: {reject:?}", request.len()),
    }
}

/// How the Rust node's reply differs from the C++ node's, or `None`.
fn difference(cpp: &[Vec<u8>], rust: &[Vec<u8>]) -> Option<String> {
    if cpp.len() != rust.len() {
        return Some(format!(
            "C++ sent {} datagrams, Rust {}",
            cpp.len(),
            rust.len()
        ));
    }
    for (k, (c, r)) in cpp.iter().zip(rust).enumerate() {
        if c.len() != r.len() {
            return Some(format!(
                "datagram {k}: C++ {} bytes, Rust {}",
                c.len(),
                r.len()
            ));
        }
        if let Some(at) = c.iter().zip(r).position(|(a, b)| a != b) {
            return Some(format!("datagram {k} differs from byte {at}"));
        }
    }
    None
}

/// The last status line a node published, read with `ros2 topic echo`.
fn status_line(node: &str) -> Result<String> {
    let topic = format!("/{node}/status");
    let out = Command::new("timeout")
        .args(["15", "ros2", "topic", "echo", "--once"])
        .args([topic.as_str(), "std_msgs/msg/String"])
        .stdin(Stdio::null())
        .output()?;
    let text = String::from_utf8_lossy(&out.stdout);
    text.lines()
        .find_map(|line| line.strip_prefix("data: "))
        .map(|data| data.trim_matches(|c| c == '\'' || c == '"').to_owned())
        .ok_or_else(|| format!("no message on {topic}: {text}").into())
}

/// What one round found.
struct Round {
    requests: usize,
    datagrams: usize,
    bytes: usize,
    diffs: Vec<String>,
    differing: usize,
    status_cpp: String,
    status_rust: String,
    exit_cpp: ExitStatus,
    exit_rust: ExitStatus,
}

/// Starts both nodes with `params`, sends every request to both, compares
/// the replies, then the status lines, and stops them.
fn round(opts: &Options, name: &str, params: &[Param], reqs: &[Vec<u8>]) -> Result<Round> {
    let mut cpp = Node::start("relay_cpp", &opts.cpp_node, opts.cpp_port, params)?;
    let mut rust = Node::start("relay_rs", &opts.rust_node, opts.rust_port, params)?;
    let to_cpp = UdpSocket::bind("127.0.0.1:0")?;
    let to_rust = UdpSocket::bind("127.0.0.1:0")?;

    let mut result = Round {
        requests: reqs.len(),
        datagrams: 0,
        bytes: 0,
        diffs: Vec::new(),
        differing: 0,
        status_cpp: String::new(),
        status_rust: String::new(),
        exit_cpp: ExitStatus::default(),
        exit_rust: ExitStatus::default(),
    };
    for (i, request) in reqs.iter().enumerate() {
        to_cpp.send_to(request, cpp.addr())?;
        to_rust.send_to(request, rust.addr())?;
        // Both wait for quiet at the same time, so a request costs one quiet
        // period, not two.
        let (got_cpp, got_rust) = thread::scope(|s| {
            let cpp_side = s.spawn(|| collect(&to_cpp, opts.quiet));
            let rust_side = collect(&to_rust, opts.quiet);
            (cpp_side.join(), rust_side)
        });
        let got_cpp = got_cpp.map_err(|_| "the collector thread panicked")??;
        let got_rust = got_rust?;
        result.datagrams += got_cpp.len();
        result.bytes += got_cpp.iter().map(Vec::len).sum::<usize>();
        if let Some(diff) = difference(&got_cpp, &got_rust) {
            result.differing += 1;
            if result.diffs.len() < MAX_REPORTED {
                result.diffs.push(format!(
                    "{name}: request {i} ({}): {diff}",
                    describe(request)
                ));
            }
        }
    }

    // Two status periods, so each line counts every request above.
    thread::sleep(Duration::from_secs(1));
    let (status_cpp, status_rust) = thread::scope(|s| {
        let cpp_side = s.spawn(|| status_line(cpp.name).map_err(|e| e.to_string()));
        let rust_side = status_line(rust.name).map_err(|e| e.to_string());
        (cpp_side.join(), rust_side)
    });
    result.status_cpp = status_cpp.map_err(|_| "the status thread panicked")??;
    result.status_rust = status_rust?;
    result.exit_cpp = cpp.stop()?;
    result.exit_rust = rust.stop()?;
    Ok(result)
}

fn run() -> Result<bool> {
    let opts = Options::parse()?;
    let file_samples = std::fs::metadata(&opts.dataset)?.len() / (2 * CHANNELS);
    let dataset = opts
        .dataset
        .to_str()
        .ok_or("the dataset path is not UTF-8")?;

    let synthetic = requests(SYNTHETIC_SAMPLES)?;
    let from_file = requests(file_samples)?;
    let path = ("dataset_path", dataset.to_owned());
    let rounds = [
        ("synthetic", vec![], synthetic.as_slice()),
        ("dataset", vec![path.clone()], from_file.as_slice()),
        (
            "dataset_no_loop",
            vec![path, ("loop", "false".to_owned())],
            from_file.as_slice(),
        ),
    ];

    let mut clean = true;
    for (name, params, reqs) in rounds {
        let r = round(&opts, name, &params, reqs)?;
        let equal = r.status_cpp == r.status_rust;
        println!(
            "{name}: {} requests, {} datagrams ({} bytes) from the C++ relay, \
             {} replies differ",
            r.requests, r.datagrams, r.bytes, r.differing
        );
        for diff in &r.diffs {
            println!("  {diff}");
        }
        println!("  relay_cpp: {}", r.status_cpp);
        println!("  relay_rs:  {}", r.status_rust);
        println!(
            "  status lines {}; exits: relay_cpp {}, relay_rs {}",
            if equal { "equal" } else { "DIFFER" },
            r.exit_cpp,
            r.exit_rust
        );
        clean &= r.differing == 0 && equal;
    }
    println!(
        "replay_diff: {}",
        if clean {
            "no differences"
        } else {
            "DIFFERENCES"
        }
    );
    Ok(clean)
}

fn main() -> ExitCode {
    match run() {
        Ok(true) => ExitCode::SUCCESS,
        Ok(false) => ExitCode::FAILURE,
        Err(e) => {
            eprintln!("replay_diff: {e}");
            ExitCode::FAILURE
        }
    }
}

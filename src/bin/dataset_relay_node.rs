//! `dataset_relay_node`: the ROS 2 node that serves the replay dataset to the
//! board over UDP.
//!
//! Thin ROS wrapper around [`ReplayServer`]: parameters in, status out.
//! The socket loop runs on `ReplayServer`'s own thread, never on the
//! executor. The PS works to a millisecond-scale latency budget for a buffer
//! swap; an executor shared with a status timer and parameter callbacks is
//! not a place to put that.
//!
//! What it prints, logs and publishes matches the C++ node it replaces line
//! for line, so the launch file, hwtest and anyone reading the logs see no
//! difference.

#![forbid(unsafe_code)]

use std::fmt;
use std::process::ExitCode;
use std::sync::{Arc, Mutex, MutexGuard, PoisonError};
use std::time::Duration;

use argus_sim_core::server::{Config, ReplayServer, ReplayStats, ServerError};
use argus_sim_core::source::{ReplaySource, SourceError};
use argus_sim_core::wire::{MAX_CHANNELS, REPLAY_PORT};
use rclrs::{
    Context, CreateBasicExecutor, DeclarationError, Executor, IntoPrimitiveOptions, Logger,
    MandatoryParameter, Node, Publisher, RclrsError, SpinOptions, Timer, log_fatal, log_info,
    log_warn,
};
use ros_env::std_msgs::msg::String as StringMsg;

/// The node's name, and the logger a startup error goes to: the C++ logged
/// it on `rclcpp::get_logger("dataset_relay")`, whatever the node was
/// renamed to.
const NODE_NAME: &str = "dataset_relay";

/// The widest `channel_count` the node accepts. A row of 4096 channels is
/// far wider than a chunk, so the server refuses it anyway; this bound is the
/// C++ node's, and keeps the value inside a `u16`.
const MAX_CHANNEL_COUNT: i64 = 4096;

/// The shortest status period; a smaller `status_period_s` is raised to it.
const MIN_STATUS_PERIOD_S: f64 = 0.1;

/// The longest timer period rclrs can hand to rcl, which takes nanoseconds
/// as an `i64`.
const MAX_STATUS_PERIOD: Duration = Duration::from_nanos(i64::MAX.unsigned_abs());

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            // A logger is only a name; making one fails only on a name rcl
            // cannot hold, which this one is not.
            if let Ok(logger) = Logger::new(NODE_NAME) {
                log_fatal!(&logger, "{error}");
            } else {
                eprintln!("{error}");
            }
            ExitCode::FAILURE
        }
    }
}

/// Starts the relay, spins until SIGINT or SIGTERM, then stops it.
fn run() -> Result<(), StartupError> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let relay = DatasetRelay::new(&executor)?;

    // rclrs installs no signal handler of its own. This one only asks the
    // executor to stop; the server is stopped below, on this thread, once
    // spin has returned.
    let commands = Arc::clone(executor.commands());
    ctrlc::set_handler(move || commands.halt_spinning())?;

    let errors = executor.spin(SpinOptions::default());
    relay.stop();
    match errors.into_iter().next() {
        Some(error) => Err(error.into()),
        None => Ok(()),
    }
}

/// The node, its relay and the status timer.
struct DatasetRelay {
    _node: Node,
    _parameters: DeclaredParameters,
    _timer: Timer,
    /// Shared with the status timer, which only reads the counters. The
    /// lock is taken once per status tick and once to stop, so the server
    /// thread, which keeps its counters behind its own lock, never waits on
    /// it.
    server: Arc<Mutex<ReplayServer>>,
}

impl DatasetRelay {
    /// Declares the parameters, builds the source, starts the server and the
    /// status timer, logging what it serves, in the C++ node's order.
    fn new(executor: &Executor) -> Result<Self, StartupError> {
        let node = executor.create_node(NODE_NAME)?;
        let declared = DeclaredParameters::declare(&node)?;
        let params = declared.values();
        let logger = node.logger();

        if !(1..=MAX_CHANNEL_COUNT).contains(&params.channel_count) {
            return Err(StartupError::ChannelCountOutOfRange);
        }
        let channels = low_u16(params.channel_count);

        let source = if params.dataset_path.is_empty() {
            // The C++ cast a negative count to a huge size_t and died
            // allocating it; refusing it as no samples at all is the nearest
            // error that can be matched.
            let samples = u64::try_from(params.synthetic_samples).unwrap_or(0);
            let source = ReplaySource::synthetic(samples, channels)?;
            log_warn!(
                logger,
                "no dataset_path set; serving {} samples of synthetic identity pattern",
                low_i32(params.synthetic_samples)
            );
            source
        } else {
            let source = ReplaySource::from_file(&*params.dataset_path, channels)?;
            log_info!(logger, "dataset: {}", params.dataset_path);
            source
        };
        let source = Arc::new(source);

        // The narrowing casts are the C++ node's: port mod 65536 and
        // drop_mask mod 2^32, without an error.
        let config = Config {
            bind_addr: params.bind_addr.to_string(),
            port: low_u16(params.port),
            looping: params.looping,
            drop_mask: low_u32(params.drop_mask),
            ..Config::default()
        };
        if params.drop_mask != 0 {
            log_warn!(
                logger,
                "FAULT INJECTION ACTIVE: drop_mask=0x{:X} -- test use only",
                config.drop_mask
            );
        }

        let mut server = ReplayServer::new(Arc::clone(&source), config)?;
        server.start()?;
        let samples_per_chunk = server.samples_per_chunk();
        let server = Arc::new(Mutex::new(server));

        let publisher = node.create_publisher::<StringMsg>("~/status".keep_last(10))?;
        let timer = node.create_timer_repeating(
            status_period(params.status_period_s),
            status_callback(node.clone(), publisher, Arc::clone(&server)),
        )?;

        log_info!(
            logger,
            "replay server on {}:{} -- {} samples x {} channels, {} samples/chunk, loop={}",
            params.bind_addr,
            low_i32(params.port),
            source.sample_count(),
            source.channel_count(),
            samples_per_chunk,
            params.looping
        );
        if params.channel_count != i64::from(MAX_CHANNELS) {
            log_warn!(
                logger,
                "serving {} channels but ARGUS_MAX_CHANNELS is {}; \
                 the PL and telemetry frame must agree",
                params.channel_count,
                MAX_CHANNELS
            );
        }

        Ok(Self {
            _node: node,
            _parameters: declared,
            _timer: timer,
            server,
        })
    }

    /// Stops the server: the thread ends within its 200 ms receive timeout
    /// and the socket closes.
    fn stop(&self) {
        lock(&self.server).stop();
    }
}

/// The server, whether or not a thread panicked while holding it: stats()
/// and stop() leave nothing half-done.
fn lock(server: &Mutex<ReplayServer>) -> MutexGuard<'_, ReplayServer> {
    server.lock().unwrap_or_else(PoisonError::into_inner)
}

/// The node's parameters as declared, with the C++ node's names, types and
/// defaults. rclrs undeclares a parameter when its handle drops, so the node
/// keeps these for as long as it runs; `ros2 param` then sees all eight.
struct DeclaredParameters {
    bind_addr: MandatoryParameter<Arc<str>>,
    channel_count: MandatoryParameter<i64>,
    dataset_path: MandatoryParameter<Arc<str>>,
    drop_mask: MandatoryParameter<i64>,
    looping: MandatoryParameter<bool>,
    port: MandatoryParameter<i64>,
    status_period_s: MandatoryParameter<f64>,
    synthetic_samples: MandatoryParameter<i64>,
}

impl DeclaredParameters {
    /// Declares the eight parameters, in the C++ order.
    fn declare(node: &Node) -> Result<Self, DeclarationError> {
        Ok(Self {
            bind_addr: node
                .declare_parameter("bind_addr")
                .default(Arc::from("0.0.0.0"))
                .mandatory()?,
            channel_count: node
                .declare_parameter("channel_count")
                .default(i64::from(MAX_CHANNELS))
                .mandatory()?,
            dataset_path: node
                .declare_parameter("dataset_path")
                .default(Arc::from(""))
                .mandatory()?,
            drop_mask: node.declare_parameter("drop_mask").default(0).mandatory()?,
            looping: node.declare_parameter("loop").default(true).mandatory()?,
            port: node
                .declare_parameter("port")
                .default(i64::from(REPLAY_PORT))
                .mandatory()?,
            status_period_s: node
                .declare_parameter("status_period_s")
                .default(2.0)
                .mandatory()?,
            synthetic_samples: node
                .declare_parameter("synthetic_samples")
                .default(30000)
                .mandatory()?,
        })
    }

    /// The values now. The node reads them once, at startup, as the C++
    /// node did: setting one later changes nothing until a restart.
    fn values(&self) -> Parameters {
        Parameters {
            bind_addr: self.bind_addr.get(),
            channel_count: self.channel_count.get(),
            dataset_path: self.dataset_path.get(),
            drop_mask: self.drop_mask.get(),
            looping: self.looping.get(),
            port: self.port.get(),
            status_period_s: self.status_period_s.get(),
            synthetic_samples: self.synthetic_samples.get(),
        }
    }
}

/// The parameters' values at startup. Integers are ROS integers (`i64`);
/// the node narrows them as the C++ did.
struct Parameters {
    bind_addr: Arc<str>,
    channel_count: i64,
    dataset_path: Arc<str>,
    drop_mask: i64,
    looping: bool,
    port: i64,
    status_period_s: f64,
    synthetic_samples: i64,
}

/// The status timer's period: `status_period_s` seconds, at least 0.1.
/// A NaN takes the minimum, as `std::max(0.1, NaN)` did in the C++; a period
/// too long for rcl takes the longest it can.
fn status_period(seconds: f64) -> Duration {
    Duration::try_from_secs_f64(seconds.max(MIN_STATUS_PERIOD_S))
        .map_or(MAX_STATUS_PERIOD, |period| period.min(MAX_STATUS_PERIOD))
}

/// The status timer's callback: publishes the counters, and warns when
/// malformed requests arrived since the last tick.
fn status_callback(
    node: Node,
    publisher: Publisher<StringMsg>,
    server: Arc<Mutex<ReplayServer>>,
) -> impl FnMut() + Send + 'static {
    let mut last_bad = 0;
    move || {
        let stats = lock(&server).stats();

        // Underrun on the PS side shows up here as a request rate below the
        // play rate; a silent gap is worse than a noisy one during bring-up.
        let msg = StringMsg {
            data: status_text(&stats),
        };
        if let Err(error) = publisher.publish(msg) {
            log_warn!(node.logger(), "status not published: {error}");
        }

        if stats.bad_packets != last_bad {
            log_warn!(
                node.logger(),
                "{} malformed request(s) rejected",
                stats.bad_packets.wrapping_sub(last_bad)
            );
            last_bad = stats.bad_packets;
        }
    }
}

/// The status line, with `peer=none` before the first request.
fn status_text(stats: &ReplayStats) -> String {
    let peer: &dyn fmt::Display = match &stats.last_peer {
        Some(peer) => peer,
        None => &"none",
    };
    format!(
        "requests={} retransmits={} chunks={} dropped={} bytes={} bad={} wraps={} offset={} peer={}",
        stats.requests_served,
        stats.retransmits_served,
        stats.chunks_sent,
        stats.chunks_dropped,
        stats.bytes_sent,
        stats.bad_packets,
        stats.wraps,
        stats.last_offset,
        peer
    )
}

/// The low 16 bits, as the C++ `static_cast<uint16_t>` took them.
fn low_u16(value: i64) -> u16 {
    let [a, b, ..] = value.to_le_bytes();
    u16::from_le_bytes([a, b])
}

/// The low 32 bits, as the C++ `static_cast<uint32_t>` took them.
fn low_u32(value: i64) -> u32 {
    let [a, b, c, d, ..] = value.to_le_bytes();
    u32::from_le_bytes([a, b, c, d])
}

/// The low 32 bits as a signed value, as the C++ `static_cast<int>` took
/// them for its log lines. A port of 70000 is logged as 70000 by both, and
/// bound as 4464.
fn low_i32(value: i64) -> i32 {
    let [a, b, c, d, ..] = value.to_le_bytes();
    i32::from_le_bytes([a, b, c, d])
}

/// Why the node did not start. Its text is the FATAL line.
#[derive(Debug)]
enum StartupError {
    /// `channel_count` is outside 1..=4096.
    ChannelCountOutOfRange,
    /// The dataset could not be opened or built.
    Source(SourceError),
    /// The server could not be built or bound.
    Server(ServerError),
    /// A parameter could not be declared, typically a value of the wrong
    /// type on the command line.
    Parameter(DeclarationError),
    /// rclrs failed: the context, the node, the publisher or the timer.
    Ros(RclrsError),
    /// The SIGINT and SIGTERM handler could not be installed.
    Signal(ctrlc::Error),
}

impl fmt::Display for StartupError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::ChannelCountOutOfRange => f.write_str("channel_count out of range"),
            Self::Source(error) => error.fmt(f),
            Self::Server(error) => error.fmt(f),
            Self::Parameter(error) => error.fmt(f),
            Self::Ros(error) => error.fmt(f),
            Self::Signal(error) => write!(f, "signal handler: {error}"),
        }
    }
}

impl From<SourceError> for StartupError {
    fn from(error: SourceError) -> Self {
        Self::Source(error)
    }
}

impl From<ServerError> for StartupError {
    fn from(error: ServerError) -> Self {
        Self::Server(error)
    }
}

impl From<DeclarationError> for StartupError {
    fn from(error: DeclarationError) -> Self {
        Self::Parameter(error)
    }
}

impl From<RclrsError> for StartupError {
    fn from(error: RclrsError) -> Self {
        Self::Ros(error)
    }
}

impl From<ctrlc::Error> for StartupError {
    fn from(error: ctrlc::Error) -> Self {
        Self::Signal(error)
    }
}

#[cfg(test)]
mod tests {
    use std::net::{IpAddr, Ipv4Addr};

    use super::*;

    #[test]
    fn integers_narrow_as_the_cpp_casts() {
        assert_eq!(low_u16(26010), 26010);
        assert_eq!(low_u16(26010 + 65536), 26010);
        assert_eq!(low_u16(-1), 65535);
        assert_eq!(low_u16(65536), 0);
        assert_eq!(low_u32(2056), 0x808);
        assert_eq!(low_u32((1 << 32) + 1), 1);
        assert_eq!(low_u32(-1), u32::MAX);
        assert_eq!(low_i32(70000), 70000);
        assert_eq!(low_i32(1 << 32), 0);
        assert_eq!(low_i32(-5), -5);
    }

    #[test]
    fn status_period_has_a_floor_and_a_ceiling() {
        assert_eq!(status_period(2.0), Duration::from_secs(2));
        assert_eq!(status_period(0.01), Duration::from_millis(100));
        assert_eq!(status_period(-3.0), Duration::from_millis(100));
        assert_eq!(status_period(f64::NAN), Duration::from_millis(100));
        assert_eq!(status_period(f64::INFINITY), MAX_STATUS_PERIOD);
        assert_eq!(status_period(1e30), MAX_STATUS_PERIOD);
    }

    #[test]
    fn status_text_says_none_before_the_first_request() {
        assert_eq!(
            status_text(&ReplayStats::default()),
            "requests=0 retransmits=0 chunks=0 dropped=0 bytes=0 bad=0 wraps=0 offset=0 peer=none"
        );
    }

    #[test]
    fn status_text_prints_every_counter() {
        let stats = ReplayStats {
            bad_packets: 6,
            bytes_sent: 5,
            chunks_dropped: 4,
            chunks_sent: 3,
            last_offset: 8,
            requests_served: 1,
            retransmits_served: 2,
            wraps: 7,
            last_peer: Some(IpAddr::V4(Ipv4Addr::new(192, 168, 1, 10))),
        };
        assert_eq!(
            status_text(&stats),
            "requests=1 retransmits=2 chunks=3 dropped=4 bytes=5 bad=6 wraps=7 offset=8 \
             peer=192.168.1.10"
        );
    }

    #[test]
    fn startup_errors_print_the_cpp_texts() {
        assert_eq!(
            StartupError::ChannelCountOutOfRange.to_string(),
            "channel_count out of range"
        );
        assert_eq!(
            StartupError::from(SourceError::NoSyntheticDimensions).to_string(),
            "sythetic source needs positive dimensions"
        );
        assert_eq!(
            StartupError::from(ServerError::BadBindAddress("x".to_owned())).to_string(),
            "bad bind address: x"
        );
    }
}

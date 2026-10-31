//! The `replay_client_test` executable against an in-process relay on
//! loopback.
//!
//! The argus_sim package's own test, so that `colcon test` (which runs
//! `cargo test` for this package only) exercises the relay and the board's
//! client end to end and not just the executables' unit tests. Nothing here
//! needs ROS or a network beyond 127.0.0.1: the relay is argus-sim-core's
//! server on a port the kernel picks, and the executable is the one Cargo
//! built for this test.

use std::error::Error;
use std::net::{Ipv4Addr, UdpSocket};
use std::process::{Command, Output};
use std::sync::Arc;

use argus_sim_core::server::{Config, ReplayServer, ReplayStats};
use argus_sim_core::source::ReplaySource;

type TestResult<T = ()> = Result<T, Box<dyn Error>>;

/// The relay's synthetic source as the client expects it: the node's
/// default size, the board's channel count.
const N_SAMPLES: u64 = 30_000;
const CHANNELS: u16 = 96;

/// A relay on 127.0.0.1 at a port the kernel picks, started.
fn relay(drop_mask: u32) -> TestResult<(ReplayServer, u16)> {
    let source = Arc::new(ReplaySource::synthetic(N_SAMPLES, CHANNELS)?);
    let config = Config {
        bind_addr: Ipv4Addr::LOCALHOST.to_string(),
        any_port: true,
        drop_mask,
        ..Config::default()
    };
    let mut server = ReplayServer::new(source, config)?;
    server.start()?;
    let port = server.local_addr().ok_or("relay has no address")?.port();
    Ok((server, port))
}

/// Runs the executable against 127.0.0.1:`port`.
fn client(port: u16) -> TestResult<Output> {
    let output = Command::new(env!("CARGO_BIN_EXE_replay_client_test"))
        .arg(Ipv4Addr::LOCALHOST.to_string())
        .arg(port.to_string())
        .output()?;
    Ok(output)
}

/// The client's stdout as lines.
fn lines(output: &Output) -> TestResult<Vec<String>> {
    let text = String::from_utf8(output.stdout.clone())?;
    Ok(text.lines().map(str::to_owned).collect())
}

#[test]
fn passes_against_the_relay() -> TestResult {
    let (mut server, port) = relay(0)?;
    let output = client(port)?;
    server.stop();

    let lines = lines(&output)?;
    assert_eq!(output.status.code(), Some(0), "{lines:?}");
    assert_eq!(
        lines.first().map(String::as_str),
        Some(format!("replay client -> 127.0.0.1:{port}, 96 channels, 7 samples/chunk").as_str())
    );
    let tail: Vec<&str> = lines
        .iter()
        .rev()
        .take(2)
        .rev()
        .map(String::as_str)
        .collect();
    assert_eq!(
        tail,
        [
            "requests=3 retransmits=0 accepted=63 rejected=0 timeouts=0",
            "PASS"
        ]
    );

    // The relay saw the same three fetches: 21 chunks each, the last one
    // across the end of the dataset.
    let stats = server.stats();
    assert_eq!(
        stats,
        ReplayStats {
            requests_served: 3,
            chunks_sent: 63,
            bytes_sent: 86_436,
            wraps: 1,
            last_offset: 29_927,
            last_peer: Some(Ipv4Addr::LOCALHOST.into()),
            ..ReplayStats::default()
        }
    );
    Ok(())
}

/// With chunks 3 and 11 of every first attempt withheld, each fetch
/// completes through two timeouts and two retransmits: the line the C++
/// relay and C++ client printed for the same run.
#[test]
fn recovers_withheld_chunks() -> TestResult {
    let (mut server, port) = relay(0x808)?;
    let output = client(port)?;
    server.stop();

    let lines = lines(&output)?;
    assert_eq!(output.status.code(), Some(0), "{lines:?}");
    assert!(
        lines
            .iter()
            .any(|l| l == "requests=3 retransmits=6 accepted=63 rejected=0 timeouts=6"),
        "{lines:?}"
    );
    let stats = server.stats();
    assert_eq!(stats.retransmits_served, 6);
    assert_eq!(stats.chunks_dropped, 6);
    Ok(())
}

/// A peer that never answers: every fetch runs out of retransmits, and the
/// executable says so and exits 1.
#[test]
fn fails_without_a_relay() -> TestResult {
    // Held open, so nothing else takes the port while the client sends to it.
    let silent = UdpSocket::bind((Ipv4Addr::LOCALHOST, 0))?;
    let port = silent.local_addr()?.port();
    let output = client(port)?;

    let lines = lines(&output)?;
    assert_eq!(output.status.code(), Some(1), "{lines:?}");
    let failed = lines
        .iter()
        .filter(|l| *l == " FAIL fetch did not complete")
        .count();
    assert_eq!(failed, 3, "{lines:?}");
    assert_eq!(lines.last().map(String::as_str), Some("FAIL: 3 check(s)"));
    Ok(())
}

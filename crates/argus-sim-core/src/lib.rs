//! The Argus replay relay, without ROS.
//!
//! The board plays a recorded dataset into the fabric's simulated amplifier
//! chips. It fetches the dataset from the workstation half a buffer at a
//! time: it sends a replay request over UDP, and the relay answers with a
//! burst of chunks carrying the requested samples. This crate is that relay:
//!
//! - [`wire`]: requests and chunks as the relay reads and writes them, on
//!   top of the structs and the CRC from `argus-wire`;
//! - [`source`]: the dataset, a file of rows of little-endian `u16` samples
//!   or a synthetic identity pattern;
//! - [`server`]: the UDP server that answers requests from a source.
//!
//! The `dataset_relay_node` executable wraps it in a ROS 2 node. Nothing
//! here needs ROS, so the tests run on any host.

#![forbid(unsafe_code)]

mod os_error;
pub mod server;
pub mod source;
pub mod wire;

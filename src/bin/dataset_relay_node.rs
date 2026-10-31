//! `dataset_relay_node`: the ROS 2 node that serves the replay dataset to the
//! board over UDP.
//!
//! Not ported yet: it says so and exits 1, so nothing mistakes it for the
//! relay.

#![forbid(unsafe_code)]

use std::process::ExitCode;

fn main() -> ExitCode {
    eprintln!("dataset_relay_node: not ported yet");
    ExitCode::FAILURE
}

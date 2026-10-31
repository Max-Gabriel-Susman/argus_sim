//! `replay_client_test`: fetches from a running relay with the board's replay
//! client and checks what comes back.
//!
//! Not ported yet: it says so and exits 1, so no check reads as a PASS.

#![forbid(unsafe_code)]

use std::process::ExitCode;

fn main() -> ExitCode {
    eprintln!("replay_client_test: not ported yet");
    ExitCode::FAILURE
}

//! The UDP server that answers replay requests.
//!
//! One thread of its own receives requests, in arrival order, and answers
//! each with a burst of chunks read from a [`crate::source`], keeping the
//! counters the node publishes as its status line.

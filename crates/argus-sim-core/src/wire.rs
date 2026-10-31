//! Replay requests and chunks as the relay reads and writes them.
//!
//! The structs, the constants and the CRC are `argus-wire`'s, shared with the
//! firmware; this module checks a received request against them and builds
//! the chunks of a reply. Nothing in it redefines the wire format.

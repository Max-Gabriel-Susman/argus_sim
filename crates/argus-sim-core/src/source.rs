//! The dataset the relay serves.
//!
//! A source is a sequence of rows, one row per sample, each row one
//! little-endian `u16` per channel. It is read from a recorded `.bin` file or
//! generated as a synthetic identity pattern, and read back in runs of rows
//! that wrap at the end, so a looping replay never runs out.

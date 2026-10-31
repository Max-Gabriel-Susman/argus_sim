//! Operating system errors in the words the C++ relay used.
//!
//! The C++ printed `strerror(errno)`. A Rust `io::Error` from the OS prints
//! the same `strerror` text and then ` (os error N)`; the log lines and the
//! error texts the node prints must match the C++, so that suffix is cut.

use std::io;

/// The `strerror` text of `err`, without the ` (os error N)` Rust adds.
/// An error that did not come from the OS keeps its own text.
pub fn strerror(err: &io::Error) -> String {
    let text = err.to_string();
    let Some(code) = err.raw_os_error() else {
        return text;
    };
    let suffix = format!(" (os error {code})");
    match text.strip_suffix(&suffix) {
        Some(bare) => bare.to_owned(),
        None => text,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cuts_the_os_error_suffix() {
        let err = io::Error::from_raw_os_error(2);
        assert_eq!(strerror(&err), "No such file or directory");
    }

    #[test]
    fn keeps_a_custom_error_as_it_is() {
        let err = io::Error::other("short read");
        assert_eq!(strerror(&err), "short read");
    }
}

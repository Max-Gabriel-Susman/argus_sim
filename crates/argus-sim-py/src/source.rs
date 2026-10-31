//! The dataset source for Python: a recorded file or the identity pattern.
//!
//! A thin wrapper over `argus_sim_core::source`, the code the relay serves
//! from. What this module adds is the Python side: the source's errors as
//! the exceptions Python code expects, with the C++ relay's texts, and reads
//! returned as `bytes` filled in place, with the GIL released while the rows
//! are read.

use std::io;
use std::path::PathBuf;
use std::sync::Arc;

use argus_sim_core::os_error::strerror;
use argus_sim_core::source::{self, SourceError};
use argus_sim_core::wire;
use pyo3::exceptions::{PyMemoryError, PyOSError, PyValueError};
use pyo3::prelude::*;
use pyo3::types::PyBytes;

use crate::args::Unsigned;

/// An OSError for `error`, of the subclass Python would raise for it
/// (FileNotFoundError for ENOENT, PermissionError for EACCES, ...), with
/// `text` as its message and the OS's code as its `errno`.
///
/// Only `errno` is set, not `strerror` or `filename`: with those, Python
/// prints its own `[Errno N] ...` text instead of `text`, and `text` is the
/// C++ relay's.
pub(crate) fn os_error(py: Python<'_>, error: &io::Error, text: String) -> PyErr {
    // PyO3 picks the subclass from the error's kind alone, so a new error of
    // the same kind finds it; an io::Error cannot be cloned.
    let class = PyErr::from(io::Error::from(error.kind())).get_type(py);
    let err = PyErr::from_type(class, text);
    if let Some(code) = error.raw_os_error()
        && let Err(failed) = err.value(py).setattr("errno", code)
    {
        // Setting an attribute fails only when Python is out of memory;
        // that error is the one to raise then.
        return failed;
    }
    err
}

/// The exception for a source that could not be built: ValueError for
/// dimensions that are not allowed, MemoryError for a synthetic source too
/// large to hold, OSError for a file that cannot be opened or is not whole
/// rows. The texts are the C++ relay's, typos included.
fn source_error(py: Python<'_>, err: SourceError) -> PyErr {
    let text = err.to_string();
    match &err {
        SourceError::NoChannels | SourceError::NoSyntheticDimensions => PyValueError::new_err(text),
        SourceError::SyntheticTooLarge { .. } => PyMemoryError::new_err(text),
        SourceError::Open { error, .. } | SourceError::Stat { error, .. } => {
            os_error(py, error, text)
        }
        SourceError::NotWholeRows { .. } => PyOSError::new_err(text),
    }
}

/// A replay dataset: `sample_count` rows of `channel_count` little-endian
/// u16 samples, read from a recorded file or generated as the identity
/// pattern, exactly as the relay serves it.
#[pyclass(frozen, module = "argus_sim_py")]
pub struct ReplaySource {
    /// Behind an `Arc` because the core's server takes its source that way,
    /// so a relay can serve this source while Python goes on reading it.
    inner: Arc<source::ReplaySource>,
}

impl ReplaySource {
    /// The core's source, shared with whoever else serves or reads it.
    pub(crate) fn shared(&self) -> Arc<source::ReplaySource> {
        Arc::clone(&self.inner)
    }
}

#[pymethods]
impl ReplaySource {
    /// Opens a replay file: little-endian u16 samples, sample-major, with no
    /// header, `channel_count` samples per row. The file is read on demand,
    /// not loaded.
    ///
    /// Raises ValueError when `channel_count` is 0, and OSError when the file
    /// cannot be opened (FileNotFoundError, PermissionError, ... with
    /// `.errno` set) or is empty or ends partway through a row.
    #[staticmethod]
    #[pyo3(
        signature = (path, channel_count = Unsigned(wire::MAX_CHANNELS)),
        text_signature = "(path, channel_count=96)"
    )]
    fn from_file(py: Python<'_>, path: PathBuf, channel_count: Unsigned<u16>) -> PyResult<Self> {
        let source = py
            .detach(|| source::ReplaySource::from_file(&path, channel_count.0))
            .map_err(|err| source_error(py, err))?;
        Ok(Self {
            inner: Arc::new(source),
        })
    }

    /// The identity pattern the fabric's self-test expects: `n_samples` rows
    /// of `channel_count` samples, each the chip (`channel // 32`, two bits)
    /// in bits 15..14, the channel within the chip in bits 13..8 (0 to 31,
    /// so bit 13 stays 0) and the sample index modulo 256 in bits 7..0.
    ///
    /// Raises ValueError when either is 0, and MemoryError when the rows
    /// cannot be held in memory.
    #[staticmethod]
    #[pyo3(
        signature = (n_samples, channel_count = Unsigned(wire::MAX_CHANNELS)),
        text_signature = "(n_samples, channel_count=96)"
    )]
    fn synthetic(
        py: Python<'_>,
        n_samples: Unsigned<u64>,
        channel_count: Unsigned<u16>,
    ) -> PyResult<Self> {
        let source = py
            .detach(|| source::ReplaySource::synthetic(n_samples.0, channel_count.0))
            .map_err(|err| source_error(py, err))?;
        Ok(Self {
            inner: Arc::new(source),
        })
    }

    /// The rows in the source; never 0.
    #[getter]
    fn sample_count(&self) -> u64 {
        self.inner.sample_count()
    }

    /// The samples in a row; never 0.
    #[getter]
    fn channel_count(&self) -> u16 {
        self.inner.channel_count()
    }

    /// `count` rows from row `sample_offset`, as bytes: the rows the relay
    /// would send for that request. The offset is taken modulo
    /// `sample_count` and the rows wrap at the end, as many times as it
    /// takes, so `count` may be more than the whole source.
    ///
    /// Raises ValueError for a negative argument, MemoryError when the rows
    /// would not fit in memory, and OSError when the file cannot be read
    /// (it shrank after it was opened, say).
    #[pyo3(
        signature = (sample_offset, count = Unsigned(u64::from(wire::SAMPLES_PER_HALF))),
        text_signature = "($self, sample_offset, count=147)"
    )]
    fn read<'py>(
        &self,
        py: Python<'py>,
        sample_offset: Unsigned<u64>,
        count: Unsigned<u64>,
    ) -> PyResult<Bound<'py, PyBytes>> {
        let source = &self.inner;
        // A Python bytes object holds at most isize::MAX bytes.
        let len = usize::try_from(count.0)
            .ok()
            .and_then(|rows| rows.checked_mul(source.row_bytes()))
            .filter(|&len| isize::try_from(len).is_ok())
            .ok_or_else(|| {
                PyMemoryError::new_err(format!(
                    "{} samples of {} channels do not fit in memory",
                    count.0,
                    source.channel_count()
                ))
            })?;
        // The rows go straight into the new bytes object, which no Python
        // code can see until it is returned, so the GIL need not be held
        // while they are read.
        PyBytes::new_with(py, len, |out| {
            py.detach(|| source.read_rows(sample_offset.0, out))
                .map_err(|err| {
                    let text = format!(
                        "read {} samples from {}: {}",
                        count.0,
                        sample_offset.0,
                        strerror(&err)
                    );
                    os_error(py, &err, text)
                })
        })
    }

    fn __repr__(&self) -> String {
        format!(
            "ReplaySource(sample_count={}, channel_count={})",
            self.inner.sample_count(),
            self.inner.channel_count()
        )
    }
}

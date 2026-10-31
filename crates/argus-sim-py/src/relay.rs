//! The relay for Python: `argus_sim_core::server` in the calling process.
//!
//! The same server the node runs, on its own thread, serving a
//! [`ReplaySource`] over UDP. Python starts and stops it, reads its counters,
//! and talks to it over a socket like the board does. Nothing of the server
//! runs in Python: once started, it answers requests whether or not Python
//! code holds the GIL.

use std::sync::{Mutex, MutexGuard, PoisonError};

use argus_sim_core::server::{Config, ReplayServer, ServerError};
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use pyo3::sync::MutexExt;
use pyo3::types::PyDict;

use crate::args::Unsigned;
use crate::source::{ReplaySource, os_error};

/// The exception for a relay that could not be built or started:
/// ValueError for a source or bind address it cannot serve, OSError (of the
/// subclass for the error, `.errno` set) when the socket cannot be made or
/// bound or the thread cannot be started. The texts are the C++ relay's.
fn server_error(py: Python<'_>, err: ServerError) -> PyErr {
    let text = err.to_string();
    match &err {
        ServerError::EmptySource
        | ServerError::ChannelsTooWide { .. }
        | ServerError::BadBindAddress(_) => PyValueError::new_err(text),
        ServerError::Socket(error)
        | ServerError::Bind { error, .. }
        | ServerError::Spawn(error) => os_error(py, error, text),
    }
}

/// The server, whether or not a thread panicked while holding it: a panic
/// in the core is a bug, but the server is still safe to stop and drop.
fn lock(server: &Mutex<ReplayServer>) -> MutexGuard<'_, ReplayServer> {
    server.lock().unwrap_or_else(PoisonError::into_inner)
}

/// A replay relay serving a ReplaySource over UDP, on its own thread: the
/// server the dataset_relay_node executable runs, answering requests byte
/// for byte as the C++ relay did.
#[pyclass(frozen, module = "argus_sim_py")]
pub struct Relay {
    /// Behind a mutex because Python may call into the relay from several
    /// threads, and start and stop change it. The lock is taken with the GIL
    /// released, so a thread waiting for it never holds up the others.
    ///
    /// A relay Python lets go of without stopping it is stopped as the
    /// server drops, with the GIL held: the server's thread never needs the
    /// GIL, so that wait (200 ms at most) cannot deadlock.
    server: Mutex<ReplayServer>,
}

#[pymethods]
impl Relay {
    /// A relay for `source`, not yet started. `port` 0 binds a port the
    /// kernel picks, so that two relays never collide; `.port` tells which.
    /// `loop` wraps every request at the end of the source (a request can
    /// also ask for that itself, with F_LOOP); bit N of `drop_mask` withholds
    /// chunk N of every first attempt, to exercise a client's retransmits.
    ///
    /// Raises ValueError when a row of the source does not fit one chunk, or
    /// an argument is out of range.
    #[new]
    #[pyo3(
        signature = (
            source,
            port = Unsigned(0),
            bind_addr = "127.0.0.1".to_owned(),
            r#loop = true,
            drop_mask = Unsigned(0),
        ),
        text_signature = "(source, port=0, bind_addr=\"127.0.0.1\", loop=True, drop_mask=0)"
    )]
    fn new(
        py: Python<'_>,
        source: &Bound<'_, ReplaySource>,
        port: Unsigned<u16>,
        bind_addr: String,
        r#loop: bool,
        drop_mask: Unsigned<u32>,
    ) -> PyResult<Self> {
        let config = Config {
            bind_addr,
            port: port.0,
            // The core reads port 0 as the board's port, as the node's
            // parameter did; from Python, 0 asks the kernel for one.
            any_port: port.0 == 0,
            looping: r#loop,
            drop_mask: drop_mask.0,
            max_samples_per_request: 0,
        };
        let server = ReplayServer::new(source.get().shared(), config)
            .map_err(|err| server_error(py, err))?;
        Ok(Self {
            server: Mutex::new(server),
        })
    }

    /// Binds the socket and starts the relay's thread; it answers requests
    /// from then on. Does nothing when the relay is already running. A
    /// relay that was stopped can be started again, on a new socket.
    ///
    /// Raises ValueError for a bind address that is not a dotted quad, and
    /// OSError when the socket cannot be bound (OSError with errno
    /// EADDRINUSE for a port in use) or the thread cannot be started.
    fn start(&self, py: Python<'_>) -> PyResult<()> {
        // Binding is quick, but other Python threads need not wait for it.
        py.detach(|| lock(&self.server).start())
            .map_err(|err| server_error(py, err))
    }

    /// Stops the relay's thread and closes its socket, so the port is free
    /// again when this returns. That takes up to 200 ms, the thread's
    /// receive timeout, with the GIL released. Does nothing when the relay
    /// is not running.
    fn stop(&self, py: Python<'_>) {
        py.detach(|| lock(&self.server).stop());
    }

    /// The UDP port the relay is bound to while it runs, None when it is
    /// stopped.
    #[getter]
    fn port(&self, py: Python<'_>) -> Option<u16> {
        let server = self
            .server
            .lock_py_attached(py)
            .unwrap_or_else(PoisonError::into_inner);
        server.local_addr().map(|addr| addr.port())
    }

    /// The sample rows in one chunk: as many as fit MAX_PAYLOAD bytes (7 at
    /// 96 channels).
    #[getter]
    fn samples_per_chunk(&self, py: Python<'_>) -> u16 {
        let server = self
            .server
            .lock_py_attached(py)
            .unwrap_or_else(PoisonError::into_inner);
        server.samples_per_chunk()
    }

    /// The relay's counters since it was made, as a dict: the requests
    /// served (retransmits included) and the retransmits among them, the
    /// chunks sent and withheld by `drop_mask`, the bytes sent, the
    /// malformed requests rejected, the requests that wrapped, the offset
    /// of the last request after any wrap, and the address of the last
    /// requester, None before the first. The counters go on across a stop
    /// and a start.
    fn stats<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyDict>> {
        let stats = self
            .server
            .lock_py_attached(py)
            .unwrap_or_else(PoisonError::into_inner)
            .stats();
        let dict = PyDict::new(py);
        dict.set_item("requests_served", stats.requests_served)?;
        dict.set_item("retransmits_served", stats.retransmits_served)?;
        dict.set_item("chunks_sent", stats.chunks_sent)?;
        dict.set_item("chunks_dropped", stats.chunks_dropped)?;
        dict.set_item("bytes_sent", stats.bytes_sent)?;
        dict.set_item("bad_packets", stats.bad_packets)?;
        dict.set_item("wraps", stats.wraps)?;
        dict.set_item("last_offset", stats.last_offset)?;
        dict.set_item("last_peer", stats.last_peer.map(|ip| ip.to_string()))?;
        Ok(dict)
    }

    /// Starts the relay; `with Relay(source) as relay:` serves inside the
    /// block.
    fn __enter__<'py>(slf: &Bound<'py, Self>) -> PyResult<Bound<'py, Self>> {
        slf.get().start(slf.py())?;
        Ok(slf.clone())
    }

    /// Stops the relay, however the block ended; an exception from the
    /// block goes on.
    fn __exit__(
        &self,
        py: Python<'_>,
        _exc_type: &Bound<'_, PyAny>,
        _exc_value: &Bound<'_, PyAny>,
        _traceback: &Bound<'_, PyAny>,
    ) -> bool {
        self.stop(py);
        false
    }

    fn __repr__(&self, py: Python<'_>) -> String {
        match self.port(py) {
            Some(port) => format!("Relay(port={port}, running)"),
            None => "Relay(stopped)".to_owned(),
        }
    }
}

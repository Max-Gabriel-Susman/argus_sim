//! Wire fields and other unsigned arguments taken from Python ints.
//!
//! PyO3 converts a Python int to a `u16` or `u32` on its own, but an int out
//! of range (negative, or too large for the field) then raises
//! `OverflowError`. Python code that checks arguments expects `ValueError`
//! for a value of the right type that is not allowed, so [`Unsigned`] turns
//! the one into the other. A value of the wrong type is still `TypeError`.

use pyo3::exceptions::{PyOverflowError, PyValueError};
use pyo3::prelude::*;

/// An unsigned integer type a wire field (or a count of samples) is made of.
pub trait Field: Copy {
    /// The largest value the field holds, for the error message.
    const MAX: u64;
}

impl Field for u16 {
    const MAX: u64 = u16::MAX as u64;
}

impl Field for u32 {
    const MAX: u64 = u32::MAX as u64;
}

impl Field for u64 {
    const MAX: u64 = u64::MAX;
}

/// A wire field from a Python int, refused with `ValueError` when it does not
/// fit. PyO3 adds the argument's name to the error as a note.
#[derive(Clone, Copy)]
pub struct Unsigned<T>(pub T);

impl<'a, 'py, T> FromPyObject<'a, 'py> for Unsigned<T>
where
    T: Field + FromPyObject<'a, 'py>,
{
    type Error = PyErr;

    fn extract(obj: Borrowed<'a, 'py, PyAny>) -> PyResult<Self> {
        let py = obj.py();
        T::extract(obj).map(Self).map_err(|err| {
            let err: PyErr = err.into();
            if err.is_instance_of::<PyOverflowError>(py) {
                PyValueError::new_err(format!("{} is not in 0..={}", *obj, T::MAX))
            } else {
                err
            }
        })
    }
}

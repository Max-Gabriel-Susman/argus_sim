//! The dataset the relay serves.
//!
//! A source is a sequence of rows, one row per sample, each row one
//! little-endian `u16` per channel. It is read from a recorded `.bin` file or
//! generated as a synthetic identity pattern, and read back in runs of rows
//! that wrap at the end, so a looping replay never runs out.
//!
//! The C++ relay mapped the file. This one reads it with `pread`
//! ([`FileExt::read_exact_at`]) straight into the transmit buffer: no unsafe
//! code, and the page cache keeps the file in memory as the mapping did. A
//! `pread` takes no file position, so the server thread and anyone else can
//! read one source at the same time through a shared reference.

use std::fmt;
use std::fs::File;
use std::io;
use std::os::unix::fs::FileExt;
use std::path::{Path, PathBuf};

use crate::os_error::strerror;

/// Bytes per sample code: a little-endian `u16`.
const CODE_BYTES: usize = size_of::<u16>();

/// Why a source could not be built. The texts are the C++ relay's, typos
/// included, because the node prints them as its FATAL line.
#[derive(Debug)]
pub enum SourceError {
    /// A file source with zero channels.
    NoChannels,
    /// The file could not be opened.
    Open {
        /// The path as given.
        path: PathBuf,
        /// What `open` reported.
        error: io::Error,
    },
    /// The file's size could not be read.
    Stat {
        /// The path as given.
        path: PathBuf,
        /// What `fstat` reported.
        error: io::Error,
    },
    /// The file is empty, or its size is not a whole number of rows.
    NotWholeRows {
        /// The path as given.
        path: PathBuf,
        /// The file's size.
        bytes: u64,
        /// The bytes in one row.
        row_bytes: usize,
        /// The channels in one row.
        channels: u16,
    },
    /// A synthetic source with zero samples or zero channels.
    NoSyntheticDimensions,
    /// A synthetic source too large to hold in memory. The C++ died here
    /// with the C++ library's allocation message instead.
    SyntheticTooLarge {
        /// The samples asked for.
        samples: u64,
        /// The channels asked for.
        channels: u16,
    },
}

impl fmt::Display for SourceError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NoChannels => f.write_str("channel_count must be positive"),
            Self::Open { path, error } => {
                write!(f, "open {}: {}", path.display(), strerror(error))
            }
            Self::Stat { path, error } => {
                write!(f, "fstat {}: {}", path.display(), strerror(error))
            }
            // "multiples" is the C++ text; kept for parity.
            Self::NotWholeRows {
                path,
                bytes,
                row_bytes,
                channels,
            } => write!(
                f,
                "{}: {bytes} bytes is not a multiples of {row_bytes} ({channels} channels)",
                path.display()
            ),
            // "sythetic" is the C++ text; kept for parity.
            Self::NoSyntheticDimensions => f.write_str("sythetic source needs positive dimensions"),
            Self::SyntheticTooLarge { samples, channels } => write!(
                f,
                "synthetic source of {samples} samples x {channels} channels does not fit in memory"
            ),
        }
    }
}

impl std::error::Error for SourceError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Open { error, .. } | Self::Stat { error, .. } => Some(error),
            _ => None,
        }
    }
}

/// Where the rows live.
#[derive(Debug)]
enum Rows {
    /// A recorded file, read with `pread` on demand.
    File(File),
    /// Rows generated up front, as the C++ did.
    Memory(Vec<u8>),
}

/// A dataset of `sample_count()` rows of `channel_count()` little-endian
/// `u16` codes, sample-major, with no header.
#[derive(Debug)]
pub struct ReplaySource {
    rows: Rows,
    sample_count: u64,
    channel_count: u16,
}

impl ReplaySource {
    /// Opens a replay file: little-endian `u16` codes, sample-major, with no
    /// header, `channel_count` codes per row.
    ///
    /// # Errors
    ///
    /// [`SourceError::NoChannels`] when `channel_count` is 0;
    /// [`SourceError::Open`] or [`SourceError::Stat`] when the file cannot be
    /// opened or sized; [`SourceError::NotWholeRows`] when it is empty or
    /// ends partway through a row.
    pub fn from_file(path: impl AsRef<Path>, channel_count: u16) -> Result<Self, SourceError> {
        let path = path.as_ref();
        if channel_count == 0 {
            return Err(SourceError::NoChannels);
        }
        let file = File::open(path).map_err(|error| SourceError::Open {
            path: path.to_owned(),
            error,
        })?;
        let bytes = file
            .metadata()
            .map_err(|error| SourceError::Stat {
                path: path.to_owned(),
                error,
            })?
            .len();

        let row_bytes = usize::from(channel_count) * CODE_BYTES;
        // A u16 channel count makes a row at most 128 KiB, which fits a u64.
        let row = row_bytes as u64;
        if bytes == 0 || !bytes.is_multiple_of(row) {
            return Err(SourceError::NotWholeRows {
                path: path.to_owned(),
                bytes,
                row_bytes,
                channels: channel_count,
            });
        }
        Ok(Self {
            rows: Rows::File(file),
            sample_count: bytes / row,
            channel_count,
        })
    }

    /// Builds the identity pattern the fabric's self-test expects:
    /// `n_samples` rows of `channel_count` codes, each code [`identity`] of
    /// its sample and channel. A chunk delivered out of order or at the wrong
    /// offset is then visible in the data itself.
    ///
    /// # Errors
    ///
    /// [`SourceError::NoSyntheticDimensions`] when either is 0, and
    /// [`SourceError::SyntheticTooLarge`] when the rows cannot be allocated.
    pub fn synthetic(n_samples: u64, channel_count: u16) -> Result<Self, SourceError> {
        if channel_count == 0 || n_samples == 0 {
            return Err(SourceError::NoSyntheticDimensions);
        }
        let too_large = || SourceError::SyntheticTooLarge {
            samples: n_samples,
            channels: channel_count,
        };
        let len = usize::try_from(n_samples)
            .ok()
            .and_then(|n| n.checked_mul(usize::from(channel_count) * CODE_BYTES))
            .ok_or_else(too_large)?;
        // Reserve fallibly, so an absurd size is an error and not an abort.
        let mut data = Vec::new();
        data.try_reserve_exact(len).map_err(|_| too_large())?;
        for sample in 0..n_samples {
            for channel in 0..channel_count {
                data.extend_from_slice(&identity(sample, channel).to_le_bytes());
            }
        }
        Ok(Self {
            rows: Rows::Memory(data),
            sample_count: n_samples,
            channel_count,
        })
    }

    /// Rows in the source; never 0.
    #[must_use]
    pub fn sample_count(&self) -> u64 {
        self.sample_count
    }

    /// Codes per row; never 0.
    #[must_use]
    pub fn channel_count(&self) -> u16 {
        self.channel_count
    }

    /// Bytes per row: two per channel.
    #[must_use]
    pub fn row_bytes(&self) -> usize {
        usize::from(self.channel_count) * CODE_BYTES
    }

    /// Fills `out` with whole rows, row `i` of `out` being row
    /// `(first + i) mod sample_count()` of the source. A read that runs past
    /// the end continues from the start, as many times as it takes, so `out`
    /// may be longer than the whole source.
    ///
    /// Each contiguous run of rows is one read: at most two for a read
    /// shorter than the source, which is what the server makes.
    ///
    /// # Errors
    ///
    /// `InvalidInput` when `out` is not a whole number of rows, and whatever
    /// `pread` reports for a file source (`UnexpectedEof` if the file shrank
    /// after it was opened). `out` is then partly written.
    pub fn read_rows(&self, first: u64, out: &mut [u8]) -> io::Result<()> {
        let row_bytes = self.row_bytes();
        if !out.len().is_multiple_of(row_bytes) {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "buffer is not a whole number of rows",
            ));
        }

        let mut row = first % self.sample_count;
        let mut rest = out;
        while !rest.is_empty() {
            // The rows left before the end of the source, as bytes. Capped by
            // what is left to fill, so it fits a usize.
            let to_end = (self.sample_count - row).saturating_mul(row_bytes as u64);
            let run = usize::try_from(to_end).map_or(rest.len(), |b| b.min(rest.len()));
            let (head, tail) = rest.split_at_mut(run);
            self.read_run(row * row_bytes as u64, head)?;
            rest = tail;
            // A run either fills `out` or reaches the end of the source, so
            // the next one, if any, starts from the first row.
            row = 0;
        }
        Ok(())
    }

    /// Reads `out.len()` contiguous bytes from byte `at` of the rows.
    fn read_run(&self, at: u64, out: &mut [u8]) -> io::Result<()> {
        match &self.rows {
            Rows::File(file) => file.read_exact_at(out, at),
            Rows::Memory(data) => {
                let src = usize::try_from(at)
                    .ok()
                    .and_then(|start| data.get(start..start.checked_add(out.len())?))
                    .ok_or_else(|| io::Error::from(io::ErrorKind::UnexpectedEof))?;
                out.copy_from_slice(src);
                Ok(())
            }
        }
    }
}

/// The identity pattern's code for `channel` of `sample`: bits 15..14 the
/// chip (`channel / 32`, in two bits, so channel 128 is chip 0 again),
/// bits 12..8 the channel within the chip, bits 7..0 the sample modulo 256.
#[must_use]
pub fn identity(sample: u64, channel: u16) -> u16 {
    let chip = (channel / 32) & 0x3;
    let in_chip = channel % 32;
    let [index, ..] = sample.to_le_bytes();
    (chip << 14) | (in_chip << 8) | u16::from(index)
}

#[cfg(test)]
mod tests {
    use super::*;

    use std::sync::atomic::{AtomicU32, Ordering};

    type TestResult = Result<(), Box<dyn std::error::Error>>;

    /// A file in the temporary directory, removed when dropped.
    struct TempFile(PathBuf);

    impl TempFile {
        /// A new file holding `bytes`, named after the process and a counter
        /// so tests running in parallel never share one.
        fn with(bytes: &[u8]) -> io::Result<Self> {
            static NEXT: AtomicU32 = AtomicU32::new(0);
            let name = format!(
                "argus-sim-source-{}-{}.bin",
                std::process::id(),
                NEXT.fetch_add(1, Ordering::Relaxed)
            );
            let path = std::env::temp_dir().join(name);
            std::fs::write(&path, bytes)?;
            Ok(Self(path))
        }
    }

    impl Drop for TempFile {
        fn drop(&mut self) {
            let _ = std::fs::remove_file(&self.0);
        }
    }

    /// The code `(sample, channel)` for a test file: distinct for every pair
    /// up to 4096 samples and 16 channels, and not the identity pattern, so a
    /// file read is not mistaken for a synthetic one.
    fn file_code(sample: u64, channel: u16) -> u16 {
        let code = (sample % 4096) | (u64::from(channel % 16) << 12);
        u16::try_from(code).unwrap_or(u16::MAX) ^ 0x5A5A
    }

    /// `samples` rows of `channels` codes of [`file_code`], as bytes.
    fn file_rows(first: u64, samples: u64, channels: u16) -> Vec<u8> {
        (first..first + samples)
            .flat_map(|s| (0..channels).flat_map(move |c| file_code(s, c).to_le_bytes()))
            .collect()
    }

    /// The codes of `bytes`, which holds little-endian `u16`s.
    fn codes(bytes: &[u8]) -> Vec<u16> {
        bytes
            .chunks_exact(2)
            .filter_map(|pair| pair.try_into().ok().map(u16::from_le_bytes))
            .collect()
    }

    #[test]
    fn the_pattern_packs_chip_channel_and_sample() {
        // Chip boundaries at channels 31 and 32, the last channel at 96, and
        // the sample index wrapping its byte between 255 and 256.
        assert_eq!(identity(255, 31), 0x1FFF);
        assert_eq!(identity(256, 31), 0x1F00);
        assert_eq!(identity(255, 32), 0x40FF);
        assert_eq!(identity(256, 32), 0x4000);
        assert_eq!(identity(255, 95), 0x9FFF);
        assert_eq!(identity(256, 95), 0x9F00);
        assert_eq!(identity(0, 0), 0x0000);
        // Two bits of chip, as the C++ masked them: channel 128 is chip 0.
        assert_eq!(identity(1, 128), identity(1, 0));
    }

    #[test]
    fn a_synthetic_source_holds_the_pattern() -> TestResult {
        let source = ReplaySource::synthetic(300, 96)?;
        assert_eq!(source.sample_count(), 300);
        assert_eq!(source.channel_count(), 96);
        assert_eq!(source.row_bytes(), 192);
        for sample in [0, 255, 256, 299] {
            let mut row = vec![0u8; source.row_bytes()];
            source.read_rows(sample, &mut row)?;
            let row = codes(&row);
            for channel in [0u16, 31, 32, 95] {
                assert_eq!(
                    row.get(usize::from(channel)).copied(),
                    Some(identity(sample, channel)),
                    "sample {sample} channel {channel}"
                );
            }
        }
        Ok(())
    }

    #[test]
    fn a_file_reads_back_as_written() -> TestResult {
        let file = TempFile::with(&file_rows(0, 50, 3))?;
        let source = ReplaySource::from_file(&file.0, 3)?;
        assert_eq!(source.sample_count(), 50);
        assert_eq!(source.channel_count(), 3);
        assert_eq!(source.row_bytes(), 6);

        let mut out = vec![0u8; 10 * 6];
        source.read_rows(17, &mut out)?;
        assert_eq!(out, file_rows(17, 10, 3));
        Ok(())
    }

    #[test]
    fn a_read_wraps_at_the_end() -> TestResult {
        let file = TempFile::with(&file_rows(0, 50, 3))?;
        let source = ReplaySource::from_file(&file.0, 3)?;

        // Rows 47, 48, 49 and then 0 to 3.
        let mut out = vec![0u8; 7 * 6];
        source.read_rows(47, &mut out)?;
        let mut want = file_rows(47, 3, 3);
        want.extend(file_rows(0, 4, 3));
        assert_eq!(out, want);

        // A first row past the end is taken modulo the length.
        let mut again = vec![0u8; 7 * 6];
        source.read_rows(47 + 2 * 50, &mut again)?;
        assert_eq!(again, want);
        Ok(())
    }

    #[test]
    fn a_read_longer_than_the_source_goes_round_again() -> TestResult {
        let file = TempFile::with(&file_rows(0, 5, 2))?;
        let source = ReplaySource::from_file(&file.0, 2)?;
        let synthetic = ReplaySource::synthetic(5, 2)?;

        // Twelve rows of five from row 3: 3, 4, 0..5, 0..5.
        let mut want = file_rows(3, 2, 2);
        want.extend(file_rows(0, 5, 2));
        want.extend(file_rows(0, 5, 2));
        let mut out = vec![0u8; 12 * 4];
        source.read_rows(3, &mut out)?;
        assert_eq!(out, want);

        // The synthetic source wraps the same way.
        synthetic.read_rows(3, &mut out)?;
        let want: Vec<u16> = [3, 4, 0, 1, 2, 3, 4, 0, 1, 2, 3, 4]
            .into_iter()
            .flat_map(|s| [identity(s, 0), identity(s, 1)])
            .collect();
        assert_eq!(codes(&out), want);
        Ok(())
    }

    #[test]
    fn an_empty_read_reads_nothing() -> TestResult {
        let source = ReplaySource::synthetic(5, 2)?;
        source.read_rows(4, &mut [])?;
        Ok(())
    }

    #[test]
    fn a_read_of_part_of_a_row_is_refused() -> TestResult {
        let source = ReplaySource::synthetic(5, 2)?;
        let err = source
            .read_rows(0, &mut [0u8; 6])
            .err()
            .ok_or("a partial row was read")?;
        assert_eq!(err.kind(), io::ErrorKind::InvalidInput);
        Ok(())
    }

    #[test]
    fn an_empty_file_is_refused() -> TestResult {
        let file = TempFile::with(&[])?;
        let err = ReplaySource::from_file(&file.0, 96)
            .err()
            .ok_or("an empty file was taken")?;
        assert_eq!(
            err.to_string(),
            format!(
                "{}: 0 bytes is not a multiples of 192 (96 channels)",
                file.0.display()
            )
        );
        Ok(())
    }

    #[test]
    fn a_partial_row_at_the_end_is_refused() -> TestResult {
        let file = TempFile::with(&[0u8; 192 * 3 + 2])?;
        let err = ReplaySource::from_file(&file.0, 96)
            .err()
            .ok_or("a partial row was taken")?;
        assert_eq!(
            err.to_string(),
            format!(
                "{}: 578 bytes is not a multiples of 192 (96 channels)",
                file.0.display()
            )
        );
        Ok(())
    }

    #[test]
    fn a_missing_file_is_refused_with_strerror() -> TestResult {
        let path = std::env::temp_dir().join("argus-sim-source-no-such-file.bin");
        let err = ReplaySource::from_file(&path, 96)
            .err()
            .ok_or("a missing file was opened")?;
        assert_eq!(
            err.to_string(),
            format!("open {}: No such file or directory", path.display())
        );
        assert!(matches!(err, SourceError::Open { .. }));
        assert!(std::error::Error::source(&err).is_some());
        Ok(())
    }

    #[test]
    fn zero_channels_are_refused() -> TestResult {
        // The file check comes before the open, as in the C++, so the path
        // need not exist.
        let err = ReplaySource::from_file("no-such-file.bin", 0)
            .err()
            .ok_or("zero channels were taken")?;
        assert_eq!(err.to_string(), "channel_count must be positive");

        let err = ReplaySource::synthetic(30_000, 0)
            .err()
            .ok_or("zero channels were taken")?;
        assert_eq!(err.to_string(), "sythetic source needs positive dimensions");
        Ok(())
    }

    #[test]
    fn zero_synthetic_samples_are_refused() -> TestResult {
        let err = ReplaySource::synthetic(0, 96)
            .err()
            .ok_or("zero samples were taken")?;
        assert_eq!(err.to_string(), "sythetic source needs positive dimensions");
        Ok(())
    }

    #[test]
    fn an_impossible_synthetic_size_is_an_error_not_an_abort() -> TestResult {
        let err = ReplaySource::synthetic(u64::MAX, 4096)
            .err()
            .ok_or("an impossible size was allocated")?;
        assert!(matches!(err, SourceError::SyntheticTooLarge { .. }));
        Ok(())
    }
}

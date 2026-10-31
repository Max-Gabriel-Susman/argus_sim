"""The dataset source: the identity pattern, file reads and every error.

The references here are numpy, independent of the bindings: the identity
pattern written out from the C++ relay's formula, and test files written and
read back with numpy (`np.fromfile`), so a row read from the wrong place or
in the wrong byte order shows.
"""

import errno
import os
import pathlib

import numpy as np
import pytest

import argus_sim_py as asp

# A sample on the wire: a little-endian u16.
CODE = np.dtype("<u2")


def identity(n_samples, channels):
    """The identity pattern, as the C++ relay built it: the chip in bits
    15..14, the channel within the chip in bits 12..8, the sample's low byte
    in bits 7..0."""
    sample = np.arange(n_samples, dtype=np.uint64)[:, None]
    channel = np.arange(channels, dtype=np.uint64)[None, :]
    chip = (channel // 32) & 0x3
    code = (chip << 14) | ((channel % 32) << 8) | (sample & 0xFF)
    return code.astype(CODE)


def as_rows(data, channels):
    """`data`, read from the bindings, as an array of rows."""
    return np.frombuffer(data, dtype=CODE).reshape(-1, channels)


@pytest.fixture
def recorded(tmp_path):
    """A recorded file of 500 rows of 12 channels of random codes, and the
    rows as numpy reads them back."""
    rng = np.random.default_rng(20161005)
    path = tmp_path / "recorded.bin"
    rng.integers(0, 1 << 16, size=(500, 12), dtype=np.uint16).astype(CODE).tofile(path)
    return path, np.fromfile(path, dtype=CODE).reshape(-1, 12)


def test_the_synthetic_source_is_the_identity_pattern():
    source = asp.ReplaySource.synthetic(600)
    assert source.sample_count == 600
    assert source.channel_count == 96
    rows = as_rows(source.read(0, 600), 96)
    np.testing.assert_array_equal(rows, identity(600, 96))
    # The corners of the pattern, written out: chip boundaries at channels
    # 31 and 32, the sample byte wrapping between 255 and 256.
    assert rows[255, 31] == 0x1FFF
    assert rows[256, 32] == 0x4000
    assert rows[255, 95] == 0x9FFF


def test_the_identity_pattern_masks_the_chip_to_two_bits():
    # Channel 128 is chip 0 again, as the C++ masked it.
    rows = as_rows(asp.ReplaySource.synthetic(3, 160).read(0, 3), 160)
    np.testing.assert_array_equal(rows, identity(3, 160))
    np.testing.assert_array_equal(rows[:, 128:], rows[:, :32])


def test_a_synthetic_read_wraps_at_the_end():
    source = asp.ReplaySource.synthetic(30000)
    want = identity(30000, 96)
    rows = as_rows(source.read(30000 - 73, 147), 96)
    np.testing.assert_array_equal(rows, np.concatenate([want[-73:], want[:74]]))


def test_the_default_read_is_half_a_buffer():
    source = asp.ReplaySource.synthetic(1000)
    assert len(source.read(10)) == asp.SAMPLES_PER_HALF * 96 * 2
    assert source.read(10) == source.read(10, 147)


def test_a_file_reads_back_as_numpy_reads_it(recorded):
    path, want = recorded
    source = asp.ReplaySource.from_file(path, 12)
    assert source.sample_count == 500
    assert source.channel_count == 12
    np.testing.assert_array_equal(as_rows(source.read(0, 500), 12), want)
    np.testing.assert_array_equal(as_rows(source.read(123, 45), 12), want[123:168])


def test_a_file_read_wraps_at_the_end(recorded):
    path, want = recorded
    source = asp.ReplaySource.from_file(path, 12)
    rows = as_rows(source.read(490, 30), 12)
    np.testing.assert_array_equal(rows, np.concatenate([want[490:], want[:20]]))
    # An offset past the end is taken modulo the length.
    assert source.read(490 + 3 * 500, 30) == source.read(490, 30)


def test_a_read_longer_than_the_source_goes_round_again(recorded):
    path, want = recorded
    source = asp.ReplaySource.from_file(path, 12)
    rows = as_rows(source.read(321, 1234), 12)
    np.testing.assert_array_equal(rows, want[(321 + np.arange(1234)) % 500])

    synthetic = asp.ReplaySource.synthetic(5, 2)
    rows = as_rows(synthetic.read(3, 12), 2)
    np.testing.assert_array_equal(rows, identity(5, 2)[[3, 4, 0, 1, 2, 3, 4, 0, 1, 2, 3, 4]])


def test_an_empty_read_is_empty():
    assert asp.ReplaySource.synthetic(5).read(4, 0) == b""


def test_the_real_layout_reads_like_the_tools_read_it(tmp_path):
    # 96 channels, as the recorded datasets are, read the way decode_test.py
    # and hw_bitexact.py read them: a flat u16 array, reshaped to rows.
    rng = np.random.default_rng(7)
    path = tmp_path / "ninety_six.bin"
    rng.integers(0, 1 << 16, size=300 * 96, dtype=np.uint16).astype(CODE).tofile(path)
    want = np.fromfile(path, dtype=CODE).reshape(-1, 96)
    source = asp.ReplaySource.from_file(str(path))
    assert source.channel_count == 96
    rows = as_rows(source.read(300 - 7, 147), 96)
    np.testing.assert_array_equal(rows, want[(293 + np.arange(147)) % 300])


def test_a_path_may_be_a_string_or_a_path(recorded):
    path, _ = recorded
    assert asp.ReplaySource.from_file(str(path), 12).read(0, 1) == \
        asp.ReplaySource.from_file(pathlib.Path(path), 12).read(0, 1)


def test_the_source_is_read_only(recorded):
    path, _ = recorded
    source = asp.ReplaySource.from_file(path, 12)
    with pytest.raises(AttributeError):
        source.sample_count = 1
    assert repr(source) == "ReplaySource(sample_count=500, channel_count=12)"


def test_a_missing_file_is_file_not_found_with_the_c_text(tmp_path):
    path = tmp_path / "no-such-file.bin"
    with pytest.raises(FileNotFoundError) as caught:
        asp.ReplaySource.from_file(path)
    assert str(caught.value) == f"open {path}: No such file or directory"
    assert caught.value.errno == errno.ENOENT


@pytest.mark.skipif(os.geteuid() == 0, reason="root opens any file")
def test_an_unreadable_file_is_permission_error(tmp_path):
    path = tmp_path / "unreadable.bin"
    path.write_bytes(bytes(192))
    path.chmod(0)
    with pytest.raises(PermissionError) as caught:
        asp.ReplaySource.from_file(path)
    assert str(caught.value) == f"open {path}: Permission denied"
    assert caught.value.errno == errno.EACCES


def test_an_empty_file_is_refused_with_the_c_text(tmp_path):
    path = tmp_path / "empty.bin"
    path.write_bytes(b"")
    with pytest.raises(OSError) as caught:
        asp.ReplaySource.from_file(path)
    # "multiples" is the C++ text, kept for parity.
    assert str(caught.value) == f"{path}: 0 bytes is not a multiples of 192 (96 channels)"


def test_a_partial_row_is_refused_with_the_c_text(tmp_path):
    path = tmp_path / "partial.bin"
    path.write_bytes(bytes(192 * 3 + 2))
    with pytest.raises(OSError) as caught:
        asp.ReplaySource.from_file(path)
    assert str(caught.value) == f"{path}: 578 bytes is not a multiples of 192 (96 channels)"
    # The same bytes are whole rows of one channel.
    assert asp.ReplaySource.from_file(path, 1).sample_count == 289


def test_a_file_that_shrinks_after_opening_fails_to_read(recorded):
    path, _ = recorded
    source = asp.ReplaySource.from_file(path, 12)
    os.truncate(path, 24 * 100)
    assert len(source.read(0, 100)) == 24 * 100
    with pytest.raises(OSError, match=r"^read 10 samples from 495: "):
        source.read(495, 10)


def test_zero_channels_are_value_errors_with_the_c_texts(tmp_path):
    # The channel check comes before the open, as in the C++.
    with pytest.raises(ValueError, match=r"^channel_count must be positive$"):
        asp.ReplaySource.from_file(tmp_path / "no-such-file.bin", 0)
    # "sythetic" is the C++ text, kept for parity.
    with pytest.raises(ValueError, match=r"^sythetic source needs positive dimensions$"):
        asp.ReplaySource.synthetic(30000, 0)
    with pytest.raises(ValueError, match=r"^sythetic source needs positive dimensions$"):
        asp.ReplaySource.synthetic(0)


def test_an_impossible_synthetic_source_is_a_memory_error():
    with pytest.raises(MemoryError):
        asp.ReplaySource.synthetic(2**64 - 1, 4096)


def test_an_impossible_read_is_a_memory_error():
    with pytest.raises(MemoryError):
        asp.ReplaySource.synthetic(5).read(0, 2**64 - 1)


@pytest.mark.parametrize("call", [
    lambda: asp.ReplaySource.synthetic(-1),
    lambda: asp.ReplaySource.synthetic(5, -1),
    lambda: asp.ReplaySource.synthetic(5, 1 << 16),
    lambda: asp.ReplaySource.synthetic(2**64),
    lambda: asp.ReplaySource.synthetic(5).read(-1),
    lambda: asp.ReplaySource.synthetic(5).read(0, -1),
])
def test_an_argument_out_of_range_is_a_value_error(call):
    with pytest.raises(ValueError):
        call()


@pytest.mark.parametrize("call", [
    lambda: asp.ReplaySource.synthetic("5"),
    lambda: asp.ReplaySource.synthetic(5.0),
    lambda: asp.ReplaySource.synthetic(5).read(None),
    lambda: asp.ReplaySource.from_file(5),
])
def test_an_argument_of_the_wrong_type_is_a_type_error(call):
    with pytest.raises(TypeError):
        call()


def test_a_source_cannot_be_built_directly():
    with pytest.raises(TypeError):
        asp.ReplaySource()

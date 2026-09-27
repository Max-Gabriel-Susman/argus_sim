#!/usr/bin/env python3
"""
nwb_to_replay.py -- Sabes/O'Doherty broadband NWB -> dataset_relay_node file.

Produces what ReplaySource::from_file() mmaps: a headerless file of
little-endian uint16, sample-major, one row per sample, CHANNELS columns.
Values are RHD2132 ADC codes -- offset binary, 0x8000 = 0 V at the
electrode, 0.195 uV per LSB, +/-6.39 mV full scale -- so what the simulated
chips return over SPI is what real silicon would have returned for the same
electrode voltage.

Between the NWB and the file:

  1. Segment.  --start / --seconds select a window; the whole session is
     minutes long at 24.4 kHz x 96 ch and the relay loops anyway.
  2. AC-couple.  The recording is unfiltered below 7.5 kHz, so it carries
     each electrode's DC offset, which would sit at the rails of a 6.39 mV
     range. Per-channel mean removal, then a first-order high-pass at
     --hp-hz, is what the RHD2132's analog coupling does to a real
     electrode.
  3. Resample.  24414.0625 Hz -> --target-hz (the fabric sweeps at
     30012 Hz: 125 MHz / 35 slots / 119 clocks). Polyphase, so spike
     widths and filter cutoffs are correct in the fabric's time base.
     --no-resample plays the recording 1.23x fast instead.
  4. Quantise.  volts / 0.195e-6 + 32768, clipped, with the clipped
     fraction reported -- a nonzero number means --gain is too high or the
     conversion attribute in the NWB is not what we assumed.

Memory: the segment is processed in --chunk-seconds pieces, two passes
(one for the per-channel mean, one to filter and write), with the filter
state carried across chunk boundaries, so a 200 s segment costs the same
RAM as a 10 s one. Resampling is the exception: resample_poly has no
carried state, so with resampling on the segment is loaded whole and the
script says how much memory that is before it does.

Reads /acquisition/timeseries/broadband/{data,timestamps}; NWB 1.0.6 HDF5,
the same h5py path inference_node.py uses. The data array is k x n integer
codes with a `conversion` attribute to volts.

    # for the relay: 10 s at the fabric rate
    python3 nwb_to_replay.py indy_20161005_06_broadband.nwb \
        --start 120 --seconds 10 \
        --out ~/argus_data/indy_20161005_06_s120_10s.bin

    # for decode_test.py: the whole overlap with the .mat, native rate
    python3 nwb_to_replay.py indy_20161005_06_broadband.nwb \
        --start 10 --seconds 199 --no-resample \
        --out ~/argus_data/indy_20161005_06_s10_199s_24k.bin
"""

import argparse
import fractions
import os
import sys

import h5py
import numpy as np
from scipy import signal

RHD_LSB_V = 0.195e-6           # RHD2132 ADC step referred to the electrode
RHD_ZERO = 32768               # offset-binary code for 0 V
RHD_FULL = 65535
CHANNELS = 96                  # ARGUS_MAX_CHANNELS; the relay rejects other widths

DEFAULT_DATA = "/acquisition/timeseries/broadband/data"
DEFAULT_TIMESTAMPS = "/acquisition/timeseries/broadband/timestamps"


def parse_args():
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("nwb", help="broadband supplement, NWB 1.0.6 HDF5")
    p.add_argument("--out", required=True, help="output .bin for dataset_relay_node")
    p.add_argument("--start", type=float, default=0.0,
                   help="segment start, seconds into the recording (default 0)")
    p.add_argument("--seconds", type=float, default=10.0,
                   help="segment length in seconds (default 10)")
    p.add_argument("--channels", type=int, default=CHANNELS,
                   help=f"channels to keep, from the first (default {CHANNELS})")
    p.add_argument("--hp-hz", type=float, default=0.5,
                   help="first-order high-pass cutoff after mean removal; "
                        "0 disables and leaves mean removal only (default 0.5)")
    p.add_argument("--target-hz", type=float, default=30012.0,
                   help="output sample rate (default 30012, the fabric sweep rate)")
    p.add_argument("--no-resample", action="store_true",
                   help="keep the recording's own rate (required for chunked processing)")
    p.add_argument("--gain", type=float, default=1.0,
                   help="extra scale applied before quantising (default 1.0)")
    p.add_argument("--chunk-seconds", type=float, default=10.0,
                   help="chunk length for the two-pass path (default 10)")
    p.add_argument("--data", default=DEFAULT_DATA, help="HDF5 path of the sample array")
    p.add_argument("--timestamps", default=DEFAULT_TIMESTAMPS,
                   help="HDF5 path of the per-sample timestamps")
    return p.parse_args()


class Source:
    """The segment, as a handle: row range, rate, conversion. Reads are
    slices of the HDF5 dataset, so nothing is loaded until asked for."""

    def __init__(self, args):
        self.f = h5py.File(args.nwb, "r")
        if args.data not in self.f:
            self.f.close()
            sys.exit(f"{args.data} not found. Top-level groups: {list(self.f.keys())}")
        self.d = self.f[args.data]
        ts = self.f[args.timestamps]

        self.k, self.n = self.d.shape
        self.conversion = float(self.d.attrs.get("conversion", 1.0))
        unit = self.d.attrs.get("unit", b"?")
        self.unit = unit.decode() if isinstance(unit, bytes) else str(unit)

        probe = np.asarray(ts[: min(self.k, 100000)], dtype=np.float64)
        self.fs = 1.0 / np.median(np.diff(probe))
        self.t0 = float(probe[0])

        print(f"{args.nwb}")
        print(f"  {self.k} samples x {self.n} channels, {self.d.dtype}, "
              f"{self.k / self.fs:.1f} s at {self.fs:.4f} Hz")
        print(f"  conversion={self.conversion:g} unit={self.unit}  t0={self.t0:.3f} s")

        if self.n < args.channels:
            sys.exit(f"only {self.n} channels in the file; --channels {args.channels} requested")
        self.nch = args.channels

        self.i0 = int(round(args.start * self.fs))
        self.i1 = self.i0 + int(round(args.seconds * self.fs))
        if self.i0 < 0 or self.i1 > self.k:
            sys.exit(f"segment [{self.i0}, {self.i1}) is outside the {self.k}-sample recording")
        self.rows = self.i1 - self.i0
        print(f"  segment {args.start:.2f}..{args.start + args.seconds:.2f} s into the recording "
              f"= t {self.t0 + args.start:.2f}..{self.t0 + args.start + args.seconds:.2f} s, "
              f"{self.rows} rows")

    def chunks(self, chunk_rows):
        for a in range(self.i0, self.i1, chunk_rows):
            b = min(a + chunk_rows, self.i1)
            yield np.asarray(self.d[a:b, : self.nch], dtype=np.float64) * self.conversion

    def close(self):
        self.f.close()


def channel_means(src, chunk_rows):
    acc = np.zeros(src.nch, dtype=np.float64)
    for x in src.chunks(chunk_rows):
        acc += x.sum(axis=0)
    return acc / src.rows


def hpf_sos(hp_hz, fs, nch):
    if hp_hz <= 0:
        return None, None
    sos = signal.butter(1, hp_hz, btype="highpass", fs=fs, output="sos")
    zi = np.zeros((sos.shape[0], 2, nch))    # mean is already gone: start at rest
    return sos, zi


def quantise(x, gain):
    codes = np.rint(x * gain / RHD_LSB_V) + RHD_ZERO
    clipped = int(np.count_nonzero((codes < 0) | (codes > RHD_FULL)))
    return np.clip(codes, 0, RHD_FULL).astype("<u2"), clipped


class Stats:
    def __init__(self, nch):
        self.sumsq = np.zeros(nch)
        self.peak = 0.0
        self.rows = 0
        self.clipped = 0

    def add(self, x, clipped):
        self.sumsq += np.sum(x * x, axis=0)
        self.peak = max(self.peak, float(np.max(np.abs(x))))
        self.rows += x.shape[0]
        self.clipped += clipped

    def report(self, nch, out_fs):
        rms_uv = np.sqrt(self.sumsq / max(self.rows, 1)) * 1e6
        total = self.rows * nch
        print(f"  after AC coupling: RMS {rms_uv.min():.1f}..{rms_uv.max():.1f} uV "
              f"(median {np.median(rms_uv):.1f}), peak {self.peak * 1e6:.0f} uV")
        print(f"  quantised: {self.rows} samples x {nch} ch, "
              f"clipped {self.clipped}/{total} ({100 * self.clipped / max(total, 1):.4f}%)")
        if self.clipped:
            print("  WARNING: clipping. Real spikes are 50-500 uV; if RMS above is "
                  "in the mV range the conversion attribute is probably wrong.")


def run_chunked(src, args, out):
    """Two passes over the segment, fixed memory. No resampling."""
    chunk_rows = max(1, int(round(args.chunk_seconds * src.fs)))
    mean = channel_means(src, chunk_rows)
    sos, zi = hpf_sos(args.hp_hz, src.fs, src.nch)
    stats = Stats(src.nch)

    with open(out, "wb") as fo:
        for x in src.chunks(chunk_rows):
            x -= mean
            if sos is not None:
                x, zi = signal.sosfilt(sos, x, axis=0, zi=zi)
            codes, clipped = quantise(x, args.gain)
            stats.add(x, clipped)
            codes.tofile(fo)

    return stats, src.fs


def run_in_memory(src, args, out):
    """Whole segment at once, because resample_poly carries no state."""
    need_gb = src.rows * src.nch * 8 * 3 / 1e9      # input, filtered, resampled
    print(f"  resampling: loading the segment whole (~{need_gb:.1f} GB peak). "
          f"--no-resample processes in chunks instead.")

    x = np.concatenate(list(src.chunks(src.rows)), axis=0)
    x -= x.mean(axis=0, keepdims=True)
    sos, zi = hpf_sos(args.hp_hz, src.fs, src.nch)
    if sos is not None:
        x = signal.sosfilt(sos, x, axis=0)

    ratio = fractions.Fraction(args.target_hz / src.fs).limit_denominator(64)
    up, down = ratio.numerator, ratio.denominator
    out_fs = src.fs * up / down
    print(f"  resample {src.fs:.4f} -> {out_fs:.1f} Hz  ({up}/{down}; "
          f"{100 * (out_fs - args.target_hz) / args.target_hz:+.3f}% from target)")
    x = signal.resample_poly(x, up, down, axis=0)

    codes, clipped = quantise(x, args.gain)
    stats = Stats(src.nch)
    stats.add(x, clipped)
    codes.tofile(out)
    return stats, out_fs


def main():
    args = parse_args()
    src = Source(args)

    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    if args.no_resample:
        stats, out_fs = run_chunked(src, args, args.out)
    else:
        stats, out_fs = run_in_memory(src, args, args.out)
    src.close()

    stats.report(src.nch, out_fs)

    size = os.path.getsize(args.out)
    row = src.nch * 2
    assert size % row == 0, "output is not a whole number of rows"
    print(f"  wrote {args.out}: {size} bytes = {size // row} rows of {src.nch} ch, "
          f"{size / row / out_fs:.2f} s at {out_fs:.0f} Hz")


if __name__ == "__main__":
    main()

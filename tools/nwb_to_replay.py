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

Reads /acquisition/timeseries/broadband/{data,timestamps}; NWB 1.0.6 HDF5,
the same h5py path inference_node.py uses. The data array is k x n integer
codes with a `conversion` attribute to volts.

    python3 nwb_to_replay.py indy_20161005_06_broadband.nwb \
        --start 120 --seconds 10 \
        --out ~/argus_data/indy_20161005_06_s120_10s.bin

    ros2 run argus_sim dataset_relay_node --ros-args \
        -p dataset_path:=$HOME/argus_data/indy_20161005_06_s120_10s.bin
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
                   help="keep the recording's own rate")
    p.add_argument("--gain", type=float, default=1.0,
                   help="extra scale applied before quantising (default 1.0)")
    p.add_argument("--data", default=DEFAULT_DATA, help="HDF5 path of the sample array")
    p.add_argument("--timestamps", default=DEFAULT_TIMESTAMPS,
                   help="HDF5 path of the per-sample timestamps")
    return p.parse_args()


def load_segment(args):
    with h5py.File(args.nwb, "r") as f:
        if args.data not in f:
            sys.exit(f"{args.data} not found. Top-level groups: {list(f.keys())}")
        d = f[args.data]
        ts = f[args.timestamps]

        k, n = d.shape
        conversion = float(d.attrs.get("conversion", 1.0))
        unit = d.attrs.get("unit", b"?")
        unit = unit.decode() if isinstance(unit, bytes) else str(unit)

        # Sample rate from the timestamps rather than assumed.
        probe = np.asarray(ts[: min(k, 100000)], dtype=np.float64)
        fs = 1.0 / np.median(np.diff(probe))
        t0 = float(probe[0])

        print(f"{args.nwb}")
        print(f"  {k} samples x {n} channels, {d.dtype}, {k / fs:.1f} s at {fs:.4f} Hz")
        print(f"  conversion={conversion:g} unit={unit}  t0={t0:.3f} s")

        if n < args.channels:
            sys.exit(f"only {n} channels in the file; --channels {args.channels} requested")

        i0 = int(round(args.start * fs))
        i1 = i0 + int(round(args.seconds * fs))
        if i0 < 0 or i1 > k:
            sys.exit(f"segment [{i0}, {i1}) is outside the {k}-sample recording")

        x = np.asarray(d[i0:i1, : args.channels], dtype=np.float64) * conversion

    return x, fs


def ac_couple(x, fs, hp_hz):
    x = x - x.mean(axis=0, keepdims=True)
    if hp_hz > 0:
        # Causal, like the chip. Mean is already gone so the settling
        # transient is negligible.
        sos = signal.butter(1, hp_hz, btype="highpass", fs=fs, output="sos")
        x = signal.sosfilt(sos, x, axis=0)
    return x


def resample(x, fs, target_hz):
    ratio = fractions.Fraction(target_hz / fs).limit_denominator(64)
    up, down = ratio.numerator, ratio.denominator
    achieved = fs * up / down
    print(f"  resample {fs:.4f} -> {achieved:.1f} Hz  ({up}/{down}; "
          f"{100 * (achieved - target_hz) / target_hz:+.3f}% from target)")
    return signal.resample_poly(x, up, down, axis=0), achieved


def quantise(x, gain):
    codes = np.rint(x * gain / RHD_LSB_V) + RHD_ZERO
    clipped = np.count_nonzero((codes < 0) | (codes > RHD_FULL))
    codes = np.clip(codes, 0, RHD_FULL)
    return codes.astype("<u2"), clipped


def main():
    args = parse_args()

    x, fs = load_segment(args)
    x = ac_couple(x, fs, args.hp_hz)

    out_fs = fs
    if not args.no_resample:
        x, out_fs = resample(x, fs, args.target_hz)

    rms_uv = np.sqrt(np.mean(x ** 2, axis=0)) * 1e6
    peak_uv = np.max(np.abs(x)) * 1e6
    print(f"  after AC coupling: RMS {rms_uv.min():.1f}..{rms_uv.max():.1f} uV "
          f"(median {np.median(rms_uv):.1f}), peak {peak_uv:.0f} uV")

    codes, clipped = quantise(x, args.gain)
    total = codes.size
    print(f"  quantised: {codes.shape[0]} samples x {codes.shape[1]} ch, "
          f"clipped {clipped}/{total} ({100 * clipped / total:.4f}%)")
    if clipped:
        print("  WARNING: clipping. Real spikes are 50-500 uV; if RMS above is "
              "in the mV range the conversion attribute is probably wrong.")

    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    codes.tofile(args.out)

    size = os.path.getsize(args.out)
    row = codes.shape[1] * 2
    assert size % row == 0, "output is not a whole number of rows"
    print(f"  wrote {args.out}: {size} bytes = {size // row} rows of {codes.shape[1]} ch, "
          f"{size / row / out_fs:.2f} s at {out_fs:.0f} Hz")


if __name__ == "__main__":
    main()

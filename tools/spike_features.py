#!/usr/bin/env python3
"""
spike_features.py -- bit-exact model of the codec's feature extraction.

Runs the same arithmetic the PL will run, on a dataset_relay_node .bin, and
writes what the fabric must produce for it. It is the reference the GHDL
testbench compares against, and a place to tune parameters on real data
before any of this becomes VHDL.

FEATURES (after Willett et al., the inner-speech dataset README)

  Per channel, per bin of BIN samples:
    count  -- threshold crossings: the high-passed signal going below
              -MULT x RMS, where RMS is a running estimate. One count per
              crossing, with a refractory hold-off so one spike's several
              sub-threshold samples count once. --bipolar counts positive
              crossings too.
    power  -- spike-band power: the sum of the squared high-passed signal
              over the bin. Divide by BIN for the mean; the fabric sends the
              sum.

ARITHMETIC (the contract the RTL implements; do not change one without
the other)

  x[n]   = code[n] - 32768                     signed, |x| <= 32768

  High-pass, order 1 (Q1.15):
  d[n]   = x[n] - x[n-1]                       17 bits signed
  y[n]   = (B0*d[n] + A1*y[n-1] + 2^14) >> 15  >> is an arithmetic shift;
                                               +2^14 rounds. Without it the
                                               floor on every step
                                               accumulates through the
                                               recursion to a -10 code DC
                                               bias.

  High-pass, order 2: the order-1 stage twice, in cascade.
  ya[n]  = (B0*(x[n] - x[n-1])  + A1*ya[n-1] + 2^14) >> 15
  y[n]   = (B0*(ya[n] - ya[n-1]) + A1*y[n-1] + 2^14) >> 15
                                               Same B0, A1, same Q1.15, one
                                               extra state word (ya[n-1] is
                                               both stage a's y and stage
                                               b's x). Each section is cut at
                                               0.6436 x HP_HZ so the pair is
                                               -3 dB at HP_HZ; 12 dB/octave
                                               below. A direct-form biquad
                                               was tried first: with poles at
                                               radius 0.964 its per-step
                                               rounding noise is amplified
                                               ~700x and the output missed
                                               float by 19 codes mean. The
                                               cascade misses by ~1.

  Either way HPF gain <= 1 so |y| <= 32768; RTL: 18-bit signed.

  sq[n]  = y[n] * y[n]                         <= 2^30; RTL: 32-bit unsigned
  u[n]   = min(sq[n], T[n-1])  after the fast-attack phase, else sq[n]
                                               Winsorised EMA input, a guard
                                               against a channel's own spikes
                                               raising its threshold. On the
                                               Indy segment it measured as a
                                               2% effect; kept because it is
                                               one mux on a comparator that
                                               already exists. --no-winsorize
                                               removes it.
  ms[n]  = ms[n-1] + ((u[n] - ms[n-1] + 2^(k-1)) >> k)
                                               EMA of u, rounded: the floor
                                               alone biases ms low by 2^(k-1),
                                               ~40% of a typical mean-square
                                               at K=15, turning 4.5 sigma
                                               into 3.5. k = K_FAST for the
                                               first 2^K samples, then K.
  T[n]   = (MULT_NUM * ms[n]) >> MULT_SHIFT    MULT^2 = NUM / 2^SHIFT,
                                               e.g. 4.5^2 = 81/4
  below  = (sq[n] > T[n]) and (y[n] < 0 or BIPOLAR)
  cross  = below and not below[n-1] and refractory == 0
  count += cross; power += sq[n]               power: RTL 48-bit
  refractory reloads to REFRAC on a cross and counts down otherwise

  Crossings are not counted for the first WARMUP samples, which defaults
  to 2^K -- by then the fast-attack phase is long over and the slow EMA
  has been tracking for most of a time constant. The RTL holds the same
  warm-up counter.

Coefficients come from scipy.signal.butter at the requested cutoff and are
quantised as above. The float filter is also run and the worst-case
difference from the fixed-point one is reported.

    python3 spike_features.py ~/argus_data/indy_20161005_06_s120_10s.bin \
        --golden ~/argus_data/indy_20161005_06_s120_10s.golden.txt

Outputs
  stdout    per-channel crossing rates, the channels that look alive, the
            fixed-vs-float filter error, and the quantised coefficients
  --golden  one line per (bin, channel): "bin ch count power" -- what the
            GHDL testbench reads and checks the DUT against. The header
            line records every parameter that produced it.
  --trace   the first --trace-samples of y for --trace-channel, one per
            line, for a filter-only unit test
"""

import argparse
import sys

import numpy as np
from scipy import signal

CHANNELS = 96
RHD_ZERO = 32768


def parse_args():
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("bin", help="dataset_relay_node .bin: uint16 LE, sample-major, 96 ch")
    p.add_argument("--fs", type=float, default=30012.0, help="sample rate (default 30012)")
    p.add_argument("--hp-hz", type=float, default=250.0, help="high-pass cutoff (default 250)")
    p.add_argument("--hp-order", type=int, default=1, choices=(1, 2),
                   help="high-pass order: 1 = one section, 2 = two cascaded sections (default 1)")
    p.add_argument("--bin", type=int, default=1500, dest="bin_len",
                   help="bin length in samples (default 1500 = 50 ms at 30012)")
    p.add_argument("--mult", type=float, default=4.5,
                   help="threshold as a multiple of RMS (default 4.5; Willett uses 3.5 or 4.5)")
    p.add_argument("--bipolar", action="store_true",
                   help="count positive crossings as well as negative")
    p.add_argument("--ms-shift", type=int, default=15,
                   help="EMA shift K while tracking; time constant 2^K samples (default 15 = 1.09 s)")
    p.add_argument("--ms-shift-fast", type=int, default=8,
                   help="EMA shift for the first 2^K samples (default 8: settled in ~1000 samples)")
    p.add_argument("--no-winsorize", action="store_true",
                   help="feed the EMA raw sq instead of min(sq, T)")
    p.add_argument("--refrac", type=int, default=30,
                   help="refractory samples after a crossing (default 30 = 1 ms)")
    p.add_argument("--warmup", type=int, default=None,
                   help="samples before crossings count (default 2^K)")
    p.add_argument("--golden", help="write per-bin golden file for the GHDL testbench")
    p.add_argument("--trace", help="write the filtered signal of one channel")
    p.add_argument("--trace-channel", type=int, default=0)
    p.add_argument("--trace-samples", type=int, default=4096)
    return p.parse_args()


def mult_to_rational(mult):
    """MULT^2 as NUM / 2^SHIFT with SHIFT chosen so NUM is an integer.

    4.5 -> 20.25 = 81/4 (SHIFT 2). 3.5 -> 12.25 = 49/4. Anything else is
    approximated with SHIFT 8, good to 0.4%.
    """
    sq = mult * mult
    for shift in range(0, 9):
        num = sq * (1 << shift)
        if abs(num - round(num)) < 1e-9:
            return int(round(num)), shift
    return int(round(sq * 256)), 8


# Two identical first-order sections are -3 dB at fc when each is cut at
# fc * sqrt(sqrt(2) - 1).
CASCADE_SECTION_SCALE = (2 ** 0.5 - 1) ** 0.5


def hpf_coefficients(hp_hz, fs, order):
    """Float b, a of one first-order section, plus its Q1.15 integers.

    Order 1 uses one section at hp_hz. Order 2 uses two identical sections,
    each at hp_hz * 0.6436, so the pair is -3 dB at hp_hz. The recurrence
    is the same either way; the RTL runs it once or twice per sample.
    """
    section_hz = hp_hz if order == 1 else hp_hz * CASCADE_SECTION_SCALE
    b, a = signal.butter(1, section_hz, btype="highpass", fs=fs)
    q = {"B0": int(round(b[0] * 32768)), "A1": int(round(-a[1] * 32768))}
    return b, a, q, section_hz


def run_fixed(codes, q, order, bin_len, mult_num, mult_shift, ms_shift, ms_shift_fast,
              refrac_len, warmup, winsorize, bipolar, keep_y=0, progress=None):
    """The arithmetic block above, vectorised across channels, looped over
    samples. int64 throughout so nothing overflows in the model; the RTL
    widths are in the header.

    codes may be a uint16 memmap: rows are converted one at a time, so a
    multi-minute file costs no more memory than a ten-second one. The
    filtered signal is returned only for the first keep_y rows.
    """
    n, nch = codes.shape

    x1 = np.zeros(nch, dtype=np.int64)     # x[n-1]
    ya1 = np.zeros(nch, dtype=np.int64)    # stage a output, previous
    y1 = np.zeros(nch, dtype=np.int64)     # final output, previous
    ms = np.zeros(nch, dtype=np.int64)
    thr = np.zeros(nch, dtype=np.int64)
    below_prev = np.zeros(nch, dtype=bool)
    refrac = np.zeros(nch, dtype=np.int64)
    count = np.zeros(nch, dtype=np.int64)
    power = np.zeros(nch, dtype=np.int64)

    n_bins = n // bin_len
    counts = np.zeros((n_bins, nch), dtype=np.int64)
    powers = np.zeros((n_bins, nch), dtype=np.int64)
    y_out = np.zeros((min(keep_y, n), nch), dtype=np.int64)

    B0, A1 = q["B0"], q["A1"]

    for i in range(n):
        if progress is not None and (i & 0x3FFFF) == 0 and i:
            progress(i, n)
        xi = codes[i].astype(np.int64) - RHD_ZERO

        if order == 1:
            y = (B0 * (xi - x1) + A1 * y1 + (1 << 14)) >> 15
        else:
            ya = (B0 * (xi - x1) + A1 * ya1 + (1 << 14)) >> 15
            y = (B0 * (ya - ya1) + A1 * y1 + (1 << 14)) >> 15
            ya1 = ya

        sq = y * y
        fast = i < (1 << ms_shift)
        k = ms_shift_fast if fast else ms_shift
        u = sq if (fast or not winsorize) else np.minimum(sq, thr)
        ms = ms + ((u - ms + (1 << (k - 1))) >> k)
        thr = (mult_num * ms) >> mult_shift

        big = sq > thr
        below = big if bipolar else (big & (y < 0))
        cross = below & ~below_prev & (refrac == 0) & (i >= warmup)

        count += cross
        power += sq
        refrac = np.where(cross, refrac_len, np.maximum(refrac - 1, 0))

        if i < keep_y:
            y_out[i] = y
        x1 = xi
        y1 = y
        below_prev = below

        if (i + 1) % bin_len == 0:
            b = (i + 1) // bin_len - 1
            counts[b] = count
            powers[b] = power
            count[:] = 0
            power[:] = 0

    return counts, powers, y_out


def main():
    args = parse_args()
    if args.warmup is None:
        args.warmup = 1 << args.ms_shift

    raw = np.memmap(args.bin, dtype="<u2", mode="r")
    if raw.size % CHANNELS != 0:
        sys.exit(f"{args.bin}: {raw.size} words is not a multiple of {CHANNELS}")
    codes = raw.reshape(-1, CHANNELS)
    n = codes.shape[0]
    print(f"{args.bin}: {n} samples x {CHANNELS} ch, {n / args.fs:.2f} s at {args.fs:.0f} Hz")

    b, a, q, section_hz = hpf_coefficients(args.hp_hz, args.fs, args.hp_order)
    mult_num, mult_shift = mult_to_rational(args.mult)
    stages = "one section" if args.hp_order == 1 else f"two sections at {section_hz:.0f} Hz"
    print(f"  HPF -3 dB at {args.hp_hz:.0f} Hz, {stages}: "
          + " ".join(f"{k}={v}" for k, v in q.items()) + " (Q1.15)")
    print(f"  threshold {args.mult}xRMS{' bipolar' if args.bipolar else ''} -> "
          f"MULT_NUM={mult_num} MULT_SHIFT={mult_shift}; "
          f"EMA shift {args.ms_shift_fast} then {args.ms_shift} "
          f"({(1 << args.ms_shift) / args.fs:.2f} s); "
          f"refractory {args.refrac}; warm-up {args.warmup}; "
          f"EMA input {'raw sq' if args.no_winsorize else 'min(sq, T)'}")

    # The float cross-check runs on a prefix so a long file stays cheap.
    check_n = min(n, 300000)
    keep_y = max(check_n, args.trace_samples if args.trace else 0)

    counts, powers, y_fixed = run_fixed(
        codes, q, args.hp_order, args.bin_len, mult_num, mult_shift,
        args.ms_shift, args.ms_shift_fast, args.refrac, args.warmup,
        not args.no_winsorize, args.bipolar, keep_y=keep_y)

    y_float = codes[:check_n].astype(np.float64) - RHD_ZERO
    for _ in range(args.hp_order):
        y_float = signal.lfilter(b, a, y_float, axis=0)
    err = np.abs(y_fixed[:check_n] - y_float)
    print(f"  fixed vs float HPF over the first {check_n} samples: max |err| {err.max():.2f} "
          f"codes, mean {err.mean():.3f} (rounded per step; ~0.7 codes per section is normal)")

    n_bins = counts.shape[0]
    counted_s = max(n - args.warmup, 1) / args.fs
    rate = counts.sum(axis=0) / counted_s
    alive = rate > 1.0
    order_idx = np.argsort(rate)[::-1]

    print(f"  {n_bins} bins of {args.bin_len} samples; crossings counted over {counted_s:.2f} s")
    print(f"  crossing rate: min {rate.min():.1f}  median {np.median(rate):.1f}  "
          f"max {rate.max():.1f} Hz;  {alive.sum()}/{CHANNELS} channels above 1 Hz")
    print("  busiest channels: " +
          ", ".join(f"ch{c} {rate[c]:.0f} Hz" for c in order_idx[:5]))
    print(f"  spike-band power per bin: median {np.median(powers):.3g}, "
          f"max {powers.max():.3g} (sum of squares over {args.bin_len} samples)")

    if args.golden:
        coeffs = " ".join(f"{k}={v}" for k, v in q.items())
        with open(args.golden, "w", encoding="ascii") as f:
            f.write(f"# bin ch count power  -- {n_bins} bins x {CHANNELS} ch, "
                    f"ORDER={args.hp_order} {coeffs} NUM={mult_num} SHIFT={mult_shift} "
                    f"K={args.ms_shift} K_FAST={args.ms_shift_fast} REFRAC={args.refrac} "
                    f"WARMUP={args.warmup} WINSOR={0 if args.no_winsorize else 1} "
                    f"BIPOLAR={1 if args.bipolar else 0}\n")
            for bi in range(n_bins):
                for c in range(CHANNELS):
                    f.write(f"{bi} {c} {counts[bi, c]} {powers[bi, c]}\n")
        print(f"  wrote {args.golden}: {n_bins * CHANNELS} lines")

    if args.trace:
        c = args.trace_channel
        m = min(args.trace_samples, n, y_fixed.shape[0])
        with open(args.trace, "w", encoding="ascii") as f:
            f.write(f"# code y  -- channel {c}, first {m} samples, ORDER={args.hp_order} "
                    + " ".join(f"{k}={v}" for k, v in q.items()) + "\n")
            for i in range(m):
                f.write(f"{codes[i, c]} {y_fixed[i, c]}\n")
        print(f"  wrote {args.trace}: channel {c}, {m} samples")


if __name__ == "__main__":
    main()

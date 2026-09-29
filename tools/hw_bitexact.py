#!/usr/bin/env python3
"""
hw_bitexact.py -- is the codec on silicon bit-exact with spike_features.py.

The GHDL bench proves argus_feature.vhd against the model in simulation.
This proves the board: the NeuralFrames that come off the wire during a
replay run, against the model's arithmetic on the same replay .bin.

Two steps, because the capture has to be listening before the run starts:

    python3 hw_bitexact.py capture /tmp/cap.npz --seconds 150 &
    scripts/hwtest.sh --seconds 90
    python3 hw_bitexact.py compare ~/argus_data/indy_20161005_06_s120_10s.bin /tmp/cap.npz

capture   subscribes to the receiver's NeuralFrame topic and records every
          frame, in arrival order: (sample, channels[96], power[96]).
compare   finds where streaming starts, runs the model from sample 0 and
          compares counts and power exactly.

WHERE STREAMING STARTS

The receiver comes up while the board still runs the previous image, and
the new image sends identity-source bins before priming completes. Entry
to external mode soft-resets the chain: the feature state clears, and
FEATURE_INDEX -- the frame's sample field -- restarts at 0 and advances
once per completed bin. So the stream of interest is the one after the
last point where sample went down. Its frame with sample j carries the
bin the fabric finished j-th, which is model bin j - 1 (--index-offset).

THE MODEL

spike_features.run_fixed with the parameters argus_feature.vhd is built
with (its generic defaults): first-order 250 Hz HPF, 3.5 sigma, K=15,
K_FAST=8, refractory 30, warm-up 32768, winsorised, negative-going. Model
bin k covers replay samples [1500k, 1500k + 1500); the relay loops the file,
so sample s is file row s mod rows, with the filter state carried across
the wrap as the fabric carries it. Power on the wire is the bin's sum of
squares divided by 1500 in the firmware (integer division), so that is
what is compared.

The first ~22 bins have zero counts on both sides: crossings are not
counted until the EMA has run 2^15 samples. Power is live from bin 0.

A match needs the replay to have been underrun-free over the compared
window -- an underrun replays a stale half, which is a different input.
Check the hwtest log's stream: lines for that; this tool cannot see them.
"""

import argparse
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import spike_features as sf  # noqa: E402

CHANNELS = 96
BIN_LEN = 1500

# argus_feature.vhd generic defaults, as the top level instantiates it.
RTL = {'hp_hz': 250.0, 'fs': 30012.0, 'order': 1, 'mult': 3.5, 'ms_shift': 15,
       'ms_shift_fast': 8, 'refrac': 30, 'warmup': 32768, 'winsorize': True,
       'bipolar': False}


def parse_args():
    """Command line: capture or compare."""
    p = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    sub = p.add_subparsers(dest='cmd', required=True)

    c = sub.add_parser('capture', help='record NeuralFrames to an .npz')
    c.add_argument('out', help='output .npz')
    c.add_argument('--seconds', type=float, default=150.0,
                   help='how long to listen (default 150: covers a 90 s hwtest run and its build)')
    c.add_argument('--topic', default='/argus/neural_interface_bridge/neural_data',
                   help="NeuralFrame topic (default: neural_udp_receiver's output)")

    m = sub.add_parser('compare', help='compare a capture against the model')
    m.add_argument('bin', help='the replay .bin the relay served: uint16 LE, sample-major, 96 ch')
    m.add_argument('capture', help='.npz written by capture')
    m.add_argument('--bins', type=int, default=100, help='bins to compare (default 100)')
    m.add_argument('--index-offset', type=int, default=1,
                   help='model bin = sample - this '
                        '(default 1: FEATURE_INDEX counts completed bins)')
    return p.parse_args()


def capture(args):
    """Record every NeuralFrame on the topic for --seconds, then save."""
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from argus_core.msg import NeuralFrame

    rclpy.init()
    node = rclpy.create_node('hw_bitexact_capture')
    samples, counts, powers = [], [], []

    def on_frame(msg):
        samples.append(msg.sample)
        counts.append(np.array(msg.channels, dtype=np.uint16))
        powers.append(np.array(msg.power, dtype=np.uint32))

    qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                     history=HistoryPolicy.KEEP_LAST, depth=1000)
    node.create_subscription(NeuralFrame, args.topic, on_frame, qos)

    print(f'capture: listening on {args.topic} for {args.seconds:.0f} s', flush=True)
    end = time.monotonic() + args.seconds
    try:
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        np.savez(args.out,
                 sample=np.array(samples, dtype=np.uint32),
                 channels=np.array(counts, dtype=np.uint16).reshape(-1, CHANNELS),
                 power=np.array(powers, dtype=np.uint32).reshape(-1, CHANNELS))
        print(f'capture: {len(samples)} frames -> {args.out}', flush=True)
        node.destroy_node()
        rclpy.shutdown()


def last_stream(sample):
    """Index of the first frame after the last place sample went down."""
    drops = np.nonzero(np.diff(sample.astype(np.int64)) < 0)[0]
    return int(drops[-1]) + 1 if drops.size else 0


def model_bins(bin_path, n_bins):
    """Return counts and wire power (sum // BIN_LEN) of the first n_bins, file looped."""
    raw = np.memmap(bin_path, dtype='<u2', mode='r')
    if raw.size % CHANNELS != 0:
        sys.exit(f'{bin_path}: {raw.size} words is not a multiple of {CHANNELS}')
    codes = raw.reshape(-1, CHANNELS)
    rows = codes.shape[0]
    n = n_bins * BIN_LEN
    looped = codes[np.arange(n) % rows] if n > rows else np.asarray(codes[:n])

    _, _, q, _ = sf.hpf_coefficients(RTL['hp_hz'], RTL['fs'], RTL['order'])
    mult_num, mult_shift = sf.mult_to_rational(RTL['mult'])
    counts, sums, _ = sf.run_fixed(
        looped, q, RTL['order'], BIN_LEN, mult_num, mult_shift,
        RTL['ms_shift'], RTL['ms_shift_fast'], RTL['refrac'], RTL['warmup'],
        RTL['winsorize'], RTL['bipolar'])
    return counts, sums // BIN_LEN, rows


def compare(args):
    """Compare the capture's last stream with the model; 0 if every bin is exact."""
    cap = np.load(args.capture)
    sample, hw_c, hw_p = cap['sample'], cap['channels'], cap['power']
    if sample.size == 0:
        sys.exit(f'{args.capture}: no frames')

    s0 = last_stream(sample)
    sample, hw_c, hw_p = sample[s0:], hw_c[s0:].astype(np.int64), hw_p[s0:].astype(np.int64)
    print(f"capture: {cap['sample'].size} frames; stream after the last restart starts at "
          f'frame {s0}, sample {sample[0]}, {sample.size} frames')

    gaps = np.nonzero(np.diff(sample.astype(np.int64)) != 1)[0]
    if gaps.size:
        print(f'  WARNING: sample not +1 at {gaps.size} places, first after sample '
              f'{sample[gaps[0]]} -> {sample[gaps[0] + 1]}')

    # Frames to compare: model bins 0 .. bins-1, which are samples offset ..
    want = np.arange(args.bins) + args.index_offset
    have = {int(s): i for i, s in enumerate(sample)}
    missing = [int(s) for s in want if int(s) not in have]
    if missing:
        print(f'  WARNING: {len(missing)} of {args.bins} wanted samples not captured '
              f'(first {missing[0]}); they count as mismatches')

    # One extra bin either side, for the offset diagnostic.
    n_model = args.bins + 2
    print(f'model: {n_model} bins of {BIN_LEN} from {args.bin} ...', flush=True)
    t0 = time.monotonic()
    m_c, m_p, rows = model_bins(args.bin, n_model)
    print(f'  {rows} rows in the file; model took {time.monotonic() - t0:.0f} s')

    def score(offset, n):
        ok_bins = ok_c = ok_p = 0
        for k in range(n):
            i = have.get(k + offset)
            if i is None or k >= m_c.shape[0]:
                continue
            ceq = hw_c[i] == m_c[k]
            peq = hw_p[i] == m_p[k]
            ok_c += int(ceq.sum())
            ok_p += int(peq.sum())
            ok_bins += int(ceq.all() and peq.all())
        return ok_bins, ok_c, ok_p

    n = args.bins
    ok_bins, ok_c, ok_p = score(args.index_offset, n)
    total = n * CHANNELS
    print(f'compare: model bin k vs sample k+{args.index_offset}, k = 0..{n - 1}')
    print(f'  bins exact (all 96 counts and powers): {ok_bins}/{n} = {100.0 * ok_bins / n:.1f}%')
    print(f'  counts equal: {ok_c}/{total} = {100.0 * ok_c / total:.2f}%')
    print(f'  power  equal: {ok_p}/{total} = {100.0 * ok_p / total:.2f}%')

    first_nz = np.nonzero(m_c[:n].any(axis=1))[0]
    print(f"  model's first bin with a nonzero count: {first_nz[0] if first_nz.size else 'none'}")

    for k in range(n):
        i = have.get(k + args.index_offset)
        if i is None:
            continue
        bad = np.nonzero((hw_c[i] != m_c[k]) | (hw_p[i] != m_p[k]))[0]
        if bad.size:
            ch = int(bad[0])
            print(f'  first mismatch: bin {k} (sample {k + args.index_offset}), '
                  f'{bad.size} channels, '
                  f'ch{ch} hw count {hw_c[i][ch]} power {hw_p[i][ch]} vs model count '
                  f'{m_c[k][ch]} power {m_p[k][ch]}')
            break

    print('  offset diagnostic (bins exact over the same span):')
    for off in (args.index_offset - 1, args.index_offset, args.index_offset + 1):
        b, _, _ = score(off, n)
        print(f'    sample = bin + {off}: {b}/{n}')

    return 0 if ok_bins == n else 1


def main():
    """Run the chosen subcommand."""
    args = parse_args()
    if args.cmd == 'capture':
        capture(args)
        return 0
    return compare(args)


if __name__ == '__main__':
    sys.exit(main())

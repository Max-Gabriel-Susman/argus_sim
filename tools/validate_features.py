#!/usr/bin/env python3
"""
validate_features.py -- check the model's crossings against the lab's sorting.

spike_features.py counts threshold crossings on the broadband. The .mat for
the same session carries spike times from the lab's own detector and sorter,
on the same clock. If the two agree on which channels are busy, and roughly
how busy, the fixed-point detector is confirmed against an independent
reference on real data -- which is the bar to clear before its parameters
become RTL generics.

Rates will not match exactly. The lab's threshold, refractory and sorting
are theirs; ours are the Willett defaults. So the test is rank agreement
across channels (Spearman rho) and overlap of the busiest few, plus a look
at channels the sorter found nothing on: the model's rate there is its
false-crossing floor on real noise.

    python3 validate_features.py \
        ~/argus_data/indy_20161005_06_s120_10s.golden.txt \
        ~/argus_data/indy_20161005_06.mat \
        --start 120 --t0 1278.0

--t0 is the broadband recording's first timestamp, printed by
nwb_to_replay.py. --start, --fs, --bin and --warmup must match the run that
produced the golden file; the defaults are spike_features.py's defaults.
"""

import argparse
import sys

import h5py
import numpy as np
from scipy import stats

CHANNELS = 96


def parse_args():
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("golden", help="spike_features.py --golden output")
    p.add_argument("mat", help="indy_20161005_06.mat (v7.3)")
    p.add_argument("--start", type=float, required=True,
                   help="segment start, seconds into the broadband (the nwb_to_replay --start)")
    p.add_argument("--t0", type=float, required=True,
                   help="broadband first timestamp, seconds (nwb_to_replay prints t0)")
    p.add_argument("--fs", type=float, default=30012.0)
    p.add_argument("--bin", type=int, default=1500, dest="bin_len")
    p.add_argument("--warmup", type=int, default=32768)
    p.add_argument("--top", type=int, default=10, help="channels to list (default 10)")
    return p.parse_args()


def load_golden(path):
    g = np.loadtxt(path, dtype=np.int64, comments="#")
    n_bins = int(g[:, 0].max()) + 1
    counts = np.zeros((n_bins, CHANNELS), dtype=np.int64)
    counts[g[:, 0], g[:, 1]] = g[:, 2]
    return counts


def load_mat_spikes(path):
    """spikes is (units x channels) of references to time vectors. Mirrors
    inference_node._load_mat_v73 / _read_spike_times so the two tools read
    the file the same way."""
    out = []
    with h5py.File(path, "r") as f:
        refs = np.array(f["/spikes"])
        n_units, n_ch = refs.shape
        for u in range(n_units):
            row = []
            for c in range(n_ch):
                ref = refs[u, c]
                if ref == 0:
                    row.append(np.empty((0,), dtype=np.float64))
                    continue
                arr = np.array(f[ref]).squeeze().astype(np.float64).reshape(-1)
                row.append(arr)
            out.append(row)
    return out, n_units, n_ch


def main():
    args = parse_args()

    counts = load_golden(args.golden)
    n_bins = counts.shape[0]
    model = counts.sum(axis=0)

    # Window the model actually counted over: after warm-up, to the last bin.
    seg0 = args.t0 + args.start
    w0 = seg0 + args.warmup / args.fs
    w1 = seg0 + n_bins * args.bin_len / args.fs
    dur = w1 - w0

    spikes, n_units, n_ch = load_mat_spikes(args.mat)
    if n_ch < CHANNELS:
        sys.exit(f"{args.mat}: only {n_ch} channels")

    lab_all = np.zeros(CHANNELS, dtype=np.int64)     # every unit incl. unsorted
    lab_sorted = np.zeros(CHANNELS, dtype=np.int64)  # units 1.. only
    for c in range(CHANNELS):
        for u in range(n_units):
            st = spikes[u][c]
            n = int(np.count_nonzero((st >= w0) & (st < w1)))
            lab_all[c] += n
            if u > 0:
                lab_sorted[c] += n

    print(f"window {w0:.2f}..{w1:.2f} s ({dur:.2f} s), {n_bins} bins, "
          f"{n_units} units/channel in the .mat")
    print(f"  model: {model.sum()} crossings, {np.count_nonzero(model / dur > 1)} ch above 1 Hz")
    print(f"  lab:   {lab_all.sum()} spikes all units, {lab_sorted.sum()} sorted; "
          f"{np.count_nonzero(lab_all / dur > 1)} ch above 1 Hz")

    rho_all, _ = stats.spearmanr(model, lab_all)
    alive = (model / dur > 1) | (lab_all / dur > 1)
    rho_alive, _ = stats.spearmanr(model[alive], lab_all[alive]) if alive.sum() > 2 else (float("nan"), 0)
    print(f"  Spearman rho, model vs lab (all units): {rho_all:.3f} over 96 ch, "
          f"{rho_alive:.3f} over the {alive.sum()} ch above 1 Hz in either")
    print("    (the 96-ch figure is dragged down by ties among dead electrodes; "
          "the alive-only figure is the one that matters)")

    k = args.top
    top_model = set(int(c) for c in np.argsort(model)[::-1][:5])
    top_lab = set(int(c) for c in np.argsort(lab_all)[::-1][:5])
    print(f"  busiest-5 overlap: {len(top_model & top_lab)}/5  "
          f"model {sorted(top_model)}  lab {sorted(top_lab)}")

    dead = lab_all == 0
    if dead.any():
        print(f"  {dead.sum()} channels with no lab spikes in window: model rate there "
              f"min {model[dead].min() / dur:.2f}  median {np.median(model[dead]) / dur:.2f}  "
              f"max {model[dead].max() / dur:.2f} Hz  <- false-crossing floor on real noise")

    print(f"\n  {'ch':>4} {'model Hz':>9} {'lab all Hz':>11} {'lab sorted Hz':>14}")
    for c in np.argsort(model)[::-1][:k]:
        print(f"  {c:>4} {model[c] / dur:9.1f} {lab_all[c] / dur:11.1f} {lab_sorted[c] / dur:14.1f}")


if __name__ == "__main__":
    main()

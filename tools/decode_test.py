#!/usr/bin/env python3
"""
decode_test.py -- does the fixed-point feature stream decode?

The bar for the codec's feature extraction is not "match the lab's spike
sorter" -- a threshold detector cannot, and Willett's whole result is that
it does not need to. The bar is whether its output decodes intent as well
as the features the decoder was trained on. This runs that comparison on
the Indy session, offline, with everything on disk already.

Three feature sets, same bins, same labels, same classifier:

  model     threshold-crossing counts from spike_features.run_fixed on the
            broadband .bin -- what the fabric will produce
  mat-u1    spike counts of unit 1 per channel from the .mat -- exactly what
            inference_node trains on (ARGUS_UNIT_INDEX=1)
  mat-all   every unit per channel from the .mat, hash included -- the
            lab's own threshold-crossing stream, roughly

Labels are inference_node's: the angle from cursor to target at each bin's
centre, quadrantised to four classes. Classifier is inference_node's:
StandardScaler -> LDA, 80/20 stratified split with its random_state, plus
5-fold CV for a number that does not depend on one split. A shuffled-label
run on the model features gives the chance floor.

    python3 decode_test.py \
        ~/argus_data/indy_20161005_06_s10_199s_24k.bin \
        ~/argus_data/indy_20161005_06.mat \
        --start 10 --t0 1278.0 --fs 24414.0625

The .bin should be the recording's native rate (nwb_to_replay --no-resample)
and cover the .mat's behavioural window: the broadband starts at t0=1278 and
the .mat's cursor data spans 1288..1487, so --start 10 --seconds 199. The
model runs once and its features are cached beside the .bin; rerunning
with different classifier options is then instant.

If model tracks mat-u1 within a few points, the feature stream is validated
for its purpose and the RTL contract in spike_features.py is final. If it
is far below, the detector's parameters need revisiting -- with this as
the metric, not sorter agreement.
"""

import argparse
import hashlib
import math
import os
import sys
import time
import warnings

import h5py
import numpy as np
from sklearn.discriminant_analysis import LinearDiscriminantAnalysis
from sklearn.model_selection import StratifiedKFold, cross_val_score, train_test_split
from sklearn.pipeline import make_pipeline
from sklearn.preprocessing import StandardScaler

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import spike_features as sf  # noqa: E402

CHANNELS = 96


def parse_args():
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("bin", help="broadband .bin at native rate, covering the .mat window")
    p.add_argument("mat", help="indy_20161005_06.mat (v7.3)")
    p.add_argument("--start", type=float, required=True,
                   help="seconds into the broadband the .bin starts (nwb_to_replay --start)")
    p.add_argument("--t0", type=float, required=True,
                   help="broadband first timestamp (nwb_to_replay prints t0)")
    p.add_argument("--fs", type=float, default=24414.0625, help="the .bin's sample rate")
    p.add_argument("--bin-s", type=float, default=0.05, help="bin length, seconds (default 0.05)")
    # model parameters, spike_features defaults
    p.add_argument("--hp-hz", type=float, default=250.0)
    p.add_argument("--hp-order", type=int, default=1, choices=(1, 2))
    p.add_argument("--mult", type=float, default=4.5)
    p.add_argument("--bipolar", action="store_true")
    p.add_argument("--ms-shift", type=int, default=15)
    p.add_argument("--ms-shift-fast", type=int, default=8)
    p.add_argument("--no-winsorize", action="store_true")
    p.add_argument("--refrac", type=int, default=30)
    # classifier
    p.add_argument("--features", choices=("counts", "power", "both"), default="counts",
                   help="model feature set (default counts, matching inference_node)")
    p.add_argument("--shrinkage", action="store_true",
                   help="LDA solver=lsqr shrinkage=auto instead of inference_node's default svd")
    p.add_argument("--seed", type=int, default=7, help="split seed (inference_node uses 7)")
    p.add_argument("--no-cache", action="store_true")
    return p.parse_args()


# --- .mat, mirroring inference_node ---------------------------------------

def load_mat_behaviour(path):
    with h5py.File(path, "r") as f:
        t = np.array(f["/t"]).squeeze().astype(np.float64)
        cursor = np.array(f["/cursor_pos"]).T.astype(np.float64)
        target = np.array(f["/target_pos"]).T.astype(np.float64)
    return t, cursor, target


def labels_for(bin_times, t, cursor, target):
    idx = np.searchsorted(t, bin_times, side="left")
    idx = np.clip(idx, 0, t.shape[0] - 1)
    vec = target[idx] - cursor[idx]
    ang = np.arctan2(vec[:, 1], vec[:, 0])
    y = np.zeros(bin_times.shape[0], dtype=np.int64)
    y[(ang >= math.pi / 4) & (ang < 3 * math.pi / 4)] = 1
    y[(ang >= 3 * math.pi / 4) | (ang < -3 * math.pi / 4)] = 2
    y[(ang >= -3 * math.pi / 4) & (ang < -math.pi / 4)] = 3
    return y


def bin_mat_spikes(path, edges, nch):
    """Counts per bin per channel: unit 1 only, and all units."""
    n_bins = edges.shape[0] - 1
    u1 = np.zeros((n_bins, nch), dtype=np.float32)
    allu = np.zeros((n_bins, nch), dtype=np.float32)
    with h5py.File(path, "r") as f:
        refs = np.array(f["/spikes"])
        n_units, n_ch = refs.shape
        for c in range(min(nch, n_ch)):
            for u in range(n_units):
                ref = refs[u, c]
                if ref == 0:
                    continue
                st = np.array(f[ref]).squeeze().astype(np.float64).reshape(-1)
                if st.size == 0:
                    continue
                h, _ = np.histogram(st, bins=edges)
                allu[:, c] += h
                if u == 1:
                    u1[:, c] += h
    return u1, allu


# --- model, cached ----------------------------------------------------------

def model_features(args, codes, bin_len):
    key = (f"{os.path.getsize(args.bin)}|{args.fs}|{bin_len}|{args.hp_hz}|{args.hp_order}|"
           f"{args.mult}|{args.bipolar}|{args.ms_shift}|{args.ms_shift_fast}|"
           f"{args.no_winsorize}|{args.refrac}")
    tag = hashlib.sha1(key.encode()).hexdigest()[:10]
    cache = f"{args.bin}.features.{tag}.npz"

    if not args.no_cache and os.path.exists(cache):
        z = np.load(cache)
        print(f"  model features from cache {os.path.basename(cache)}")
        return z["counts"], z["powers"], int(z["warmup"])

    b, a, q, section_hz = sf.hpf_coefficients(args.hp_hz, args.fs, args.hp_order)
    num, shift = sf.mult_to_rational(args.mult)
    warmup = 1 << args.ms_shift
    n = codes.shape[0]
    print(f"  running model on {n} samples ({n / args.fs:.0f} s): "
          f"B0={q['B0']} A1={q['A1']} NUM={num} SHIFT={shift} K={args.ms_shift} "
          f"order {args.hp_order}{' bipolar' if args.bipolar else ''}")

    t_start = time.time()

    def progress(i, total):
        el = time.time() - t_start
        print(f"    {100 * i / total:3.0f}%  {el:.0f} s elapsed, ~{el * (total / i - 1):.0f} s left",
              flush=True)

    counts, powers, _ = sf.run_fixed(
        codes, q, args.hp_order, bin_len, num, shift, args.ms_shift, args.ms_shift_fast,
        args.refrac, warmup, not args.no_winsorize, args.bipolar, keep_y=0, progress=progress)
    print(f"    done in {time.time() - t_start:.0f} s")

    if not args.no_cache:
        np.savez_compressed(cache, counts=counts, powers=powers, warmup=warmup)
    return counts, powers, warmup


# --- classifier, mirroring inference_node -----------------------------------

def make_pipe(shrinkage):
    lda = (LinearDiscriminantAnalysis(solver="lsqr", shrinkage="auto") if shrinkage
           else LinearDiscriminantAnalysis())
    return make_pipeline(StandardScaler(with_mean=True, with_std=True), lda)


def fit_score(X, y, seed, shrinkage):
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")   # collinear dead channels; sklearn copes
        X_tr, X_te, y_tr, y_te = train_test_split(
            X, y, test_size=0.2, random_state=seed, stratify=y)
        pipe = make_pipe(shrinkage).fit(X_tr, y_tr)
        split_acc = pipe.score(X_te, y_te)
        cv = StratifiedKFold(5, shuffle=True, random_state=seed)
        scores = cross_val_score(make_pipe(shrinkage), X, y, cv=cv)
    return split_acc, scores.mean(), scores.std()


def main():
    args = parse_args()

    raw = np.memmap(args.bin, dtype="<u2", mode="r")
    if raw.size % CHANNELS != 0:
        sys.exit(f"{args.bin}: {raw.size} words is not a multiple of {CHANNELS}")
    codes = raw.reshape(-1, CHANNELS)
    n = codes.shape[0]
    bin_len = int(round(args.bin_s * args.fs))
    n_bins = n // bin_len
    print(f"{args.bin}: {n} samples, {n / args.fs:.1f} s at {args.fs:.4f} Hz, "
          f"{n_bins} bins of {bin_len} samples ({bin_len / args.fs * 1000:.2f} ms)")

    counts, powers, warmup = model_features(args, codes, bin_len)

    # Bin edges on the session clock; drop warm-up bins and anything outside
    # the .mat's behavioural record.
    w0 = args.t0 + args.start
    edges = w0 + np.arange(n_bins + 1) * (bin_len / args.fs)
    centres = edges[:-1] + 0.5 * bin_len / args.fs

    t, cursor, target = load_mat_behaviour(args.mat)
    skip = int(math.ceil(warmup / bin_len))
    keep = np.zeros(n_bins, dtype=bool)
    keep[skip:] = True
    keep &= (centres >= t[0]) & (centres <= t[-1])
    print(f"  window {edges[0]:.1f}..{edges[-1]:.1f} s; .mat behaviour {t[0]:.1f}..{t[-1]:.1f} s; "
          f"{skip} warm-up bins dropped; {keep.sum()} bins usable")
    if keep.sum() < 200:
        sys.exit("too few usable bins -- is the .bin covering the .mat window? (see --start)")

    y = labels_for(centres[keep], t, cursor, target)
    classes, freq = np.unique(y, return_counts=True)
    chance = freq.max() / freq.sum()
    print(f"  labels: " + ", ".join(f"{c}:{k}" for c, k in zip(classes, freq))
          + f"  (majority class {100 * chance:.1f}%)")

    u1, allu = bin_mat_spikes(args.mat, edges, CHANNELS)

    Xm_counts = counts[keep].astype(np.float32)
    Xm_power = powers[keep].astype(np.float32)
    feats = {"counts": Xm_counts, "power": Xm_power,
             "both": np.hstack([Xm_counts, Xm_power])}[args.features]
    sets = [
        (f"model ({args.features})", feats),
        ("mat-u1  (inference_node's features)", u1[keep]),
        ("mat-all (every unit incl. hash)", allu[keep]),
    ]
    print(f"  model: {int(counts[keep].sum())} crossings, "
          f"{np.count_nonzero(counts[keep].sum(axis=0))} ch nonzero; "
          f"mat-u1 {int(u1[keep].sum())}, mat-all {int(allu[keep].sum())} spikes")

    print(f"\n  {'features':<40} {'80/20 split':>12} {'5-fold CV':>18}")
    for name, X in sets:
        s_acc, cv_m, cv_s = fit_score(X, y, args.seed, args.shrinkage)
        print(f"  {name:<40} {100 * s_acc:11.1f}% {100 * cv_m:11.1f} +- {100 * cv_s:4.1f}%")

    rng = np.random.default_rng(args.seed)
    s_acc, cv_m, cv_s = fit_score(feats, rng.permutation(y), args.seed, args.shrinkage)
    print(f"  {'model, labels shuffled (chance)':<40} {100 * s_acc:11.1f}% {100 * cv_m:11.1f} +- {100 * cv_s:4.1f}%")


if __name__ == "__main__":
    main()

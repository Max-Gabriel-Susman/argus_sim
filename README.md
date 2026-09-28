# argus_sim/tools — feature extraction: model, data, and validation

**Repo:** `argus_sim`. These scripts are how the codec's DSP was designed and
validated before any of it became VHDL. They are standalone Python (h5py,
numpy, scipy, scikit-learn; all installed by `rosdep` for this workspace) and
run on files in `~/argus_data/`.

| Script | Does |
| --- | --- |
| `nwb_to_replay.py` | broadband NWB → raw uint16 `.bin` of RHD2132 codes, for `dataset_relay_node` and for the model |
| `spike_features.py` | the bit-exact fixed-point model of the fabric's feature extraction; writes the GHDL golden |
| `validate_features.py` | model crossings vs the lab's spike sorting, same window, same clock |
| `decode_test.py` | model features vs the lab's features, through `inference_node`'s classifier |

## The result

On session `indy_20161005_06` (macaque M1, 96-channel Utah array, self-paced
reaching), the fixed-point feature pair the fabric will compute — threshold
crossings at 3.5σ and spike-band power, per channel per 50 ms bin — decodes
reach direction better than any feature set derived from the lab's own spike
sorting. Full session, 7,451 bins, four classes, StandardScaler → LDA with
shrinkage, 5-fold CV:

| Features | Accuracy | Events |
| --- | --- | --- |
| **model: crossings + power, 3.5σ** | **53.5 ± 2.0 %** | 65,124 |
| lab, every unit incl. hash | 49.9 ± 1.5 % | 310,777 |
| lab, unit 1 only (what `inference_node` trains on) | 43.4 ± 1.2 % | 81,949 |
| chance (majority class) | 34.4 % | — |

Nineteen points over chance with a fifth of the lab's events. The absolute
numbers are low for everyone because the label — instantaneous cursor-to-target
quadrant on a self-paced task, no history — is noisy; the comparison is what
matters, and the fabric's features win it.

Reproduce:

```bash
T=~/Documents/argus_ws/src/argus_sim/tools
python3 $T/nwb_to_replay.py ~/argus_data/indy_20161005_06_broadband.nwb \
    --start 10 --seconds 374 --no-resample \
    --out ~/argus_data/indy_20161005_06_s10_374s_24k.bin
python3 $T/decode_test.py ~/argus_data/indy_20161005_06_s10_374s_24k.bin \
    ~/argus_data/indy_20161005_06.mat --start 10 --t0 1278.0 --fs 24414.0625 \
    --mult 3.5 --features both --shrinkage
```

About two and a half minutes; the model's features are cached beside the
`.bin`, so classifier variants after that are instant.

To run that model live, add `--save-model PATH`. It pickles the pipeline for
the chosen `--features`, fitted on every usable bin, together with a small
dict describing the feature layout it expects (`features`, `channels`,
`bin_len`, `mult`, `fs`, `bin_s`, `power_units`). Power goes into the saved
model as mean-square (the per-bin sum ÷ `bin_len`), which is what the wire
carries (see below) and what makes a model fitted on 1221-sample native-rate
bins valid on the fabric's 1500-sweep bins. Point `inference_node` at it:

```bash
python3 $T/decode_test.py ... --mult 3.5 --features both --save-model ~/argus_model.pkl
ARGUS_MODEL_PATH=~/argus_model.pkl ros2 run argus_inference inference_node
```

`inference_node` then builds `[counts..., power...]` from each `NeuralFrame`
instead of training on the `.mat` at startup, and logs which path is active.

## The parameters, as locked

These are the RTL generics. The arithmetic they parameterise is the header
docstring of `spike_features.py`, line for line; change neither without the
other.

| Generic | Value | Meaning |
| --- | --- | --- |
| `B0`, `A1` | 31932, 31096 | first-order 250 Hz high-pass at 30012 Hz, Q1.15, rounded |
| `MULT_NUM`, `MULT_SHIFT` | 49, 2 | threshold² = 12.25 × mean-square, i.e. 3.5σ |
| `K`, `K_FAST` | 15, 8 | mean-square EMA shift while tracking / during the first 2^K samples |
| `REFRAC` | 30 | samples after a crossing before another counts (1 ms) |
| `WARMUP` | 32768 | samples before crossings count |
| `WINSOR` | 1 | EMA input is `min(sq, T)` once tracking |
| `BIN` | 1500 | samples per bin (50 ms) |
| order | 1 | one section; `--hp-order 2` cascades it and did not help |
| polarity | negative | `--bipolar` did not help |

The golden the testbench compares against, on the 10 s segment at the fabric
rate:

```bash
python3 $T/spike_features.py ~/argus_data/indy_20161005_06_s120_10s.bin --mult 3.5 \
    --golden ~/argus_data/indy_20161005_06_s120_10s.golden.txt \
    --trace  ~/argus_data/indy_20161005_06_s120_10s.trace.txt
```

The golden's header line records every parameter above. `tb_argus_feature`
must reproduce every `(bin, channel, count, power)` line exactly; the model
is bit-exact, so any mismatch is a bug in one of the two.

There is no `--stim` any more: the bench reads the raw `.bin` itself as its
stimulus, so a golden and its stimulus are one file plus this script's
output. CI runs the same check on a small pair checked into
`argus-neural-codec/sim/data/`: the first 6000 sweeps of this segment and
their golden at a 2048-sweep warm-up and 100-sweep bins, 5760
`(bin, channel)` pairs:

```bash
head -c $((6000*96*2)) ~/argus_data/indy_20161005_06_s120_10s.bin > sim/data/feature_ci.dat
python3 $T/spike_features.py sim/data/feature_ci.dat \
    --mult 3.5 --ms-shift 11 --warmup 2048 --bin 100 \
    --golden sim/data/feature_ci_golden.txt
```

Every derived file and the command that makes it is listed in the
[argus_data](https://github.com/Max-Gabriel-Susman/argus_data) repository
(`scripts/derive.sh`).

## How the parameters were chosen — and the wrong turn

The first validation compared the model's per-channel crossing rates against
the lab's spike sorter (`validate_features.py`). The model found 18% of the
lab's spikes, undercounting the busiest channels 3–5×, with a Spearman of
0.42 across live channels. Four hypotheses were eliminated in ten-second
runs: lowering the threshold to 3.5σ added crossings that did not track the
sorter, winsorising the RMS estimate moved it 2%, a second filter section
removed real signal, and counting positive crossings found 56 events in 96
channels. The remaining explanation is the correct one: the sorter recovers
small units by template matching that no threshold separates from noise.

That comparison was the wrong bar. A threshold detector is not a sorter and
Willett's result is that it does not need to be. `decode_test.py` asks the
question the fabric actually has to answer, and under it the picture
inverted:

| Threshold | Counts only | Counts + power |
| --- | --- | --- |
| 4.5σ | 41.6 % | — |
| 3.5σ | 43.6 % | **52.7 %** |
| 3.0σ | 46.5 % | — |

(200 s window; lab unit-1 44.4 %, lab all 50.1 %.) Accuracy rises with
event count — the crossings that failed the sorter test are decodable
multi-unit activity — and spike-band power adds nine points on top, catching
sub-threshold activity the way the lab's hash does without the lab's low
threshold. The same lever that failed one metric passed the other. The
metric was the bug.

Two real defects *were* found on the way, both by the synthetic ground-truth
test in `spike_features.py`, both floor truncation inside a recursion: a −10
code DC bias in the high-pass and a −2^(K−1) bias in the mean-square that
silently turned 4.5σ into 3.5σ. Each fix is a constant added before a shift.
Neither would have been visible from RTL.

## What power forces on the wire

Power is half the accuracy, so the fabric emits it and the firmware sends it.
Crossing counts fit `uint16[96] channels` (a busy channel is single digits
per bin). Power does not: the fabric accumulates a 48-bit sum of squares per
bin, which reaches 1.3e11 on this session.

The fabric's feature bank holds `count` (16 bits) and the raw sum (48 bits)
per channel, and wire frame version 3 (`argus_wire.h`, 594 bytes) carries
`uint32_t power[96]` after `channels`: `power[c] = sum / BIN`, mean squared
spike-band voltage in ADC code², Willett's `spikePow` up to a constant, at
most ~1.1e8 here. `NeuralFrame` has `uint32[96] power` to match, and
`neural_udp_receiver` accepts version 3 only. Within one bin length
`StandardScaler` makes sum-vs-mean irrelevant; across bin lengths (a model
fitted at 24,414 Hz, run on the fabric's 30,012 Hz bins) the mean is what
keeps the scale right, so `--save-model` fits on it too.

## Notes on the data path

`nwb_to_replay.py` processes in chunks under `--no-resample`, with the
filter state carried across chunk boundaries (verified bit-identical to a
single pass), so a full session costs the memory of a ten-second segment.
Resampling to the fabric rate loads the segment whole — `resample_poly`
carries no state — and says how much memory that is first. For the model
and the decode test the native 24,414 Hz is fine, with `--fs 24414.0625`
and 1221-sample bins; the fabric-rate `.bin` is for the relay and the GHDL
testbench.

`spike_features.py` streams a uint16 memmap row by row and keeps the
filtered signal only for the float cross-check and `--trace`, so it runs on
the full session in about two minutes at ~1.8× real time.

The broadband and the `.mat` share a session clock: broadband `t0 = 1278.0`,
behaviour 1288–1662 s. Every window here is `--start` seconds into the
broadband, and `validate_features.py` / `decode_test.py` take `--t0` to put
it on that clock.

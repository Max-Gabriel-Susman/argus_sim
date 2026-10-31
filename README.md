# argus_sim

The workstation side of the Argus replay rig, and the Python tools that
designed and validate the fabric's feature extraction.

The relay, `dataset_relay_node`, serves a recorded dataset to the board over
UDP :5010, one half-buffer at a time, while the board plays it into the
fabric's simulated amplifier chips. It is written in Rust. Version 2.0.0
replaced the C++ relay of 1.x, and the board cannot tell the two apart: the
same bytes on the wire for every request, the same parameters, topic, log
lines and exit codes. The package also builds `replay_client_test`, which
checks a running relay with the client the board runs, and a Python module,
`argus_sim_py`, with the protocol and the relay in it (see
[Python bindings](#python-bindings)).

The whole stack, and the one command that runs it on the board, is described in
[argus_bringup/README.md](https://github.com/Max-Gabriel-Susman/argus_bringup/blob/main/README.md).

## Building

The package is `ament_cargo`: colcon builds it through `colcon-ros-cargo`,
which runs Cargo. It needs:

- ROS 2 Humble from apt, upgraded to a sync that has the Rust message
  bindings (`/opt/ros/humble/share/std_msgs/rust`); `rclrs` and `ros-env`
  find them there;
- `colcon-ros-cargo` (`pip3 install colcon-ros-cargo`);
- Rust 1.97.0 with clippy and rustfmt. `rust-toolchain.toml` names it, so
  `rustup` uses it for everything built in this directory;
- `argus_core` in the same workspace, built first. It is the replay
  protocol's definition for the ROS packages; `argus_sim.repos` names it.

```bash
cd ~/Documents/argus_ws
vcs import src < src/argus_sim/argus_sim.repos   # once, if argus_core is missing
source /opt/ros/humble/setup.bash
colcon build --packages-select argus_core argus_sim
source install/setup.bash
```

The wire format and the board's replay client are not in this repository:
`argus-wire` and `argus-replay` come from the firmware's workspace
([argus_safety_controller](https://github.com/Max-Gabriel-Susman/argus_safety_controller),
tag `v2.1`, the firmware the board runs), so the structs, the constants
and the CRC exist once. `Cargo.lock` is committed and every build uses it.

## Running

```bash
ros2 run argus_sim dataset_relay_node --ros-args \
    -p dataset_path:=$HOME/argus_data/indy_20161005_06_s120_10s.bin
ros2 topic echo /dataset_relay/status
```

On the board's network the launch file starts it with
`config/argus.yaml` from argus_bringup and the dataset path. The node is
`dataset_relay` and takes these parameters:

| Parameter | Type | Default | Meaning |
| --- | --- | --- | --- |
| `bind_addr` | string | `0.0.0.0` | the address to listen on |
| `port` | integer | 5010 | the UDP port (taken modulo 65536, as the C++ cast did) |
| `dataset_path` | string | `""` | the `.bin` to serve; empty serves the synthetic identity pattern |
| `synthetic_samples` | integer | 30000 | the pattern's length, when there is no file |
| `channel_count` | integer | 96 | channels per sample row, 1 to 4096 |
| `loop` | bool | true | wrap at the end of the dataset for every request, not only those flagged LOOP |
| `drop_mask` | integer | 0 | fault injection: bit `i` withholds chunk `i` of every first attempt (modulo 2^32) |
| `status_period_s` | double | 2.0 | how often `~/status` is published, at least 0.1 s |

`~/status` is a `std_msgs/String` with the counters:
`requests=N retransmits=N chunks=N dropped=N bytes=N bad=N wraps=N offset=N peer=IP`
(`peer=none` until the first request). A startup error is logged as FATAL
with the C++ relay's text and exits 1; SIGINT or SIGTERM stops the server
and exits 0.

To check a running relay serving the synthetic pattern at its default size:

```bash
ros2 run argus_sim replay_client_test 127.0.0.1 5010
```

It makes the board's three fetches (offset 0, offset 1000, and one across
the end of the dataset), checks every sample against the pattern, prints
its counters, and exits 0 on PASS.

## Testing

```bash
source install/setup.bash          # from the workspace, for rclrs
cd src/argus_sim
cargo fmt --all -- --check
cargo clippy --locked --workspace --all-targets -- -D warnings
cargo test --locked
```

`cargo test -p argus-sim-core` needs no ROS at all. `colcon test
--packages-select argus_sim` runs the root package's tests and its format
check. Every test binds 127.0.0.1 on a port the kernel picks and finishes in
seconds. The Python bindings have their own tests, below.

## How it is built

Three crates, one Cargo workspace, and the repository root is also the ROS
package.

**The relay core** (`crates/argus-sim-core`) is the relay without ROS, so
its tests run on any host:

- `wire` checks a request and builds the chunks of a reply on argus-wire's
  structs and CRC, redefining nothing. A request is refused, in the C++
  order, for its size, magic, version, type, CRC or a zero sample count;
  a chunk's CRC covers its header up to the `crc` field and then the
  payload.
- `source` is the dataset: a `.bin` of rows of little-endian `u16`, read
  with `pread` (no mapping, so no unsafe code; the page cache keeps the file
  in memory as the mapping did), or the synthetic identity pattern, in which
  every sample says which chip, channel and sample it is. A read wraps at
  the end.
- `server` is the UDP server: one thread of its own, a blocking socket with
  a 200 ms receive timeout so `stop()` is seen promptly, one transmit buffer
  reused for every chunk, and the rows read straight into it. Requests are
  answered in arrival order and none expires: one that waited while the
  process was stopped is answered like any other, which is what the board's
  stall test checks. Each request is answered from its own fields alone, so
  the server keeps no per-client state.

**The node** (`src/bin/dataset_relay_node.rs`) is a thin rclrs wrapper:
parameters in, the server started, `~/status` on a timer, and a `ctrlc`
handler that stops spinning on SIGINT or SIGTERM (rclrs installs none). The
server never runs on the executor: the board works to a millisecond budget
per buffer swap, and an executor shared with a timer is not the place for
that. When a params file comes before a `-p`, the node applies the `-p`
last, as the C++ node did; rclrs alone would let the file win.

**The test client** (`src/bin/replay_client_test.rs`) runs `argus-replay`,
the board's own client, over a host UDP socket, so a pass means the relay
satisfies the code that will talk to it on the board.

**The bindings** (`crates/argus-sim-py`) wrap the core for Python and add no
protocol logic of their own.

No crate in the repository writes unsafe code: the core and both
executables `forbid` it, socket options come from `socket2`, and the C
client the tests compare against comes from the firmware's
`argus-difftest`.

### Parity with the C++ relay

The C++ relay is gone from the tree; these are how the Rust one is held to
it:

- `crates/argus-sim-core/tests/golden.rs` replays `testdata/relay_corpus.json`,
  412 requests under three configurations (looping, not looping, and
  `drop_mask` 0x808) whose replies were recorded once from the C++ relay by
  `tools/record_golden.py`, and compares every reply byte for byte. The
  corpus covers the edges of the dataset, every count boundary, every flag,
  every kind of malformed request and 300 seeded random ones. It is the
  reference and is never re-recorded from this relay.
- `tests/loopback.rs` (in the core) runs the board's Rust client and the C
  header it was ported from against the relay; both must fill their buffers
  identically and count the same. With `drop_mask` 0x808 both report
  `retransmits=6 timeouts=6 accepted=63 rejected=0`, as the C++ pair did.
- `tests/replay_client_test.rs` runs the executable against an in-process
  relay, so `colcon test` covers the client end to end.
- Two examples in `crates/argus-sim-core/examples` need a built C++ relay
  and run by hand (their doc comments give the command). `replay_diff` runs
  both nodes side by side and compares every datagram and their status
  lines; `replay_load` times both
  under the board's load and stalls the Rust one as the board's stall test
  does.

Measured before the C++ relay was removed, on loopback:

- the corpus: 0 differences in each configuration (5009, 4205 and 4729
  replies);
- side by side, 1032 requests per round (the corpus, 120 at the dataset's
  end, 500 random) on the synthetic pattern, the 10 s `.bin` looping and the
  `.bin` not looping: 0 differences in 12081, 12081 and 10483 datagrams, and
  equal status lines;
- the board's load, 147 samples per request for 30 s, request to last chunk
  in ms:

| Relay | Rate | Requests | p50 | p99 | max |
| --- | --- | --- | --- | --- | --- |
| C++ | 204/s | 6120 | 0.541 | 0.769 | 0.993 |
| C++ | 408/s | 12240 | 0.378 | 0.712 | 1.233 |
| Rust | 204/s | 6120 | 0.526 | 0.786 | 1.606 |
| Rust | 408/s | 12240 | 0.365 | 0.722 | 2.664 |

- the stall drill, the Rust relay stopped for 50 ms every 5 s for 30 s at
  204/s: 6120 requests, 6120 answered, all complete and in order.

The maxima move by a millisecond or two between runs on either relay, with
the host's scheduling; p50 and p99 are steady.

### Kept from the C++ on purpose

Some C++ behavior looks wrong and is kept, each pinned by a test, so that
2.0.0 changes nothing the board, the launch or a log reader sees: the
typos in two error texts (`is not a multiples of`, `sythetic source`),
`replay_client_test` printing `0x` before a decimal value in one failure
line, the server thread ending quietly on an unexpected receive error while
the node keeps running, and `port` and `drop_mask` narrowed without an
error. Two cases where the C++ crashed are now ordinary errors: a negative
`synthetic_samples` is refused with the synthetic source's message, and a
non-numeric port for `replay_client_test` prints `bad port` and exits 1.

## The tools: feature extraction: model, data, and validation

These scripts are how the codec's DSP was designed and validated before any
of it became VHDL. They are standalone Python (h5py, numpy, scipy,
scikit-learn; all installed by `rosdep` for this workspace) and run on files
in `~/argus_data/`.

| Script | Does |
| --- | --- |
| `nwb_to_replay.py` | broadband NWB → raw uint16 `.bin` of RHD2132 codes, for `dataset_relay_node` and for the model |
| `spike_features.py` | the bit-exact fixed-point model of the fabric's feature extraction; writes the GHDL golden |
| `validate_features.py` | model crossings vs the lab's spike sorting, same window, same clock |
| `decode_test.py` | model features vs the lab's features, through `inference_node`'s classifier |
| `hw_bitexact.py` | the board's NeuralFrames during a replay run vs this model on the same `.bin`, exactly |

### The result

On session `indy_20161005_06` (macaque M1, 96-channel Utah array, self-paced
reaching), the fixed-point feature pair the fabric computes (threshold
crossings at 3.5σ and spike-band power, per channel per 50 ms bin) decodes
reach direction better than any feature set derived from the lab's own spike
sorting. Full session, 7,451 bins, four classes, StandardScaler → LDA with
shrinkage, 5-fold CV:

| Features | Accuracy | Events |
| --- | --- | --- |
| **model: crossings + power, 3.5σ** | **53.5 ± 2.0 %** | 65,124 |
| lab, every unit incl. hash | 49.9 ± 1.5 % | 310,777 |
| lab, unit 1 only (what `inference_node` trains on without a saved model) | 43.4 ± 1.2 % | 81,949 |
| chance (majority class) | 34.4 % | - |

Nineteen points over chance with a fifth of the lab's events. The absolute
numbers are low for everyone because the label (instantaneous cursor-to-target
quadrant on a self-paced task, no history) is noisy; the comparison is what
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
On the board, `ros2 launch argus_bringup argus.launch.py program:=true
model:=$HOME/argus_model.pkl` does the same. On 2026-09-28 the saved model
decoded the fabric's live features, and all four intents occurred over a
90 s run.

### The parameters, as locked

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

The same check on silicon is `hw_bitexact.py`. It records the frames the
receiver publishes during a hardware run, then compares their counts and
`sum / 1500` power with `run_fixed` on the replay `.bin`, starting from
sample 0 at the soft reset that begins streaming:

```bash
python3 $T/hw_bitexact.py capture /tmp/cap.npz --seconds 170 &
~/Documents/argus_ws/src/argus_bringup/scripts/hwtest.sh --seconds 90
python3 $T/hw_bitexact.py compare ~/argus_data/indy_20161005_06_s120_10s.bin /tmp/cap.npz
```

On 2026-09-28 the first 100 bins matched 100%, and so did the whole
capture: 1450 bins, seven loops of the file, 139200 counts and 139200
powers. The replay must be underrun-free over the compared window. Check
that `underruns` holds still in the `stream:` lines of the hwtest log.

Every derived file and the command that makes it is listed in the
[argus_data](https://github.com/Max-Gabriel-Susman/argus_data) repository
(`scripts/derive.sh`).

### How the parameters were chosen, and the wrong turn

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
| 4.5σ | 41.6 % | - |
| 3.5σ | 43.6 % | **52.7 %** |
| 3.0σ | 46.5 % | - |

(200 s window; lab unit-1 44.4 %, lab all 50.1 %.) Accuracy rises with
event count (the crossings that failed the sorter test are decodable
multi-unit activity), and spike-band power adds nine points on top, catching
sub-threshold activity the way the lab's hash does without the lab's low
threshold. The same lever that failed one metric passed the other. The
metric was the bug.

Two real defects *were* found on the way, both by the synthetic ground-truth
test in `spike_features.py`, both floor truncation inside a recursion: a −10
code DC bias in the high-pass and a −2^(K−1) bias in the mean-square that
silently turned 4.5σ into 3.5σ. Each fix is a constant added before a shift.
Neither would have been visible from RTL.

### What power forces on the wire

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

### Notes on the data path

`nwb_to_replay.py` processes in chunks under `--no-resample`, with the
filter state carried across chunk boundaries (verified bit-identical to a
single pass), so a full session costs the memory of a ten-second segment.
Resampling to the fabric rate loads the segment whole (`resample_poly`
carries no state) and says how much memory that is first. For the model
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

## Python bindings

`argus_sim_py` is the replay protocol and the relay as a Python module,
built from the same Rust as `dataset_relay_node` (`crates/argus-sim-py`).
It is one abi3 wheel for Python 3.10 and later. Install it with one command
from the repository:

```bash
tools/install_bindings.sh
```

That needs maturin 1.16 on `PATH`. It builds the wheel in release, installs
it into the user site of `/usr/bin/python3` (the interpreter the launch file
and the tools run under), then imports it and prints its version and where
it loaded from. Run it again after changing the bindings.

What the module offers (`argus_sim_py.pyi` documents every item):

| Name | Is |
| --- | --- |
| `MAGIC`, `VERSION`, `MSG_REQUEST`, `MSG_CHUNK`, `F_LOOP`, `F_RETRANSMIT`, `SAMPLES_PER_HALF`, `MAX_PAYLOAD`, `REQUEST_LEN`, ... | the wire constants, from `argus_wire.h` through `argus-wire` |
| `crc16(data, crc=0xFFFF)` | the CRC-16/CCITT-FALSE every datagram carries |
| `encode_request`, `decode_request`, `Request` | a 22-byte replay request, built or checked as the relay checks it |
| `encode_chunk`, `decode_chunk`, `Chunk` | one chunk of a reply, built or checked |
| `WireError` | a `ValueError` naming the check that failed in `.kind` |
| `ReplaySource.from_file(path, channel_count=96)`, `ReplaySource.synthetic(n_samples, channel_count=96)` | a dataset, and `.read(sample_offset, count=147)`: the rows the relay would send, wrapping at the end |
| `Relay(source, port=0, bind_addr="127.0.0.1", loop=True, drop_mask=0)` | the relay, on its own thread in this process: `start()`, `stop()`, `.port`, `.samples_per_chunk`, `stats()`, and a context manager |

Errors are exceptions: `WireError` for a datagram the protocol does not
allow, `OSError` for files and sockets (with the relay's own messages),
`ValueError` for bad arguments. The relay releases the GIL while it starts
and stops, and its thread never needs it.

Serve the synthetic pattern on a port the kernel picks, fetch one half of
the board's buffer the way the board does, and check it:

```python
import socket

import argus_sim_py as asp

source = asp.ReplaySource.synthetic(30000)
with asp.Relay(source) as relay, socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
    sock.settimeout(1.0)
    request = asp.encode_request(seq=1, sample_offset=0, sample_count=asp.SAMPLES_PER_HALF)
    sock.sendto(request, ("127.0.0.1", relay.port))
    first = asp.decode_chunk(sock.recv(asp.MAX_CHUNK_LEN))
    rest = [asp.decode_chunk(sock.recv(asp.MAX_CHUNK_LEN)) for _ in range(first.chunk_total - 1)]
    half = b"".join(chunk.payload for chunk in [first, *rest])
    assert half == source.read(0, asp.SAMPLES_PER_HALF)
    print(first.chunk_total, "chunks,", relay.stats()["bytes_sent"], "bytes")
```

That prints `21 chunks, 28812 bytes`: 147 samples at 7 rows a chunk.

The tests are pytest under `crates/argus-sim-py/pytests`. They check the
module against the C++ relay's recorded replies in
`testdata/relay_corpus.json`, byte for byte. Run them against a wheel built
into a scratch directory, so nothing installed changes:

```bash
maturin build --release -m crates/argus-sim-py/Cargo.toml --out /tmp/wheels \
    --compatibility linux --interpreter python3
python3 -m zipfile -e /tmp/wheels/argus_sim_py-*.whl /tmp/argus_sim_py
PYTHONPATH=/tmp/argus_sim_py python3 -m pytest -p no:cacheprovider crates/argus-sim-py/pytests
```

The scripts in `tools/` do not use the module, on purpose. None of them
speaks the replay protocol; the ones that read a `.bin` (`spike_features.py`,
`decode_test.py`, `hw_bitexact.py`) map it with `np.memmap`, which already reads
rows on demand without loading the file, as `ReplaySource` would; and `hw_bitexact.py` runs inside hwtest's
`bitexact` stage, which must not depend on a wheel being installed.
`record_golden.py` builds its requests in plain Python (`struct` and a
bitwise CRC) so that the corpus it records is independent of the code it
checks.

## Documentation

Two references, one per language.

The Rust is documented with rustdoc: every public item has a doc comment,
and each module opens with what it is for. Start at `argus_sim_core`, whose
page lists the modules; `server::ReplayServer` is the request path, and the
executables' pages say what they print, log and publish. The workspace's
lints warn on a missing doc comment, and CI builds the docs with warnings as
errors.

```bash
source install/setup.bash          # the node's page needs rclrs
cd src/argus_sim
cargo doc --workspace --no-deps --open
```

The bindings' reference is their docstrings, which `help(argus_sim_py)`
shows, and the stub `crates/argus-sim-py/argus_sim_py.pyi` that ships in
the wheel.

The Python tools are documented by Doxygen, which reads `tools/` into a
site with this README as its front page. Each tool is a namespace page: its
functions with their signatures, its module constants with their values
(`CHANNELS`, `RHD_ZERO`, the cascade scale), and its module docstring shown
as written, because those docstrings are the tools' `--help` text and read
best verbatim.

```bash
sudo apt install doxygen graphviz
doxygen
xdg-open build/html/index.html
```

`Doxyfile` lists only what differs from the defaults, each setting with its
reason. Output lands in `build/`, which is gitignored and is not where
colcon builds. The `docs` job in CI runs it with warnings as errors. For the
Python, the docstring is the documentation as it stands, and Doxygen shows
it unchanged, so there is no second syntax to learn.

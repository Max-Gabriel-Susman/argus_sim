#!/usr/bin/env python3
"""
record_golden.py -- record a relay's replies to a fixed corpus of requests.

The Rust relay must answer every request byte for byte as the C++ relay
did. This script is how that reference was taken: it starts a relay binary
once per configuration, sends it a fixed, seeded corpus of replay requests
over UDP on loopback, and writes what came back to a JSON file. The test
crates/argus-sim-core/tests/golden.rs replays the same requests against the
Rust relay and compares every reply, so the parity check outlives the C++.

    python3 tools/record_golden.py \
        <prefix>/lib/argus_sim/dataset_relay_node 26020 \
        --out testdata/relay_corpus.json

Run it with the ROS environment the binary needs already sourced. The relay
is started with `--ros-args -p bind_addr:=127.0.0.1 -p port:=PORT` plus the
configuration's parameters, and serves its synthetic identity pattern
(30000 samples x 96 channels, the node's defaults). Each configuration gets
a fresh relay, stopped with SIGTERM afterwards.

The requests are built here, with struct and a bitwise CRC, independently
of the code under test. A request's reply is every datagram that arrives
until 50 ms pass with none. Per request the file keeps the number of
replies, the length of each (as runs of equal lengths) and a SHA-256 over
them in order; for the first 20 requests of each configuration it keeps
the replies themselves, so a mismatch can be shown field by field. The
corpus starts with requests that are rejected or answered with one chunk,
so those 20 stay small.

Only the standard library: the reference must not depend on anything the
Rust side also uses.
"""

import argparse
import hashlib
import json
import os
import random
import socket
import struct
import subprocess
import sys
import tempfile
import time

# The wire format, from argus_wire.h.
MAGIC = 0x41524753          # ARGUS_FRAME_MAGIC, "ARGS" little-endian
VERSION = 1                 # ARGUS_REPLAY_VERSION
MSG_REQUEST = 0x0010        # ARGUS_MSG_REPLAY_REQUEST
MSG_CHUNK = 0x0011          # ARGUS_MSG_REPLAY_CHUNK
F_RETRANSMIT = 0x0001       # ARGUS_REPLAY_F_RETRANSMIT
F_LOOP = 0x0002             # ARGUS_REPLAY_F_LOOP

# The request without its CRC: magic, version, type, seq, sample_offset,
# sample_count, flags. Packed, little-endian; the CRC follows as a u16.
REQUEST_BODY = struct.Struct("<IHHIIHH")

# The relay's synthetic source, at the node's defaults.
N_SAMPLES = 30000
CHANNELS = 96

# How long without a datagram ends a reply.
QUIET_S = 0.050

# Requests whose replies are kept whole.
WHOLE_REPLIES = 20

# How long a relay gets to come up.
STARTUP_S = 10.0

# The random part of the corpus: fixed, so the corpus is reproducible.
SEED = 20161005
RANDOM_REQUESTS = 300

# One relay per configuration: a name and the parameters it adds.
CONFIGS = [
    ("loop", {"loop": "true", "drop_mask": "0"}),
    ("no_loop", {"loop": "false", "drop_mask": "0"}),
    ("drop_0x808", {"loop": "true", "drop_mask": "2056"}),
]


def crc16(data, crc=0xFFFF):
    """CRC-16/CCITT-FALSE, bit by bit: poly 0x1021, no reflection."""
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else (crc << 1)
            crc &= 0xFFFF
    return crc


def request(seq, offset, count, flags=0, magic=MAGIC, version=VERSION,
            msg_type=MSG_REQUEST):
    """A 22-byte request with a correct CRC over its first 20 bytes."""
    body = REQUEST_BODY.pack(magic, version, msg_type, seq, offset, count,
                             flags)
    return body + struct.pack("<H", crc16(body))


def corpus():
    """Every request, in the order they are sent."""
    n = N_SAMPLES
    reqs = []

    # Rejected: the wrong length (one short, one long, and one longer than
    # the relay's 512-byte receive buffer), each header field wrong in turn
    # with the CRC still right, the CRC wrong, and a request for nothing.
    valid = request(1, 0, 147)
    reqs.append(valid[:21])
    reqs.append(valid + b"\x00")
    reqs.append(valid + bytes(513 - len(valid)))
    reqs.append(request(2, 0, 147, magic=MAGIC ^ 1))
    reqs.append(request(3, 0, 147, version=VERSION + 1))
    reqs.append(request(4, 0, 147, msg_type=MSG_CHUNK))
    bad_crc = bytearray(request(5, 0, 147))
    bad_crc[20] ^= 0x01
    reqs.append(bytes(bad_crc))
    reqs.append(request(6, 0, 0))

    # Around the end of the dataset with counts of one row, two, one full
    # chunk and one more than a chunk, under every flag combination. The
    # first three counts end the first 20 requests, whose replies are kept
    # whole.
    edges = [n - 1, n, n + 1, 2 * n + 5]
    flag_sets = [0, F_LOOP, F_RETRANSMIT, F_LOOP | F_RETRANSMIT]
    seq = 100
    for count in (1, 2, 7, 8):
        for i, offset in enumerate(edges):
            reqs.append(request(seq, offset, count, flag_sets[i]))
            seq += 1

    # The host test client's three fetches: from the start, at an offset,
    # and across the end (73 rows before it, 74 after).
    for offset in (0, 1000, n - 73):
        reqs.append(request(seq, offset, 147))
        seq += 1

    # Counts on both sides of the 147-sample cap and of a chunk, at the
    # start and 100 samples before the end, under every flag combination.
    for offset in (0, n - 100):
        for count in (1, 7, 8, 146, 147, 148, 65535):
            for flags in flag_sets:
                reqs.append(request(seq, offset, count, flags))
                seq += 1

    # A full half at each edge, under every flag combination.
    for offset in edges + [0xFFFFFFFF]:
        for flags in flag_sets:
            reqs.append(request(seq, offset, 147, flags))
            seq += 1

    # The seq field's extremes, and flag bits the relay does not know.
    for s in (0, 1, 0x7FFFFFFF, 0x80000000, 0xFFFFFFFF):
        reqs.append(request(s, 500, 20))
    for flags in (0x0004, 0x8000, 0xFFFF, 0xFFFE):
        reqs.append(request(seq, n - 3, 10, flags))
        seq += 1

    # Seeded random valid requests: mostly inside the dataset and up to a
    # half, some past its end or larger than the cap.
    rng = random.Random(SEED)
    for _ in range(RANDOM_REQUESTS):
        if rng.random() < 0.7:
            offset = rng.randrange(n)
        else:
            offset = rng.randrange(n - 200, 3 * n)
        if rng.random() < 0.9:
            count = rng.randint(1, 147)
        else:
            count = rng.randint(148, 65535)
        flags = rng.choice(flag_sets)
        reqs.append(request(rng.getrandbits(32), offset, count, flags))
    return reqs


def runs(lengths):
    """Lengths as [length, repeat] pairs: a 147-sample reply is twenty
    full chunks and a short one, [[1372, 20], [1176, 1]]."""
    pairs = []
    for length in lengths:
        if pairs and pairs[-1][0] == length:
            pairs[-1][1] += 1
        else:
            pairs.append([length, 1])
    return pairs


def wait_for_banner(log_path, proc):
    """Waits until the relay logs that it is serving, or fails."""
    deadline = time.monotonic() + STARTUP_S
    while time.monotonic() < deadline:
        if proc.poll() is not None:
            break
        with open(log_path, "r", errors="replace") as log:
            if "replay server on" in log.read():
                return
        time.sleep(0.05)
    with open(log_path, "r", errors="replace") as log:
        sys.stderr.write(log.read())
    raise SystemExit("the relay did not come up")


def collect(sock):
    """Every datagram until QUIET_S passes without one."""
    replies = []
    sock.settimeout(QUIET_S)
    while True:
        try:
            replies.append(sock.recv(65536))
        except socket.timeout:
            return replies


def record_config(relay, port, name, params, reqs):
    """Starts the relay with `params`, sends every request, stops it."""
    args = [relay, "--ros-args",
            "-p", "bind_addr:=127.0.0.1", "-p", f"port:={port}"]
    for key, value in params.items():
        args += ["-p", f"{key}:={value}"]

    with tempfile.TemporaryDirectory() as tmp:
        log_path = os.path.join(tmp, "relay.log")
        with open(log_path, "wb") as log:
            proc = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT)
        try:
            wait_for_banner(log_path, proc)
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 22)
            sock.bind(("127.0.0.1", 0))
            results = []
            for i, req in enumerate(reqs):
                sock.sendto(req, ("127.0.0.1", port))
                replies = collect(sock)
                result = {
                    "n": len(replies),
                    "len": runs([len(r) for r in replies]),
                    "sha256": hashlib.sha256(b"".join(replies)).hexdigest(),
                }
                if i < WHOLE_REPLIES:
                    result["data"] = [r.hex() for r in replies]
                results.append(result)
            sock.close()
        finally:
            proc.terminate()
            proc.wait(timeout=10)
    print(f"{name}: {len(reqs)} requests, "
          f"{sum(r['n'] for r in results)} replies", file=sys.stderr)
    return {"name": name, "params": params, "replies": results}


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("relay", help="the dataset_relay_node binary")
    parser.add_argument("port", type=int, help="the UDP port it binds")
    parser.add_argument("--out", required=True, help="the JSON file to write")
    opts = parser.parse_args()

    reqs = corpus()
    configs = [record_config(opts.relay, opts.port, name, params, reqs)
               for name, params in CONFIGS]
    doc = {
        "about": "Replies of the C++ dataset relay to a fixed request "
                 "corpus; written by tools/record_golden.py, checked by "
                 "crates/argus-sim-core/tests/golden.rs.",
        "source": {"synthetic_samples": N_SAMPLES, "channels": CHANNELS},
        "quiet_ms": int(QUIET_S * 1000),
        "requests": [r.hex() for r in reqs],
        "configs": configs,
    }
    with open(opts.out, "w") as out:
        out.write(dumps(doc))


def dumps(doc):
    """JSON with one request or one result per line: small, and a diff of
    two recordings points at the requests that changed."""
    compact = {"separators": (",", ":")}
    lines = ["{"]
    for key, value in doc.items():
        if key == "requests":
            lines.append(' "requests":[')
            lines.append(",\n".join(json.dumps(r) for r in value))
            lines.append(" ],")
        elif key == "configs":
            lines.append(' "configs":[')
            blocks = []
            for config in value:
                head = {k: v for k, v in config.items() if k != "replies"}
                body = ",\n".join(json.dumps(r, **compact)
                                  for r in config["replies"])
                blocks.append("  " + json.dumps(head, **compact)[:-1]
                              + ',"replies":[\n' + body + "\n  ]}")
            lines.append(",\n".join(blocks))
            lines.append(" ]")
        else:
            lines.append(f" {json.dumps(key)}:{json.dumps(value, **compact)},")
    lines.append("}")
    return "\n".join(lines) + "\n"


if __name__ == "__main__":
    main()

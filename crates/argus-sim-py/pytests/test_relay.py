"""The in-process relay: the corpus, drop_mask, starting and stopping, errors.

The relay is queried over a real UDP socket on loopback, as the board queries
the node, and its replies are compared byte for byte with the ones the C++
relay gave to the same requests (testdata/relay_corpus.json, recorded by
tools/record_golden.py).
"""

import errno
import hashlib
import json
import pathlib
import socket
import struct
import threading
import time

import pytest

import argus_sim_py as asp

REPO = pathlib.Path(__file__).resolve().parents[3]
CORPUS = REPO / "testdata" / "relay_corpus.json"

# How long to wait for one datagram the corpus says is coming, as golden.rs.
REPLY_TIMEOUT_S = 1.0
# argus_replay_chunk_hdr_t: magic, version, type, seq, sample_offset,
# sample_count, chunk_index, chunk_total, channel_count, payload_len, crc.
CHUNK = struct.Struct("<IHHIIHHHHHH")
# The server thread's name, as /proc shows it.
THREAD_NAME = "replay_server"


@pytest.fixture(scope="module")
def corpus():
    return json.loads(CORPUS.read_text(encoding="utf-8"))


@pytest.fixture(scope="module")
def source():
    return asp.ReplaySource.synthetic(30000, 96)


@pytest.fixture
def board():
    """The board's side: a socket on loopback that reads the relay's replies."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", 0))
    yield sock
    sock.close()


def receive(sock, expect, timeout=REPLY_TIMEOUT_S):
    """Up to `expect` datagrams, each within `timeout` of the one before."""
    sock.settimeout(timeout)
    replies = []
    while len(replies) < expect:
        try:
            replies.append(sock.recv(2048))
        except socket.timeout:
            break
    return replies


def quiet(sock, seconds):
    """True when nothing arrives for `seconds`."""
    return receive(sock, 1, seconds) == []


def fetch(sock, relay, seq, offset, count, flags=0):
    """Sends one request and returns its reply, decoded."""
    sock.sendto(asp.encode_request(seq, offset, count, flags), ("127.0.0.1", relay.port))
    chunks = []
    sock.settimeout(REPLY_TIMEOUT_S)
    while True:
        try:
            chunk = asp.decode_chunk(sock.recv(2048))
        except socket.timeout:
            return chunks
        chunks.append(chunk)
        if chunk.chunk_index + 1 == chunk.chunk_total and quiet(sock, 0.05):
            return chunks


def wait_for(relay, pred):
    """The relay's counters once `pred` holds of them. The relay counts a
    request after its last chunk has gone, so the chunks may arrive first."""
    deadline = time.monotonic() + REPLY_TIMEOUT_S
    while True:
        stats = relay.stats()
        if pred(stats) or time.monotonic() > deadline:
            return stats
        time.sleep(0.001)


def server_threads():
    """The relay threads in this process. Python's threading module does not
    see them: they are the core's, not Python's."""
    names = []
    for comm in pathlib.Path("/proc/self/task").glob("*/comm"):
        try:
            names.append(comm.read_text(encoding="utf-8").strip())
        except FileNotFoundError:
            pass  # the thread ended between the listing and the read
    return names.count(THREAD_NAME)


def settled_server_threads(want):
    """The relay threads once there are `want` of them, or after a second.
    /proc lags both ends of a thread: Rust names a thread from inside it, so
    for a moment after start() returns it still has its parent's name, and a
    joined thread is listed until the kernel has reaped it."""
    deadline = time.monotonic() + REPLY_TIMEOUT_S
    while True:
        count = server_threads()
        if count == want or time.monotonic() > deadline:
            return count
        time.sleep(0.001)


def port_is_free(port):
    """Whether a plain socket can bind `port`. The relay's socket sets
    SO_REUSEADDR and this one does not, so the bind fails while the relay's
    is open."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind(("127.0.0.1", port))
        return True
    except OSError:
        return False
    finally:
        sock.close()


# --- the corpus, against the C++ relay's replies ---------------------------


def describe(request):
    try:
        r = asp.decode_request(request)
    except asp.WireError as e:
        return f"{len(request)} bytes, rejected: {e.kind}"
    return f"seq={r.seq} offset={r.sample_offset} count={r.sample_count} flags={r.flags:#06x}"


def recorded_lengths(want):
    """The datagram lengths of a recorded reply, kept as [length, repeat]
    runs."""
    lengths = [length for length, repeat in want["len"] for _ in range(repeat)]
    assert len(lengths) == want["n"], "`n` disagrees with `len`"
    return lengths


def difference(got, want):
    """How `got` differs from the recorded reply `want`, or None."""
    lengths = [len(g) for g in got]
    if lengths != recorded_lengths(want):
        return f"{len(got)} datagrams of {lengths}, want {want['n']} of {want['len']}"
    for k, (g, w) in enumerate(zip(got, want.get("data") or [])):
        if g != bytes.fromhex(w):
            return f"datagram {k} differs"
    if hashlib.sha256(b"".join(got)).hexdigest() != want["sha256"]:
        return "same lengths, different bytes (SHA-256)"
    return None


@pytest.mark.parametrize("name", ["loop", "no_loop", "drop_0x808"])
def test_every_reply_matches_the_cpp_relay(corpus, board, name):
    config = next(c for c in corpus["configs"] if c["name"] == name)
    params = config["params"]
    src = corpus["source"]
    relay = asp.Relay(
        asp.ReplaySource.synthetic(src["synthetic_samples"], src["channels"]),
        loop={"true": True, "false": False}[params["loop"]],
        drop_mask=int(params["drop_mask"]),
    )
    requests = [bytes.fromhex(r) for r in corpus["requests"]]
    assert len(config["replies"]) == len(requests)
    diffs = []
    with relay:
        # The relay answers in arrival order and the recorder waited for
        # quiet after each request, so receiving exactly the recorded number
        # of datagrams per request keeps the replies aligned: one too many
        # shows up at the next request, or in the wait at the end.
        for i, (request, want) in enumerate(zip(requests, config["replies"])):
            board.sendto(request, ("127.0.0.1", relay.port))
            diff = difference(receive(board, want["n"]), want)
            if diff:
                diffs.append(f"{name}: request {i} ({describe(request)}): {diff}")
        if not quiet(board, corpus["quiet_ms"] / 1000):
            diffs.append(f"{name}: a datagram after the last reply")
    assert diffs == [], "\n".join(diffs[:10])


def test_the_corpus_runs_the_relay_through_every_config(corpus):
    names = [c["name"] for c in corpus["configs"]]
    assert names == ["loop", "no_loop", "drop_0x808"]
    drop = next(c for c in corpus["configs"] if c["name"] == "drop_0x808")
    assert int(drop["params"]["drop_mask"]) == 0x808


# --- serving ---------------------------------------------------------------


def test_a_half_buffer_arrives_in_order_and_is_counted(source, board):
    with asp.Relay(source) as relay:
        assert relay.stats()["last_peer"] is None
        chunks = fetch(board, relay, 5, 1000, 147)
        stats = wait_for(relay, lambda s: s["requests_served"] == 1)
    assert [c.chunk_index for c in chunks] == list(range(21))
    assert {c.chunk_total for c in chunks} == {21}
    assert {c.seq for c in chunks} == {5}
    assert [c.sample_offset for c in chunks] == [1000 + 7 * i for i in range(21)]
    assert b"".join(c.payload for c in chunks) == source.read(1000, 147)
    assert stats == {
        "requests_served": 1,
        "retransmits_served": 0,
        "chunks_sent": 21,
        "chunks_dropped": 0,
        "bytes_sent": 21 * (28 + 7 * 96 * 2),
        "bad_packets": 0,
        "wraps": 0,
        "last_offset": 1000,
        "last_peer": "127.0.0.1",
    }


def test_a_request_across_the_end_wraps(source, board):
    with asp.Relay(source) as relay:
        chunks = fetch(board, relay, 1, 30000 - 73, 147)
        stats = wait_for(relay, lambda s: s["requests_served"] == 1)
    assert b"".join(c.payload for c in chunks) == source.read(30000 - 73, 147)
    assert chunks[10].sample_offset == 30000 - 3
    assert chunks[11].sample_offset == 4
    assert stats["wraps"] == 1


def test_without_loop_a_request_stops_at_the_end(source, board):
    with asp.Relay(source, loop=False) as relay:
        chunks = fetch(board, relay, 1, 30000 - 10, 147)
        # Past the end, with no F_LOOP: no reply at all.
        assert fetch(board, relay, 2, 30000, 147) == []
        assert len(fetch(board, relay, 3, 30000, 147, asp.F_LOOP)) == 21
        stats = wait_for(relay, lambda s: s["requests_served"] == 2)
    assert sum(c.sample_count for c in chunks) == 10
    assert stats["requests_served"] == 2


def test_a_malformed_request_is_counted_and_not_answered(source, board):
    with asp.Relay(source) as relay:
        request = bytearray(asp.encode_request(1, 0, 147))
        request[-1] ^= 1
        board.sendto(bytes(request), ("127.0.0.1", relay.port))
        assert quiet(board, 0.1)
        stats = wait_for(relay, lambda s: s["bad_packets"] == 1)
    assert stats["bad_packets"] == 1
    assert stats["requests_served"] == 0


def test_two_relays_on_port_0_get_different_ports(source):
    with asp.Relay(source) as one, asp.Relay(source) as two:
        assert one.port != two.port
        assert settled_server_threads(2) == 2


def test_samples_per_chunk_is_what_fits_a_payload():
    assert asp.Relay(asp.ReplaySource.synthetic(10, 96)).samples_per_chunk == 7
    assert asp.Relay(asp.ReplaySource.synthetic(10, 1)).samples_per_chunk == 722
    assert asp.Relay(asp.ReplaySource.synthetic(10, 722)).samples_per_chunk == 1


def test_a_given_port_is_the_one_bound(source, board):
    port = board.getsockname()[1]
    board.close()  # free it for the relay
    with asp.Relay(source, port=port) as relay:
        assert relay.port == port


# --- drop_mask -------------------------------------------------------------


def test_drop_mask_withholds_first_attempts_only(source, board):
    with asp.Relay(source, drop_mask=0x808) as relay:
        first = fetch(board, relay, 1, 0, 147)
        again = fetch(board, relay, 1, 0, 147, asp.F_RETRANSMIT)
        stats = wait_for(relay, lambda s: s["requests_served"] == 2)
    # Bits 3 and 11: chunks 3 and 11 are withheld, the rest arrive in order.
    assert [c.chunk_index for c in first] == [i for i in range(21) if i not in (3, 11)]
    assert [c.chunk_index for c in again] == list(range(21))
    assert stats["chunks_dropped"] == 2
    assert stats["chunks_sent"] == 19 + 21
    assert stats["retransmits_served"] == 1


def test_a_full_drop_mask_withholds_every_first_attempt(source, board):
    with asp.Relay(source, drop_mask=0xFFFF_FFFF) as relay:
        assert fetch(board, relay, 1, 0, 147) == []
        assert fetch(board, relay, 2, 0, 7) == []
        assert len(fetch(board, relay, 3, 0, 7, asp.F_RETRANSMIT)) == 1
        stats = wait_for(relay, lambda s: s["requests_served"] == 3)
    assert stats["chunks_dropped"] == 21 + 1


# --- starting and stopping -------------------------------------------------


def test_start_and_stop_twice(source, board):
    relay = asp.Relay(source)
    assert relay.port is None
    for round_ in range(2):
        relay.start()
        port = relay.port
        assert port is not None
        relay.start()  # a no-op while running: the same socket
        assert relay.port == port
        assert settled_server_threads(1) == 1
        assert len(fetch(board, relay, round_, 0, 147)) == 21
        relay.stop()
        assert relay.port is None
        relay.stop()  # a no-op while stopped
        assert settled_server_threads(0) == 0
        assert port_is_free(port)
    # The counters go on across a stop and a start.
    assert relay.stats()["requests_served"] == 2


def test_after_stop_no_thread_is_left_and_the_port_is_free(source, board):
    relay = asp.Relay(source)
    relay.start()
    port = relay.port
    try:
        assert settled_server_threads(1) == 1
        assert not port_is_free(port)
        assert len(fetch(board, relay, 1, 0, 7)) == 1
    finally:
        relay.stop()
    assert settled_server_threads(0) == 0
    assert port_is_free(port)
    # And nothing answers there any more.
    board.sendto(asp.encode_request(2, 0, 7), ("127.0.0.1", port))
    assert quiet(board, 0.1)


def test_the_context_manager_stops_on_an_exception(source):
    relay = asp.Relay(source)
    with pytest.raises(RuntimeError, match="inside"):
        with relay as entered:
            assert entered is relay
            assert relay.port is not None
            raise RuntimeError("inside")
    assert relay.port is None
    assert settled_server_threads(0) == 0


def test_dropping_a_running_relay_stops_it(source):
    relay = asp.Relay(source)
    relay.start()
    port = relay.port
    del relay
    assert settled_server_threads(0) == 0
    assert port_is_free(port)


def test_stop_releases_the_gil(source, board):
    ticks = 0
    done = threading.Event()

    def count():
        nonlocal ticks
        while not done.is_set():
            ticks += 1
            time.sleep(0.001)

    with asp.Relay(source) as relay:
        # Right after a reply the thread is back in a fresh 200 ms receive,
        # so stop() waits most of that for it.
        assert len(fetch(board, relay, 1, 0, 7)) == 1
        counter = threading.Thread(target=count)
        counter.start()
        time.sleep(0.01)
        before = ticks
        started = time.monotonic()
        relay.stop()
        waited = time.monotonic() - started
        during = ticks - before
        done.set()
        counter.join()
    # The 50 ms of the quiet wait in fetch() came off the 200 ms.
    assert waited > 0.05
    # Holding the GIL, stop() would let the counter run not once.
    assert during >= 10, (during, waited)


# --- errors ----------------------------------------------------------------


def test_a_bad_bind_address_is_value_error_with_the_c_text(source):
    relay = asp.Relay(source, bind_addr="1.2.3")
    with pytest.raises(ValueError, match=r"^bad bind address: 1\.2\.3$"):
        relay.start()
    assert relay.port is None
    assert settled_server_threads(0) == 0


def test_a_port_in_use_is_os_error_with_the_c_text(source, board):
    port = board.getsockname()[1]
    relay = asp.Relay(source, port=port)
    with pytest.raises(OSError) as raised:
        relay.start()
    assert raised.value.errno == errno.EADDRINUSE
    assert str(raised.value) == f"bind 127.0.0.1:{port}: Address already in use"
    assert relay.port is None
    assert settled_server_threads(0) == 0
    # Once the port is free the same relay starts.
    board.close()
    relay.start()
    assert relay.port == port
    relay.stop()


def test_an_address_not_on_this_host_is_os_error(source):
    # 192.0.2.0/24 is TEST-NET-1: never a local address.
    with pytest.raises(OSError) as raised:
        asp.Relay(source, bind_addr="192.0.2.1").start()
    assert raised.value.errno == errno.EADDRNOTAVAIL
    assert str(raised.value).startswith("bind 192.0.2.1:")


def test_channels_too_wide_for_a_chunk_is_value_error_with_the_c_text():
    wide = asp.ReplaySource.synthetic(1, 723)
    text = "723 channels do not fit one MTU; jumbo frames or channel splitting required"
    with pytest.raises(ValueError) as raised:
        asp.Relay(wide)
    assert str(raised.value) == text


@pytest.mark.parametrize(
    "kwargs",
    [{"port": -1}, {"port": 65536}, {"drop_mask": -1}, {"drop_mask": 1 << 32}],
)
def test_arguments_out_of_range_are_value_error(source, kwargs):
    with pytest.raises(ValueError):
        asp.Relay(source, **kwargs)


def test_arguments_of_the_wrong_type_are_type_error(source):
    with pytest.raises(TypeError):
        asp.Relay(b"not a source")
    with pytest.raises(TypeError):
        asp.Relay(source, port="5010")
    with pytest.raises(TypeError):
        asp.Relay(source, loop="yes")

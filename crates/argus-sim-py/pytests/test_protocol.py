"""The protocol functions: the CRC, requests, chunks and their errors.

The references here are plain Python, independent of the bindings: a bitwise
CRC and struct layouts written out from argus_wire.h. The corpus requests and
replies were recorded from the C++ relay (tools/record_golden.py), so the
bytes compared against are the C++ relay's own.
"""

import json
import pathlib
import random
import struct

import pytest

import argus_sim_py as asp

REPO = pathlib.Path(__file__).resolve().parents[3]
CORPUS = REPO / "testdata" / "relay_corpus.json"

# argus_replay_request_t: magic, version, type, seq, sample_offset,
# sample_count, flags, crc. Packed, little-endian.
REQUEST = struct.Struct("<IHHIIHHH")
# argus_replay_chunk_hdr_t: magic, version, type, seq, sample_offset,
# sample_count, chunk_index, chunk_total, channel_count, payload_len, crc.
CHUNK = struct.Struct("<IHHIIHHHHHH")

MAGIC = 0x41524753
REQUEST_KINDS = {"size", "magic", "version", "type", "crc", "no_samples"}
CHUNK_KINDS = {"size", "magic", "version", "type", "payload", "crc"}


def crc_bitwise(data, crc=0xFFFF):
    """CRC-16/CCITT-FALSE a bit at a time: the definition, with no table."""
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else (crc << 1)
            crc &= 0xFFFF
    return crc


def request_bytes(seq, offset, count, flags=0, magic=MAGIC, version=1, kind=0x0010, crc=None):
    """A request packed by hand, its CRC computed bitwise unless given."""
    head = REQUEST.pack(magic, version, kind, seq, offset, count, flags, 0)[:20]
    if crc is None:
        crc = crc_bitwise(head)
    return head + struct.pack("<H", crc)


def chunk_bytes(seq, offset, count, index, total, channels, payload, magic=MAGIC, version=1,
                kind=0x0011, payload_len=None):
    """A chunk packed by hand, its CRC computed bitwise over the header up to
    the CRC field and then the payload."""
    if payload_len is None:
        payload_len = len(payload)
    fields = (magic, version, kind, seq, offset, count, index, total, channels, payload_len)
    head = CHUNK.pack(*fields, 0)[:26]
    crc = crc_bitwise(payload, crc_bitwise(head))
    return head + struct.pack("<H", crc) + payload


def rows(count, channels, seed=0):
    """`count` rows of `channels` u16 samples, deterministic noise."""
    rng = random.Random(seed)
    return bytes(rng.randrange(256) for _ in range(count * channels * 2))


@pytest.fixture(scope="module")
def corpus():
    return json.loads(CORPUS.read_text(encoding="utf-8"))


def expected_request_reject(data):
    """Why the relay refuses `data`, in its order, or None: the C++ checks
    written out again in Python."""
    if len(data) != 22:
        return "size"
    magic, version, kind, _, _, count, _, crc = REQUEST.unpack(data)
    if magic != MAGIC:
        return "magic"
    if version != 1:
        return "version"
    if kind != 0x0010:
        return "type"
    if crc != crc_bitwise(data[:20]):
        return "crc"
    if count == 0:
        return "no_samples"
    return None


# --- the CRC ---------------------------------------------------------------


def test_crc16_check_value():
    assert asp.crc16(b"123456789") == 0x29B1
    assert asp.crc16(bytearray(b"123456789")) == 0x29B1
    assert asp.crc16(b"") == 0xFFFF


def test_crc16_matches_bitwise():
    rng = random.Random(0x41524753)
    data = bytes(rng.randrange(256) for _ in range(1600))
    for length in range(0, len(data) + 1, 37):
        assert asp.crc16(data[:length]) == crc_bitwise(data[:length]), length


def test_crc16_resumes_from_any_register():
    rng = random.Random(7)
    data = bytes(rng.randrange(256) for _ in range(200))
    for split in range(0, len(data) + 1, 13):
        head, tail = data[:split], data[split:]
        assert asp.crc16(tail, asp.crc16(head)) == asp.crc16(data), split
    for _ in range(50):
        register = rng.randrange(0x10000)
        piece = data[: rng.randrange(len(data))]
        assert asp.crc16(piece, crc=register) == crc_bitwise(piece, register)
    assert asp.crc16(b"", 0) == 0


# --- requests --------------------------------------------------------------


def test_requests_encode_as_the_corpus_does(corpus):
    encoded = 0
    for text in corpus["requests"]:
        data = bytes.fromhex(text)
        if expected_request_reject(data) not in (None, "no_samples"):
            continue
        _, _, _, seq, offset, count, flags, _ = REQUEST.unpack(data)
        assert asp.encode_request(seq, offset, count, flags) == data, text
        encoded += 1
    # The three test fetches, the edge offsets and counts, the flags and 300
    # random requests: most of the corpus is valid.
    assert encoded > 300


def test_requests_decode_as_the_relay_judges_them(corpus):
    kinds = set()
    for text in corpus["requests"]:
        data = bytes.fromhex(text)
        want = expected_request_reject(data)
        if want is None:
            req = asp.decode_request(data)
            magic, version, kind, seq, offset, count, flags, crc = REQUEST.unpack(data)
            got = (req.seq, req.sample_offset, req.sample_count, req.flags, req.crc)
            assert got == (seq, offset, count, flags, crc), text
        else:
            with pytest.raises(asp.WireError) as err:
                asp.decode_request(data)
            assert err.value.kind == want, text
            kinds.add(want)
    # The corpus has a bad length, magic, version, type and CRC, and count 0.
    assert kinds == REQUEST_KINDS


def test_request_round_trips_at_the_field_limits():
    for seq, offset, count, flags in [
        (0, 0, 1, 0),
        (0xFFFFFFFF, 0xFFFFFFFF, 0xFFFF, 0xFFFF),
        (7, 1000, 147, asp.F_LOOP | asp.F_RETRANSMIT),
    ]:
        data = asp.encode_request(seq, offset, count, flags)
        assert data == request_bytes(seq, offset, count, flags)
        req = asp.decode_request(data)
        assert (req.seq, req.sample_offset, req.sample_count, req.flags) == (
            seq,
            offset,
            count,
            flags,
        )
        assert req.crc == crc_bitwise(data[:20])
    assert asp.encode_request(1, 2, 3) == asp.encode_request(1, 2, 3, 0)


def test_every_request_error_kind():
    good = request_bytes(1, 0, 147)
    cases = {
        "size": [b"", good[:21], good + b"\0", bytes(512)],
        "magic": [request_bytes(1, 0, 147, magic=MAGIC ^ 1)],
        "version": [request_bytes(1, 0, 147, version=2)],
        "type": [request_bytes(1, 0, 147, kind=0x0011)],
        "crc": [request_bytes(1, 0, 147, crc=crc_bitwise(good[:20]) ^ 1)],
        "no_samples": [request_bytes(1, 0, 0)],
    }
    assert set(cases) == REQUEST_KINDS
    for kind, datagrams in cases.items():
        for data in datagrams:
            with pytest.raises(asp.WireError) as err:
                asp.decode_request(data)
            assert err.value.kind == kind
            assert isinstance(err.value, ValueError)
            assert str(err.value).startswith("replay request: ")


# --- chunks ----------------------------------------------------------------


def test_corpus_chunks_decode_to_their_fields_and_encode_back(corpus):
    channels = corpus["source"]["channels"]
    seen = set()
    for config in corpus["configs"]:
        for reply in config["replies"]:
            for text in reply.get("data", []):
                data = bytes.fromhex(text)
                fields = CHUNK.unpack(data[:28])
                _, _, _, seq, offset, count, index, total, chans, length, crc = fields
                chunk = asp.decode_chunk(data)
                assert (
                    chunk.seq,
                    chunk.sample_offset,
                    chunk.sample_count,
                    chunk.chunk_index,
                    chunk.chunk_total,
                    chunk.channel_count,
                    chunk.payload_len,
                    chunk.crc,
                ) == (seq, offset, count, index, total, chans, length, crc)
                assert chans == channels
                assert chunk.payload == data[28:]
                assert crc == crc_bitwise(data[28:], crc_bitwise(data[:26]))
                again = asp.encode_chunk(
                    chunk.seq,
                    chunk.sample_offset,
                    chunk.chunk_index,
                    chunk.chunk_total,
                    chunk.channel_count,
                    chunk.payload,
                )
                assert again == data, f"{config['name']}: {text[:56]}"
                seen.add((config["name"], seq, index, total))
    # Whole replies are kept for the first 20 requests of each config: the 8
    # rejected ones and one-chunk requests (1, 2 and 7 rows) around the end of
    # the dataset. Every config has some.
    assert {name for name, _, _, _ in seen} == {c["name"] for c in corpus["configs"]}


def test_encode_chunk_matches_a_hand_packed_one():
    for count, channels in [(7, 96), (1, 96), (0, 96), (3, 1), (1, 722)]:
        payload = rows(count, channels, seed=count)
        want = chunk_bytes(0xDEADBEEF, 29_999, count, 4, 21, channels, payload)
        assert asp.encode_chunk(0xDEADBEEF, 29_999, 4, 21, channels, payload) == want
        assert asp.encode_chunk(0xDEADBEEF, 29_999, 4, 21, channels, bytearray(payload)) == want
        chunk = asp.decode_chunk(bytearray(want))
        assert chunk.sample_count == count
        assert chunk.payload_len == len(payload)


def test_every_chunk_error_kind():
    payload = rows(3, 96)
    good = chunk_bytes(1, 0, 3, 0, 1, 96, payload)
    flipped = bytearray(good)
    flipped[-1] ^= 0x80
    over = rows(8, 96)  # one row more than fits in MAX_PAYLOAD
    cases = {
        "size": [b"", good[:27]],
        "magic": [chunk_bytes(1, 0, 3, 0, 1, 96, payload, magic=MAGIC ^ 1)],
        "version": [chunk_bytes(1, 0, 3, 0, 1, 96, payload, version=2)],
        "type": [chunk_bytes(1, 0, 3, 0, 1, 96, payload, kind=0x0010)],
        "payload": [
            good[:-1],
            good + b"\0",
            chunk_bytes(1, 0, 4, 0, 1, 96, payload),
            chunk_bytes(1, 0, 3, 0, 1, 95, payload),
            chunk_bytes(1, 0, 8, 0, 1, 96, over),
            chunk_bytes(1, 0, 3, 0, 1, 96, payload, payload_len=len(payload) - 2),
        ],
        "crc": [bytes(flipped)],
    }
    assert set(cases) == CHUNK_KINDS
    for kind, datagrams in cases.items():
        for data in datagrams:
            with pytest.raises(asp.WireError) as err:
                asp.decode_chunk(data)
            assert err.value.kind == kind, data[:28].hex()
            assert str(err.value).startswith("replay chunk: ")


def test_encode_chunk_refuses_a_payload_it_cannot_carry():
    for payload, channels in [
        (b"\0", 96),  # half a sample
        (bytes(2 * 96 + 2), 96),  # a row and a sample
        (rows(8, 96), 96),  # over MAX_PAYLOAD
        (bytes(asp.MAX_PAYLOAD + 1), 1),  # over MAX_PAYLOAD, odd too
    ]:
        with pytest.raises(asp.WireError) as err:
            asp.encode_chunk(1, 0, 0, 1, channels, payload)
        assert err.value.kind == "payload"
    # Exactly MAX_PAYLOAD is allowed.
    asp.encode_chunk(1, 0, 0, 1, 2, bytes(asp.MAX_PAYLOAD))
    with pytest.raises(ValueError) as err:
        asp.encode_chunk(1, 0, 0, 1, 0, b"")
    assert type(err.value) is ValueError
    assert "channel_count must be positive" in str(err.value)


# --- bad arguments ---------------------------------------------------------


@pytest.mark.parametrize(
    "call",
    [
        lambda: asp.encode_request(-1, 0, 1),
        lambda: asp.encode_request(2**32, 0, 1),
        lambda: asp.encode_request(0, -1, 1),
        lambda: asp.encode_request(0, 2**32, 1),
        lambda: asp.encode_request(0, 0, 2**16),
        lambda: asp.encode_request(0, 0, 1, -1),
        lambda: asp.encode_request(0, 0, 1, 2**16),
        lambda: asp.encode_request(2**100, 0, 1),
        lambda: asp.crc16(b"", -1),
        lambda: asp.crc16(b"", 0x10000),
        lambda: asp.encode_chunk(2**32, 0, 0, 1, 96, b""),
        lambda: asp.encode_chunk(0, 0, 2**16, 1, 96, b""),
        lambda: asp.encode_chunk(0, 0, 0, -1, 96, b""),
        lambda: asp.encode_chunk(0, 0, 0, 1, 2**16, b""),
    ],
)
def test_a_field_out_of_range_is_a_value_error(call):
    with pytest.raises(ValueError) as err:
        call()
    assert not isinstance(err.value, OverflowError)
    assert not isinstance(err.value, asp.WireError)


@pytest.mark.parametrize(
    "call",
    [
        lambda: asp.encode_request("1", 0, 1),
        lambda: asp.encode_request(1.5, 0, 1),
        lambda: asp.encode_request(None, 0, 1),
        lambda: asp.encode_request(1, 0),
        lambda: asp.crc16("123456789"),
        lambda: asp.crc16(None),
        lambda: asp.crc16([1, 2, 3]),
        lambda: asp.crc16(b"", "0xFFFF"),
        lambda: asp.decode_request("x" * 22),
        lambda: asp.decode_request(22),
        lambda: asp.decode_chunk(None),
        lambda: asp.encode_chunk(1, 0, 0, 1, 96, [0] * 192),
        lambda: asp.encode_chunk(1, 0, 0, 1, 96, "payload"),
    ],
)
def test_a_wrong_type_is_a_type_error(call):
    with pytest.raises(TypeError):
        call()


def test_garbage_raises_wire_errors_and_nothing_else():
    """Random and mutated datagrams: every one decodes or raises WireError.
    A panic would surface as pyo3's PanicException, which is not an Exception
    and would fail this test."""
    rng = random.Random(2016)
    good_request = request_bytes(5, 100, 147)
    good_chunk = chunk_bytes(5, 100, 7, 0, 21, 96, rows(7, 96))
    samples = [bytes(rng.randrange(256) for _ in range(rng.randrange(1600))) for _ in range(300)]
    for good in (good_request, good_chunk):
        for _ in range(300):
            mutated = bytearray(good)
            for _ in range(rng.randrange(1, 4)):
                mutated[rng.randrange(len(mutated))] ^= 1 << rng.randrange(8)
            samples.append(bytes(mutated[: rng.randrange(len(mutated) + 2)]))
    for data in samples:
        for decode, kinds in ((asp.decode_request, REQUEST_KINDS), (asp.decode_chunk, CHUNK_KINDS)):
            try:
                decode(data)
            except asp.WireError as err:
                assert err.kind in kinds


# --- the classes -----------------------------------------------------------


def test_request_and_chunk_are_immutable_values():
    req = asp.decode_request(asp.encode_request(7, 1000, 147, asp.F_LOOP))
    assert req == asp.decode_request(asp.encode_request(7, 1000, 147, asp.F_LOOP))
    assert req != asp.decode_request(asp.encode_request(8, 1000, 147, asp.F_LOOP))
    assert hash(req) == hash(asp.decode_request(asp.encode_request(7, 1000, 147, asp.F_LOOP)))
    assert repr(req).startswith("Request(seq=7, sample_offset=1000, sample_count=147, flags=0x0002")
    with pytest.raises(AttributeError):
        req.seq = 8

    data = asp.encode_chunk(7, 1000, 0, 21, 96, rows(7, 96))
    chunk = asp.decode_chunk(data)
    assert chunk == asp.decode_chunk(data)
    assert hash(chunk) == hash(asp.decode_chunk(data))
    assert repr(chunk).startswith("Chunk(seq=7, sample_offset=1000, sample_count=7, chunk_index=0")
    with pytest.raises(AttributeError):
        chunk.payload = b""

    # Made by decoding only: there is no constructor to bypass the checks.
    for cls in (asp.Request, asp.Chunk):
        with pytest.raises(TypeError):
            cls()


def test_the_public_items_have_docstrings():
    for name in (
        "crc16",
        "encode_request",
        "decode_request",
        "encode_chunk",
        "decode_chunk",
        "Request",
        "Chunk",
        "WireError",
    ):
        doc = getattr(asp, name).__doc__
        assert doc and doc.strip(), f"{name} has no docstring"
    assert "data, crc=0xFFFF" in asp.crc16.__text_signature__
    assert "flags=0" in asp.encode_request.__text_signature__

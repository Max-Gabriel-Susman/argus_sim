"""The module itself: its version and the wire constants.

The expected values are written out from argus_wire.h by hand rather than
taken from any Rust, so a constant that drifted in the bindings shows here.
"""

import pathlib
import re

import argus_sim_py

# The repository root, from crates/argus-sim-py/pytests/.
REPO = pathlib.Path(__file__).resolve().parents[3]


def workspace_version():
    """The version in the workspace's Cargo.toml ([workspace.package])."""
    text = (REPO / "Cargo.toml").read_text(encoding="utf-8")
    section = text.split("[workspace.package]", 1)[1]
    match = re.search(r'^version = "([^"]+)"', section, re.MULTILINE)
    assert match, "no version in [workspace.package]"
    return match.group(1)


def test_version_is_the_crates():
    assert argus_sim_py.__version__ == workspace_version()


def test_version_is_package_xmls():
    text = (REPO / "package.xml").read_text(encoding="utf-8")
    assert f"<version>{argus_sim_py.__version__}</version>" in text


def test_protocol_constants_are_argus_wire_hs():
    assert argus_sim_py.MAGIC == 0x41524753
    assert argus_sim_py.MAGIC.to_bytes(4, "little") == b"SGRA"
    assert argus_sim_py.VERSION == 1
    assert argus_sim_py.MSG_REQUEST == 0x0010
    assert argus_sim_py.MSG_CHUNK == 0x0011
    assert argus_sim_py.F_RETRANSMIT == 0x0001
    assert argus_sim_py.F_LOOP == 0x0002
    assert argus_sim_py.REPLAY_PORT == 5010
    assert argus_sim_py.MAX_CHANNELS == 96
    assert argus_sim_py.MAX_PAYLOAD == 1444
    # ARGUS_REPLAY_SAMPLES_PER_CHUNK x ARGUS_REPLAY_CHUNKS_PER_HALF, at 96
    # channels: 7 x 21.
    assert argus_sim_py.SAMPLES_PER_HALF == 147


def test_lengths_are_the_packed_structs():
    # argus_replay_request_t: magic u32, version u16, type u16, seq u32,
    # sample_offset u32, sample_count u16, flags u16, crc u16.
    assert argus_sim_py.REQUEST_LEN == 4 + 2 + 2 + 4 + 4 + 2 + 2 + 2 == 22
    # argus_replay_chunk_hdr_t: magic u32, version u16, type u16, seq u32,
    # sample_offset u32, then sample_count, chunk_index, chunk_total,
    # channel_count, payload_len and crc, each u16.
    assert argus_sim_py.CHUNK_HEADER_LEN == 4 + 2 + 2 + 4 + 4 + 6 * 2 == 28
    assert argus_sim_py.MAX_CHUNK_LEN == 28 + 1444
    # A full chunk, with the IPv4 (20) and UDP (8) headers, fits a 1500-byte
    # MTU: the reason MAX_PAYLOAD is what it is.
    assert argus_sim_py.MAX_CHUNK_LEN + 20 + 8 <= 1500


def test_the_stub_lists_every_public_name():
    stub = (pathlib.Path(__file__).resolve().parents[1] / "argus_sim_py.pyi").read_text(
        encoding="utf-8"
    )
    # maturin wraps the extension in a package whose __init__ re-exports the
    # extension's __all__, which PyO3 fills with every exported item. dir()
    # would also list the extension submodule itself.
    public = argus_sim_py.__all__
    assert "__version__" in public
    assert "MAGIC" in public
    for name in public:
        assert re.search(rf"^{name}\b", stub, re.MULTILINE), f"{name} is not in the stub"

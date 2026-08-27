# Argus Sim

Simulation components for the Argus Cybernetics stack — the parts of the
pipeline that stand in for hardware that isn't present yet.

The system's acquisition path runs from an RHD2132 analog front end through
FPGA gateware to a ROS 2 graph. Two ends of that path can be replaced with
software: the electrode array feeding samples in, and the task environment
consuming decoded intent. This package holds both.

## Dataset relay

`dataset_relay_node` serves recorded neural data to the Zynq PS over UDP,
which writes it into PL block RAM for the simulated Intan chips to return over
SPI. From the neural codec's point of view the samples are indistinguishable
from silicon.

The PS is the client: it requests a buffer half by absolute sample offset and
the relay answers with a burst of chunks. Because requests carry an absolute
offset rather than "next", the relay holds no per-client play position — it
survives a PS reset with no resync handshake and replays identically across
runs.

The wire format lives in `argus_core/argus_wire.h`, shared with the receiver
and the bare-metal firmware. Protocol logic is in
`argus_core/argus_replay_client.h`, written to bare-metal constraints so the
host test client and the PS firmware run the same implementation over
different transports.

Run it with no dataset to serve a synthetic identity pattern, where every word
encodes its own chip, channel, and sample index — the same pattern the VHDL
model generates, so a mismatch localises to the transport rather than the
data:

    ros2 run argus_sim dataset_relay_node

Then exercise the protocol end to end:

    ros2 run argus_sim replay_client_test

/*
 *  Host test client for the replay relay.
 *
 *  Wraps argus_replay_client.h in BSD sockets. The PS wraps the
 *  same core in lwIP; everything below is transport shell, and
 *  none of the protocol logic lives here.
 *
 *  Its job is to make ReplayServer a known-good peer before the
 *  PS client is written, so that when the two ends first talk
 *  to each other over real hardware, only one of them is unproven.
 *
 *  Verifies, beyond the checks inside the client core:
 *      - burst arrives as the expected number of chunks
 *      - every sample decodes to the channel and index the
 *        identity pattern claims, so a mis-ordered or
 *        mis-offset chunk is caught rather than averaging out
 *      - a wrap at the end of the dataset produces continuous
 *        samples
 *
 *  Not built by default in a way that runs in CI; it needs a live
 *      relay.
 *
 *      ros2 run argus_sim dataset_relay_node --ros-args -p
 *      synthetic_sample s:=30000
 *
 *      ros2 run argus_sim replay_client_test
 */
 #include <arpa/inet.h>
 #include <netinet/in.h>
 #include <sys/socket.h>
 #include <unistd.h>

 #include <chrono>
 #include <cstdint>
 #include <cstdio>
 #include <cstring>
 #include <string>
 #include <vector>

 #include "argus_core/argus_replay_client.h"
 #include "argus_core/argus_wire.h"

namespace
{
struct UdpTransport
{
  int sock {-1};
  sockaddr_in peer {};
};

int udp_send(void * ctx, const void * data, uint16_t len)
{
  auto * t = static_cast<UdpTransport *>(ctx);
  const ssize_t rc = ::sendto(
    t->sock, data, len, 0,
    reinterpret_cast<const sockaddr *>(&t->peer), sizeof(t->peer));
  return (rc == static_cast<ssize_t>(len)) ? 0 : -1;
}

uint32_t now_ms()
{
  using namespace std::chrono;
  return static_cast<uint32_t>(
    duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count());
}

/*  Mirrors the IDENT pattern in argus_rhd2132_model.vhd and in
     *  ReplaySource::synthetic: chip in 15:14, channel in 13:8,
     *  index in 7:0. */
uint16_t expected_word(uint16_t channel, uint32_t sample_index)
{
  const uint16_t chip = static_cast<uint16_t>((channel / 32) & 0x3);
  const uint16_t ch_in_chip = static_cast<uint16_t>(channel % 32);
  const uint16_t idx = static_cast<uint16_t>(sample_index & 0xFF);
  return static_cast<uint16_t>((chip << 14) | (ch_in_chip << 8) | idx);
}

int failures = 0;

void check(bool ok, const std::string & what)
{
  if (!ok) {
    std::printf(" FAIL %s\n", what.c_str());
    failures++;
  }
}

/*  Runs one fetch to completion, pumping the socket and the
     *  client's timer. */
bool fetch_block(
  argus_replay_client_t & cl, UdpTransport & t,
  uint32_t offset, uint16_t count, std::vector<uint16_t> & dst)
{
  if (argus_replay_client_fetch(&cl, offset, count, dst.data(), now_ms()) != 0) {
    std::printf(" FAIL fetch() rejected offset=%u\n", offset);
    failures++;
    return false;
  }

  uint8_t rx[2048];
  while (!argus_replay_client_is_done(&cl)) {
    const ssize_t n = ::recv(t.sock, rx, sizeof(rx), 0);
    if (n > 0) {
      argus_replay_client_on_packet(&cl, rx, static_cast<uint16_t>(n));
    }
    argus_replay_client_poll(&cl, now_ms());
  }
  return cl.state == ARGUS_REPLAY_COMPLETE;
}
}  /* namespace */

int main(int argc, char ** argv)
{
  const std::string host = (argc > 1) ? argv[1] : "127.0.0.1";
  const uint16_t port = (argc > 2) ? static_cast<uint16_t>(std::stoi(argv[2])) :
    static_cast<uint16_t>(ARGUS_REPLAY_PORT);
  const uint16_t channels = ARGUS_MAX_CHANNELS;

  UdpTransport t;
  t.sock = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (t.sock < 0) {
    std::perror("socket");
    return 1;
  }

  /*  Bounded recv so a dead relay surfaces as a client timeout
      and retry rather than a hang. */
  timeval tv {};
  tv.tv_sec = 0;
  tv.tv_usec = 50000;
  ::setsockopt(t.sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  t.peer.sin_family = AF_INET;
  t.peer.sin_port = htons(port);
  if (::inet_pton(AF_INET, host.c_str(), &t.peer.sin_addr) != 1) {
    std::printf("bad_host %s\n", host.c_str());
    return 1;
  }

  argus_replay_client_t cl;
  if (argus_replay_client_init(&cl, udp_send, &t, channels, 200, 5) != 0) {
    std::printf("%u channels do not fit one MTU\n", channels);
    return 1;
  }

  std::printf(
    "replay client -> %s:%u, %u channels, %u samples/chunk\n",
    host.c_str(), port, channels, cl.samples_per_chunk);

  const uint16_t half = ARGUS_REPLAY_SAMPLES_PER_HALF;
  std::vector<uint16_t> dst(static_cast<size_t>(half) * channels);

  /* 1. One half from the start of the dataset. */
  std::printf("test 1: fetch %u samples at offset 0\n", half);
  if (fetch_block(cl, t, 0, half, dst)) {
    const uint16_t want_chunks =
      static_cast<uint16_t>((half + cl.samples_per_chunk - 1) / cl.samples_per_chunk);
    check(cl.total_chunks == want_chunks, "chunk count");

    for (uint16_t s = 0; s < half; ++s) {
      for (uint16_t c = 0; c < channels; ++c) {
        const uint16_t got = dst[static_cast<size_t>(s) * channels + c];
        if (got != expected_word(c, s)) {
          check(
            false,
            "sample " + std::to_string(s) + " ch " + std::to_string(c) +
            ": got 0x" + std::to_string(got));
          s = half;           /* one report is enough */
          break;
        }
      }
    }
  } else {
    check(false, "fetch did not complete");
  }

  /* 2. A non-zero offset, to catch base_offset arithmetic. */
  std::printf("test 2: fetch %u samples at offset 1000\n", half);
  if (fetch_block(cl, t, 1000, half, dst)) {
    for (uint16_t s = 0; s < half; ++s) {
      const uint16_t got = dst[static_cast<size_t>(s) * channels];
      if (got != expected_word(0, 1000u + s)) {
        check(false, "offset sample " + std::to_string(s));
        break;
      }
    }
  } else {
    check(false, "fetch did not complete");
  }

  /*  3. A fetch that straddles the end of the dataset. The relay
      wraps per-sample; samples must stay continuous across the
      seam. */
  const uint32_t n_samples = 30000;   /* matches the node's default */
  const uint32_t straddle = n_samples - (half / 2);
  std::printf(
    "test 3: fetch %u samples at offset %u (wraps)\n",
    half, straddle);
  if (fetch_block(cl, t, straddle, half, dst)) {
    for (uint16_t s = 0; s < half; ++s) {
      const uint32_t src = (straddle + s) % n_samples;
      const uint16_t got = dst[static_cast<size_t>(s) * channels];
      if (got != expected_word(0, src)) {
        check(false, "wrap sample " + std::to_string(s));
        break;
      }
    }
  } else {
    check(false, "fetch did not complete");
  }

  std::printf(
    "\nrequests=%u retransmits=%u accepted=%u rejected=%u timeouts=%u\n",
    cl.requests_sent, cl.retransmits_sent, cl.chunks_accepted,
    cl.chunks_rejected, cl.timeouts);

  ::close(t.sock);

  if (failures == 0) {
    std::printf("PASS\n");
    return 0;
  }
  std::printf("FAIL: %d check(s)\n", failures);
  return 1;
}

/**
 * @file replay_server.hpp
 * @brief The host end of the replay protocol: a UDP server that hands the
 *        board its next samples on request.
 *
 * The firmware's replay client asks for one half-buffer of samples at a
 * time and plays them into the fabric as if the simulated chips were
 * sampling electrodes. This server answers each request with those samples
 * as a burst of CRC'd chunks, each sized to fit one Ethernet frame, and
 * answers a retransmit request for any chunk the client missed. The request
 * and chunk layouts are argus_core's (argus_wire.h). dataset_relay_node is
 * the ROS wrapper around this class; replay_client_test.cpp drives it from
 * the host side with the same client core the firmware uses.
 *
 * Samples come from a ReplaySource: either the headerless little-endian
 * uint16 file that nwb_to_replay.py writes, memory-mapped rather than read,
 * or a synthetic identity pattern that the fabric's self-test checks
 * against.
 */
#ifndef REPLAY_SERVER_HPP
    #define REPLAY_SERVER_HPP

    #include <atomic>
    #include <cstdint>
    #include <mutex>
    #include <string>
    #include <thread>
    #include <vector>

namespace argus_sim
{
/**
 * @brief The samples a ReplayServer serves: a memory-mapped file or a
 *        synthetic pattern, read-only, sample-major.
 *
 * Rows are samples and columns are channels, in the order nwb_to_replay.py
 * writes them, so sample(i) points at channel_count() contiguous codes. A
 * file source maps the file and never copies it; a synthetic source owns
 * its vector. Move-only, because it owns a mapping.
 */
class ReplaySource
{
public:
  /**
   * @brief Map a replay file.
   *
   * @param path           A file of little-endian uint16 codes, sample-major,
   *                       with no header; its size must be a whole number of
   *                       rows. Throws std::runtime_error if it cannot be
   *                       opened, sized or mapped.
   * @param channel_count  Codes per row. Throws std::invalid_argument when 0.
   * @return               A source over the mapping; the file stays mapped
   *                       until the source is destroyed.
   */
  static ReplaySource from_file(const std::string & path, uint16_t channel_count);

  /**
   * @brief Build the identity pattern the fabric's self-test expects.
   *
   * Each code packs its own position: bits 15..14 the chip (channel / 32),
   * bits 13..8 the channel within the chip, bits 7..0 the sample index
   * modulo 256. A chunk delivered out of order or at the wrong offset is
   * then visible in the data itself.
   *
   * @param n_samples      Rows to generate. Throws std::invalid_argument when 0.
   * @param channel_count  Codes per row. Throws std::invalid_argument when 0.
   * @return               A source that owns its samples.
   */
  static ReplaySource synthetic(size_t n_samples, uint16_t channel_count);

  ReplaySource() = default;
  ~ReplaySource();

  ReplaySource(const ReplaySource &) = delete;
  ReplaySource & operator=(const ReplaySource &) = delete;
  ReplaySource(ReplaySource && other) noexcept;
  ReplaySource & operator=(ReplaySource && other) noexcept;

  /** @brief The first code of the first sample. @return nullptr for an empty source. */
  const uint16_t * data() const {return data_;}
  /** @brief Rows in the source. @return 0 for an empty source. */
  size_t sample_count() const {return n_samples_;}
  /** @brief Codes per row. @return 0 for an empty source. */
  uint16_t channel_count() const {return n_channels_;}
  /**
   * @brief Whether there is anything to serve.
   * @return true when mapped or generated and non-empty.
   */
  bool valid() const {return data_ != nullptr && n_samples_ > 0;}
  /**
   * @brief One row.
   * @param index  Row number; not range-checked.
   * @return       A pointer to channel_count() contiguous codes.
   */
  const uint16_t * sample(size_t index) const
  {
    return data_ + index * static_cast<size_t>(n_channels_);
  }

private:
  void release();
  std::vector<uint16_t> owned_;
  void * map_ {nullptr};
  size_t map_bytes_ {0};
  const uint16_t * data_ {nullptr};
  size_t n_samples_ {0};
  uint16_t n_channels_ {0};
};

/**
 * @brief Counters a ReplayServer keeps across requests. stats() returns a
 *        snapshot; dataset_relay_node prints them as its status line.
 */
struct ReplayStats
{
  /** Requests rejected: wrong size, magic, version, type or CRC, or a sample_count of 0. */
  uint64_t bad_packets {0};
  /** Bytes handed to sendto(), chunk headers included. */
  uint64_t bytes_sent {0};
  /** Chunks withheld by Config::drop_mask. Zero outside fault-injection tests. */
  uint64_t chunks_dropped {0};
  /** Chunks handed to sendto(). */
  uint64_t chunks_sent {0};
  /** sample_offset of the last request served, after any wrap. */
  uint64_t last_offset {0};
  /** Requests that passed validation, retransmits included. */
  uint64_t requests_served {0};
  /** Of those, the ones flagged ARGUS_REPLAY_F_RETRANSMIT. */
  uint64_t retransmits_served {0};
  /** Requests that ran past the end of the source and continued from its start. */
  uint64_t wraps {0};
  /** Dotted address of the last requester, normally the board. */
  std::string last_peer;
};

/**
 * @brief Serves a ReplaySource over UDP, one request at a time, on its own
 *        thread.
 *
 * start() binds the socket and spawns the thread; the loop blocks in
 * recvfrom() with a 200 ms timeout, so stop() is noticed without a wake-up
 * packet. Every request is answered from its own fields alone (offset,
 * count, flags), so the server holds no per-client state and a board reset
 * mid-run needs nothing from it. The loop never runs on a ROS executor;
 * dataset_relay_node.cpp says why.
 */
class ReplayServer
{
public:
  /** @brief Everything the server takes besides its source. */
  struct Config
  {
    /** Address to bind; the default is every interface. */
    std::string bind_addr {"0.0.0.0"};
    /**
     * Test only: bit N withholds chunk N of a first-attempt request, so the
     * client's retransmit path gets exercised. Retransmits always land.
     */
    uint32_t drop_mask {0};
    /** UDP port; 0 means ARGUS_REPLAY_PORT. */
    uint16_t port {0};
    /**
     * Continue from the start when a request runs past the end of the
     * source. A request can also ask for this itself with ARGUS_REPLAY_F_LOOP.
     */
    bool loop {true};
    /** Cap on a request's sample_count; 0 means ARGUS_REPLAY_SAMPLES_PER_HALF. */
    uint16_t max_samples_per_request {0};
  };

  /**
   * @brief Size the chunks for this source; nothing is bound until start().
   *
   * @param source  The samples to serve. Must outlive the server and be
   *                valid(); throws std::invalid_argument otherwise, or when
   *                one row does not fit a chunk payload.
   * @param config  See Config; a zero port or request cap takes the wire
   *                header's default.
   */
  ReplayServer(const ReplaySource & source, Config config);
  ~ReplayServer();

  ReplayServer(const ReplayServer &) = delete;
  ReplayServer & operator=(const ReplayServer &) = delete;

  /**
   * @brief Bind the socket and start the receive thread.
   *
   * A no-op when already running. Throws std::runtime_error if the socket
   * cannot be created or bound.
   */
  void start();

  /**
   * @brief Stop the thread and close the socket.
   *
   * A no-op when not running; the destructor calls it.
   */
  void stop();

  /**
   * @brief A snapshot of the counters, taken under the stats lock.
   * @return The counters so far.
   */
  ReplayStats stats() const;

  /**
   * @brief Samples per chunk: ARGUS_REPLAY_MAX_PAYLOAD divided by the row size.
   * @return At least 1.
   */
  uint16_t samples_per_chunk() const {return samples_per_chunk_;}

private:
  /** @brief The receive loop: recvfrom() with a timeout, then handle(), until running_ clears. */
  void run();

  /**
   * @brief Validate one request and answer it with a burst of chunks.
   *
   * @param data      The datagram.
   * @param len       Its length; anything but sizeof(argus_replay_request_t)
   *                  is rejected.
   * @param peer      The sender's sockaddr_in, where the chunks go back to.
   * @param peer_len  Its length.
   */
  void handle(const uint8_t * data, size_t len, const void * peer, size_t peer_len);

  const ReplaySource & source_;
  Config config_;
  uint16_t samples_per_chunk_ {0};
  size_t row_bytes_ {0};

  int sock_ {-1};
  std::thread thread_;
  std::atomic<bool> running_ {false};

  mutable std::mutex stats_mutex_;
  ReplayStats stats_;

  std::vector<uint8_t> tx_buffer_;
};     /* class ReplayServer */
}       /* namespace argus_sim */
#endif /* REPLAY_SERVER_HPP */

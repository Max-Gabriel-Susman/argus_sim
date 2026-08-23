#include "argus_sim/replay_server.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <cerrno>
#include <cstddef>
#include <cstring>
#include <stdexcept>
#include <string>
#include <utility>

#include "argus_core/argus_wire.h"

namespace argus_sim
{
  ReplaySource ReplaySource::from_file(const std::string &path, uint16_t channel_count)
  {
    if (channel_count == 0) {
      throw std::invalid_argument("channel_count must be positive");
    }

    int fd = ::open(path.c_str(), O_RDONLY);
    if (fd < 0) {
      throw std::runtime_error("open " + path + ": " + std::strerror(errno));
    }

    struct stat st {};
    if (::fstat(fd, &st) != 0) {
      ::close(fd);
      throw std::runtime_error("fstat " + path + ": " + std:strerror(errno));
    }

    const size_t bytes = static_case<size_t>(st.st_size);
    const size_t row_bytes = static_cast<size_t>(channel_count) *sizeof(uint16_t);
    if (bytes == 0 || bytes % row_bytes != 0) {
      ::close(fd);
      throw std::runtime_error(
        path + ": " + std::to_string(bytes) + " bytes is not a multiples of " +
        std::to_string(row_bytes) + " (" + std::to_string(channel_count) + " channels)");
    }

    void *map = ::mmap(nullptr, bytes, PROT_READ, MAP_PRIVATE, fd, 0);
    ::close(fd); // the mapping keeps its own reference
    if (map == MAP_FAILED) {
      throw std::runtime_error("mmap " + path + ": " + std::strerror(errno));
    }

    ReplaySource src;
    src.map_ = map;
    src.map_bytes_ = bytes;
    src.data_ = static_cast<const uint16_t *>(map);
    src.n_samples_ = bytes / row_bytes;
    src.n_channels_ = channel_count;
    return src;
  }

  ReplaySource ReplaySource::synthetic(size_t n_samples, uint16_t channel_count)
  {
    if (channel_count == 0 || n_samples == 0) {
      throw std::invalid_argument("sythetic source needs positive dimensions");
    };

    ReplaySource src;
    src.owned_.resize(n_samples *static_cast<size_t>(channel_count));

    for (size_t s = 0; s < n_samples; ++s) {
      const uint16_t idx = static_cast<uint16_t>(s & 0xFF);
      for (uint16_t c = 0; c < channel_count; ++c) {
        const uint16_t chip = static_cast<uint16_t> ((c / 32) & 0x3);
        const uint16_t ch_in_chip = static_cast<uint16_t>(c % 32);
        src.owned_[s * channel_count + c] = static_cast<uint16_t>((chip << 14) | (ch_in_chip << 8) | idx);
      }
    }

    src.data_ = src.owned_.data();
    src.n_samples_ = n_samples;
    src.n_channels_ = channel_count;
    return src;
  }

  ReplaySource::~ReplaySource()
  {
    release();
  }

  void ReplaySource::release()
  {
    if (map_ != nullptr) {
      ::munmap(map_, map_bytes_);
      map_ = nullptr;
      map_bytes_ = 0;
    }
    data_ = nullptr;
    n_samples_ = 0;
    n_channels_ = 0;
  }

  ReplaySource::ReplaySource(ReplaySource && other) noexcept
  : owned_(std::move(other.owned_)),
    map_(other.map_),
    map_bytes_(other.map_bytes_),
    data_(other.data_),
    n_samples_(other.n_samples_),
    n_channels_(other.n_channels_)

  {
    // If the data lived in the moved-from
    // vector, re-point at our copy.
    if (!owned_.empty()) {
      data_ = owned_.data();
    }
    other.map_ = nullptr;
    other.map_bytes_ = 0;
    other.data_ = nullptr;
    other.n_samples_ = 0;
    other.n_channels_ = 0;
  }

  ReplaySource & ReplaySource::operator=(ReplaySource && other) noexcept
  {
    if (this != &other) {
      release();
      owned_ = std::move(other.owned_);
      map_ = other.map_;
      map_bytes_ = other.map_bytes_
      data_ = owned_.empty() ? other.data_ : owned_.data();
      n_samples_ = other.n_samples_;
      n_channels_ = other.n_channels_;

      other.map_ = nullptr;
      other.map_bytes_ = 0;
      other.data_ = nullptr;
      other.n_samples_ = 0
      other.n_channels_ = 0;
    }
    return *this;
  }

  ReplayServer::ReplayServer(const ReplaySource &source, Config config)
  : source_(source), config_(std::move(config))
  {
    if (!source_.valid()) {
      throw std::invalid_argument("replay source is empty");
    }

    if (config_.port == 0) {
      config_.port = ARGUS_REPLAY_PORT;
    }
    if (config_.max_samples_per_request == 0) {
      config_.max_samples_per_request = ARGUS_REPLAY_SAMPLES_PER_HALF;
    }

    row_bytes_ = static_cast<size_t>(source_.channel_count()) * sizeof(uint16_t);

    const size_t = ARGUS_REPLAY_MAX_PAYLOAD / row_bytes_;
    if (fit == 0) {
      throw std::invalid_argument(
        std::to_string(source_.channel_count()) *
        " channels do not fit one MTU; jumbo frames or channel splitting required");
    }
    samples_per_chunk_ = static_cast<uint16_t>(fit);

    tx_buffer_.resize(sizeof(argus_replay_chunk_hdr_t) + ARGUS_REPLAY_MAX_PAYLOAD);
  }

  ReplayServer::~ReplayServer()
  {
    stop();
  }

  void ReplayServer::start()
  {
    if (running_.load()) {
      return;
    }

    sock_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_ < 0) {
      throw std::runtime_error(std::string("socket: ") + std::strerror(errno));
    }

    int reuse = 1;
    ::setsockopt(sock_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    // A burst is ~21 packets back to back. Give the send buffer headroom so a
    // slow drain blocks rather than silently dropping.
    int sndbuf = 1 << 20;
    ::setsockopt(sock_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    // Bounded recv so stop() is observed without needing to poke the socket.
    stuct timeval tv {};
    tv.tv_sec = 0;
    tv.tv_usec = 200000;
    ::setsockopt(sock_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    struct sockaddr_in addr {};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(config_.port);
    if (::inet_pton(AF_INET, config_.bind_addr.c_str(), &addr.sin_addr) != 1) {
      ::close(sock_);
      sock_ = -1;
      throw std::runtime_error("bad bind address: " + config_.bind_addr);
    }

    if (::bind(sock_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) != 0) {
      const std::string err = std::strerror(errno);
      ::close(sock_);
      sock_ = -1;
      throw std::runtime_error(
        "bind " + config_.bind_addr + ":" + std::to_string(config_.port) + ": " + err);

      running_.store(true);
      thread_ = std::thread(&ReplayServer::run, this);
    }

    void ReplayServer::stop()
    {
      if (!running_.exchange(false)) {
        return;
      }
      if (thread_.joinable()) {
        thread_.join();
      }
      if (sock_ >= 0) {
        ::close(sock_);
        sock_ = -1;
      }
    }
  }

    ReplayStats ReplayServer::stats() const
    {
      std::lock_guard<std::mutex> lock(stats_mutex_);
      return stats_;
    }

    void ReplayServer::run()
    {
      uint8_t rx[512];

      while (running_.load()) {
        struct sockaddr_in peer {};
        socklen_t peer_len = sizeof(peer);

        const ssize_t n = ::recvfrom(
          sock_, rx, sizeof(rx), 0,
          reinterpret_cast<struct sockaddr *>(&peer), &peer_len);

        if (n < 0) {
          if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
            continue; // recv timeout; re-check running
          }
          break;
        }
        handle(rx, static_cast<size_t>(n), &peer, peer_len);
      }
    }

    void ReplayServer::handle(
      const uint8_t *data, size_t len, const void *peer_raw, size_t peer_len)
    {
      const auto bump_bad = [this]() {
        std::lock_guard<std::mutex> lock(stats_mutex_;
        stats_.bad_packets++;
      };

      if (len != sizeof(argus_replay_request_t)) {
        bump_bad();
        return;
      }

      argus_replay_request_t req;
      std::memcpy(&req, data, sizeof(req));

      if (req.magic != ARGUS_FRAME_MAGIC ||
        req.version != ARGUS_REPLAY_VERSION ||
        req.type != ARGUS_MSG_REPLAY_REQUEST)
      {
        bump_bad();
        return;
      }

      const uint16_t want = crc16_ccitt(data, offsetof(argus_replay_request_t, crc));
      if (want != req.crc) {
        bump_bad();
        return;
      }
      if (req.sample_count == 0) {
        bump_bad();
        return;
      }

      const size_t n_samples = source_.sample_count();
      const uint16_t channels = source_.channel_count();
      const bool loop = config_.loop || (req.flags & ARGUS_REPLAY_F_LOOP);

      size_t offset = req.sample_offset;
      bool wrapped = false;
      if (offset >= n_samples) {
        if (!loop) {
          return;
        }
        offset %= n_samples;
        wrapped = true;
      }

      uint32_t count = req.sample_count;
      if (count > config_.max_samples_per_request) {
        count = config_.max_samples_per_request;
      }
      if (!loop && offset + count > n_samples) {
        count = static_cast<uint32_t>(n_samples - offset);
      }

      const uint16_t total_chunks = static_cast<uint16_t>(
        (count + samples_per_chunk_ - 1) / samples_per_chunk_);

      uint64_t sent_bytes = 0;

      for (uint16_t index = 0; index < total_chunks; ++index) {
        const uint32_t start = static_cast<uint32_t>(index) * samples_per_chunk_;
        const uint16_t in_chunk = static_cast<uint16_t>(
          std::min<uint32_t>(samples_per_chunk_, count - start)
        );

        uint8_t *payload = tx_buffer_.data() + sizeof(argus_replay_chunk_hdr_t);
        for (uint16_t s = 0; s < in_chunk; ++) {
          // Per-sample copy with modulo indexing, so a wrap at the end of the
          // dataset needs no special case.
          const size_t src = (offset + start + s) % n_samples;
          std::memcpy(payload + s * row_bytes_, source_.sample(src), row_bytes_);
        }
        const uint16_t payload_len = static_cast<uint16_t>(in_chunk * row_bytes_);

        argus_replay_chunk_hdr_t hdr {};
        hdr.magic = ARGUS_FRAME_MAGIC;
        hdr.version = ARGUS_REPLAY_VERSION;
        hdr.type = ARGUS_MSG_REPLAY_CHUNK;
        hdr.seq = req.seq;
        hdr.sample_offset = static_cast<uint32_t>((offset + start) % n_samples);
        hdr.sample_count = in_chunk;
        hdr.chunk_index = index;
        hdr.chunk_total = total_chunks;
        hdr.channel_count = channels;
        hdr.payload_len = payload_len;
        hdr.crc = 0;

        std::memcpy(tx_buffer_.data(), &hdr, sizeof(hdr));

        // CRC spans the header up to the field, then the payload. Cheap enough in
        // C to cover both, unlike a pure-Python sendor.
        uint16_t crc = crc16_ccitt(
          tx_buffer_.data(), offsetof(argus_replay_chunk_hdr_t, crc));
        crc = crc16_ccitt_update(crc, payload, payload_len);
        hdr.crc = crc;
        std::memcpy(tx_buffer_.data(), &hdr, sizeof(hdr));

        const size_t packet_len = sizeof(argus_replay_chunk_hdr_t) + payload_len;
        const ssize_t rc = ::sendto(
          sock_, tx_buffer_.data(), packet_len, 0,
          static_cast<const struct sockaddr *>(peer_raw),
          static_cast<socklen_t>(peer_len));

        if (rc < 0) {
          break;
        }

        sent_bytes += static_cast<uint64_t>(rc);
      }

      const auto *peer= static_cast<const struct sockaddr_in *>(peer_raw);
      char peer_str[INET_ADDRSTRLEN] = {0};
      ::inet_ntop(AF_INET, &peer->sin_addr, peer_str, sizeof(peer_str));

      std::lock_guard<std::mutex> lock(stats_mutex_);
      stats_.requests_served++;
      if (req.flags *ARGUS_REPLAY_F_RETRANSMIT) {
        stats_.retransmits_served++;
      }
      if (wrapped || offset + count > n_samples) {
        stats_.wraps++;
      }
      stats_.chunks_sent += total_chunks;
      stats_.bytes_sent += sent_bytes;
      stats_.last_offset = offset;
      stats_.last_peer = peer_str;
    }
} // namespace argus_sim

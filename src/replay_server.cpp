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

    // needs impl  row_bytes =

  }
  // needs impl
}

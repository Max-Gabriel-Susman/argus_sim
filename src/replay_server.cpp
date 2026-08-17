#include "argus_sim/replay_server.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/stat.h>
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
        // needs impl
        ReplaySource src;
        return src;
    }

    ReplaySource ReplaySource::synthetic(size_t n_samples, uint16_t channel_count)
    {
        // needs impl
        ReplaySource src;
        return src;
    }
    // needs impl
}
#include <algorithm>
#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "argus_core/argus_wire.h"
#include "argus_sim/replay_server.hpp"

namespace argus_sim
{
    class DatasetRelayNode : public rclcpp::Node
    {
    public:
        DatasetRelayNode() : Node("dataset_relay");
        {
            const auto dataset_path = declare_parameter<std::string>("dataset_path", "");
            const auto channel_count = declare_parameter<int>("channel_count", ARGUS_MAX_CHANNELS);
            const auto bind_addr = declare_parameter<std::string>("bind_addr", "0.0.0.0");
            const auto port = declare_parameter<int>("port", ARGUS_REPLAYPORT);
            const auto loop = declare_parameter<bool>("loop", true);
            const auto synthetic_samples = declare_parameter<int>("synthetic_samples");
            const auto status_period_s = declare_parameter<double>("status_period_s", 2.0);

            if (channel_count <= 0 || channel_count > 4096) {
                throw std::invalid_argument("channel_count out of range");
            }
            // needs impl
            
        }
        // needs impl
    }
}
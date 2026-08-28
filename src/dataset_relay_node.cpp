/*
 *  Thin ROS wrapper around ReplayServer: parameters in, status out.
 *  The socket loop runs on ReplayServer's own thread, never on executor.
 *  The PS works to a millisecond-scale latency budget for a buffer swap;
 *  an executor shared with a status timer and parameter callbacks is not
 *  a place to put that.
 */

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
  DatasetRelayNode()
  : Node("dataset_relay")
  {
    const auto bind_addr = declare_parameter<std::string>("bind_addr", "0.0.0.0");
    const auto channel_count = declare_parameter<int>("channel_count", ARGUS_MAX_CHANNELS);
    const auto dataset_path = declare_parameter<std::string>("dataset_path", "");
    const auto drop_mask = declare_parameter<int>("drop_mask", 0);
    const auto loop = declare_parameter<bool>("loop", true);
    const auto port = declare_parameter<int>("port", ARGUS_REPLAY_PORT);
    const auto status_period_s = declare_parameter<double>("status_period_s", 2.0);
    const auto synthetic_samples = declare_parameter<int>("synthetic_samples", 30000);

    if (channel_count <= 0 || channel_count > 4096) {
      throw std::invalid_argument("channel_count out of range");
    }

    if (dataset_path.empty()) {
      source_ = ReplaySource::synthetic(
        static_cast<size_t>(synthetic_samples),
        static_cast<uint16_t>(channel_count));
      RCLCPP_WARN(
        get_logger(),
        "no dataset_path set; serving %d samples of synthetic identity pattern",
        static_cast<int>(synthetic_samples));
    } else {
      source_ = ReplaySource::from_file(dataset_path, static_cast<uint16_t>(channel_count));
      RCLCPP_INFO(get_logger(), "dataset: %s", dataset_path.c_str());
    }

    ReplayServer::Config cfg;
    cfg.bind_addr = bind_addr;
    cfg.port = static_cast<uint16_t>(port);
    cfg.loop = loop;
    cfg.drop_mask = static_cast<uint32_t>(drop_mask);
    if (drop_mask != 0) {
      RCLCPP_WARN(
        get_logger(), "FAULT INJECTION ACTIVE: drop_mask=0x%X -- test use only",
        static_cast<unsigned>(drop_mask));
    }

    server_ = std::make_unique<ReplayServer>(source_, cfg);
    server_->start();

    status_pub_ = create_publisher<std_msgs::msg::String>("~/status", 10);
    status_timer_ = create_wall_timer(
      std::chrono::duration<double>(std::max(0.1, status_period_s)),
      std::bind(&DatasetRelayNode::publish_status, this));

    RCLCPP_INFO(
      get_logger(),
      "replay server on %s:%d -- %zu samples x %u channels, %u samples/chunk, loop=%s",
      bind_addr.c_str(), static_cast<int>(port), source_.sample_count(), source_.channel_count(),
      server_->samples_per_chunk(), loop ? "true" : "false");
    if (channel_count != ARGUS_MAX_CHANNELS) {
      RCLCPP_WARN(
        get_logger(),
        "serving %d channels but ARGUS_MAX_CHANNELS is %d; "
        "the PL and telemetry frame must agree",
        static_cast<int>(channel_count), ARGUS_MAX_CHANNELS);
    }
  }

  ~DatasetRelayNode() override
  {
    if (server_) {
      server_->stop();
    }
  }

private:
  void publish_status()
  {
    const ReplayStats s = server_->stats();

    // Underrun on the PS side shows up here as a request rate below the play
    // rate; a silent gap is worse than a noisy one during bring-up.
    std_msgs::msg::String msg;
    msg.data =
      "requests=" + std::to_string(s.requests_served) +
      " retransmits=" + std::to_string(s.retransmits_served) +
      " chunks=" + std::to_string(s.chunks_sent) +
      " bytes=" + std::to_string(s.bytes_sent) +
      " bad=" + std::to_string(s.bad_packets) +
      " wraps=" + std::to_string(s.wraps) +
      " offset=" + std::to_string(s.last_offset) +
      " peer=" + (s.last_peer.empty() ? std::string("none") : s.last_peer);
    status_pub_->publish(msg);

    if (s.bad_packets != last_bad_) {
      RCLCPP_WARN(
        get_logger(), "%lu malformed request(s) rejected",
        static_cast<unsigned long>(s.bad_packets - last_bad_));
      last_bad_ = s.bad_packets;
    }
  }

  ReplaySource source_;
  std::unique_ptr<ReplayServer> server_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  uint64_t last_bad_ {0};
};
} // namespace argus_sim

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<argus_sim::DatasetRelayNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("dataset_relay"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}

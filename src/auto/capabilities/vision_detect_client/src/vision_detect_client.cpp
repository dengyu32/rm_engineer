#include "vision_detect_client/vision_detect_client.hpp"

#include <chrono>

namespace engineer_auto::vision_detect_client {

VisionDetectClient::VisionDetectClient(rclcpp::Node &node)
    : node_(node), logger_(node.get_logger()), config_(VisionDetectClientConfig::load(node)) {
  RCLCPP_INFO(logger_, "\n%s", config_.summary().c_str());
  target_sub_ = node_.create_subscription<engineer_interfaces::msg::Target>(
      config_.vision_topic, rclcpp::QoS(10),
      std::bind(&VisionDetectClient::onTarget, this, std::placeholders::_1));
  RCLCPP_INFO(logger_, "[VISION_CLIENT] subscribed topic=%s max_age_ms=%d",
              config_.vision_topic.c_str(), config_.max_data_age_ms);
}

bool VisionDetectClient::detect(VisionDetectionResult &out) {
  std::scoped_lock lock(mutex_);
  if (!has_target_) {
    last_error_ = "no vision target received yet";
    return false;
  }

  const auto now = node_.now();
  const auto age_ms = (now - last_stamp_).seconds() * 1000.0;
  if (config_.max_data_age_ms > 0 && age_ms > static_cast<double>(config_.max_data_age_ms)) {
    last_error_ = "vision target stale";
    return false;
  }

  out = last_detection_;
  last_error_.clear();
  return true;
}

void VisionDetectClient::onTarget(const engineer_interfaces::msg::Target::SharedPtr msg) {
  if (!msg) {
    return;
  }

  std::scoped_lock lock(mutex_);
  last_detection_.pose = msg->target_pose;
  last_detection_.vector = msg->target_vector;
  last_stamp_ = node_.now();
  has_target_ = true;
}

} // namespace engineer_auto::vision_detect_client

#include "vision_detect_client/vision_detect_client.hpp"

#include <array>
#include <chrono>

#include "task_orchestrator/protocol.hpp"

namespace engineer_auto::vision_detect_client {

using step_executor::Command;
using step_executor::ExecuteResult;
using step_executor::ExecuteStatus;

VisionDetectClient::VisionDetectClient(rclcpp::Node &node)
    : node_(node), logger_(node.get_logger()), config_(VisionDetectClientConfig::load(node)) {
  RCLCPP_INFO(logger_, "\n%s", config_.summary().c_str());
  target_sub_ = node_.create_subscription<engineer_interfaces::msg::Target>(
      config_.vision_topic, rclcpp::QoS(10),
      std::bind(&VisionDetectClient::onTarget, this, std::placeholders::_1));
  RCLCPP_INFO(logger_, "[VISION_CLIENT] subscribed topic=%s max_age_ms=%d",
              config_.vision_topic.c_str(), config_.max_data_age_ms);
}

ExecuteResult VisionDetectClient::execute(const Command &cmd) {
  (void)cmd;
  ExecuteResult result{};

  VisionDetectionResult detection{};
  if (detect(detection)) {
    result.status = ExecuteStatus::Succeeded;
    const std::array<double, 7> pose{
        detection.pose.x,
        detection.pose.y,
        detection.pose.z,
        detection.pose.qx,
        detection.pose.qy,
        detection.pose.qz,
        detection.pose.qw,
    };
    const std::array<double, 3> vec{
        detection.vector.x,
        detection.vector.y,
        detection.vector.z,
    };
    result.outputs[task_orchestrator::protocol::kVisionPose] = pose;
    result.outputs[task_orchestrator::protocol::kVisionVector] = vec;
    return result;
  }

  if (last_error_ == "no vision target received yet" || last_error_ == "vision target stale") {
    result.status = ExecuteStatus::Running;
    return result;
  }

  result.status = ExecuteStatus::Failed;
  result.error.message = last_error_.empty() ? "vision command failed" : last_error_;
  result.error.retriable = true;
  return result;
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

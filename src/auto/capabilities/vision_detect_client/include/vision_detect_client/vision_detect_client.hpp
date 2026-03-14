#pragma once

#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <engineer_interfaces/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <engineer_interfaces/msg/target.hpp>

namespace engineer_auto::vision_detect_client {

struct VisionDetectionResult {
  engineer_interfaces::msg::Pose pose{};
  geometry_msgs::msg::Vector3 vector{};
};

struct VisionDetectClientConfig {
  std::string vision_topic{"/vision/target"};
  int max_data_age_ms{1000};

  static VisionDetectClientConfig load(rclcpp::Node &node);
};

class VisionDetectClient {
public:
  explicit VisionDetectClient(rclcpp::Node &node);

  bool detect(VisionDetectionResult &out);

  const std::string &lastError() const { return last_error_; }

private:
  void onTarget(const engineer_interfaces::msg::Target::SharedPtr msg);

private:
  rclcpp::Node &node_;
  rclcpp::Logger logger_;
  VisionDetectClientConfig config_;
  rclcpp::Subscription<engineer_interfaces::msg::Target>::SharedPtr target_sub_;

  mutable std::mutex mutex_;
  bool has_target_{false};
  VisionDetectionResult last_detection_{};
  rclcpp::Time last_stamp_{0, 0, RCL_ROS_TIME};

  std::string last_error_;
};

} // namespace engineer_auto::vision_detect_client

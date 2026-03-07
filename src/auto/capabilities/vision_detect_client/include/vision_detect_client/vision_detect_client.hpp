#pragma once

#include <string>

#include <rclcpp/rclcpp.hpp>

namespace engineer_auto::vision_detect_client {

class VisionDetectClient {
public:
  explicit VisionDetectClient(rclcpp::Node &node) : logger_(node.get_logger()) {}

  bool detect() {
    last_error_ = "vision detect not implemented";
    return false;
  }

  const std::string &lastError() const { return last_error_; }

private:
  rclcpp::Logger logger_;
  std::string last_error_;
};

} // namespace engineer_auto::vision_detect_client

#pragma once

#include <string>

#include "vision_detect_client/vision_detect_client.hpp"
#include "step_executor/types/capability_bridge.hpp"

namespace step_executor {

class VisionCapabilityBridge {
public:
  explicit VisionCapabilityBridge(rclcpp::Node &node);

  ExecuteResult run(const Command &cmd);
  void cancel();
  const char *lastError() const { return last_error_.c_str(); }

private:
  engineer_auto::vision_detect_client::VisionDetectClient client_;
  std::string last_error_;
};

} // namespace step_executor

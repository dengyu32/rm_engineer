#pragma once

#include <string>

#include "vision_detect_client/vision_detect_client.hpp"
#include "step_executor/types/capability_bridge.hpp"

namespace engineer_auto::auto_bridge {

// ============================================================================
//  VisionCapabilityBridge
// ----------------------------------------------------------------------------
//  - 适配 vision_detect_client
// ============================================================================

class VisionCapabilityBridge {
public:
  explicit VisionCapabilityBridge(rclcpp::Node &node);

  step_executor::ExecuteResult run(const step_executor::Command &cmd);
  void cancel();
  const char *lastError() const { return last_error_.c_str(); }

private:
  engineer_auto::vision_detect_client::VisionDetectClient client_;
  std::string last_error_;
};

} // namespace engineer_auto::auto_bridge

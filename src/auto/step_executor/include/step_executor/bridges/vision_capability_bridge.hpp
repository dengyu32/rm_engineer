#pragma once

#include <string>

#include "step_executor/capability_bridge.hpp"
#include "vision_detect_client/vision_detect_client.hpp"

namespace step_executor {

class VisionCapabilityBridge {
public:
  explicit VisionCapabilityBridge(rclcpp::Node &node);

  BridgeResult runVisionStep(const task_step_library::Step &step,
                             task_step_library::StepResult *out_result);
  void cancel() {}
  const char *lastError() const { return last_error_.c_str(); }

private:
  engineer_auto::vision_detect_client::VisionDetectClient client_;
  std::string last_error_;
};

} // namespace step_executor

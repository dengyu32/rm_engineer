#pragma once

#include <string>

#include "step_executor/bridges/arm_capability_bridge.hpp"
#include "step_executor/bridges/gripper_capability_bridge.hpp"
#include "step_executor/bridges/vision_capability_bridge.hpp"

namespace step_executor {

class CompositeCapabilityBridge : public ICapabilityBridge {
public:
  explicit CompositeCapabilityBridge(rclcpp::Node &node);

  BridgeResult runStep(const task_step_library::Step &step) override;
  void cancel() override;
  const char *lastError() const override { return last_error_.c_str(); }

private:
  ArmCapabilityBridge arm_bridge_;
  GripperCapabilityBridge gripper_bridge_;
  VisionCapabilityBridge vision_bridge_;
  std::string last_error_;
};

} // namespace step_executor

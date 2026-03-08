#pragma once

#include <string>

#include "gripper_control_node/gripper_control_node.hpp"
#include "step_executor/capability_bridge.hpp"

namespace step_executor {

class GripperCapabilityBridge {
public:
  explicit GripperCapabilityBridge(rclcpp::Node &node);

  BridgeResult runGripperStep(const task_step_library::Step &step,
                              task_step_library::StepResult *out_result);
  void cancel();
  const char *lastError() const { return last_error_.c_str(); }

private:
  engineer_auto::gripper_control_node::GripperControlNode node_;
  std::string last_error_;
};

} // namespace step_executor

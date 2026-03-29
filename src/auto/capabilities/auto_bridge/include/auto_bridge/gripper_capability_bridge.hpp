#pragma once

#include <string>

#include "gripper_control_node/gripper_control_node.hpp"
#include "step_executor/types/capability_bridge.hpp"

namespace engineer_auto::auto_bridge {

// ============================================================================
//  GripperCapabilityBridge
// ----------------------------------------------------------------------------
//  - 适配 gripper_control_node
// ============================================================================

class GripperCapabilityBridge {
public:
  explicit GripperCapabilityBridge(rclcpp::Node &node);

  step_executor::ExecuteResult run(const step_executor::Command &cmd);
  void cancel();
  const char *lastError() const { return last_error_.c_str(); }

private:
  engineer_auto::gripper_control_node::GripperControlNode node_;
  std::string last_error_;
};

} // namespace engineer_auto::auto_bridge

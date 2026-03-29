#pragma once

#include <string>

#include "gripper_control_node/gripper_control_node.hpp"
#include "step_executor/types/capability_bridge.hpp"

namespace step_executor {

class GripperCapabilityBridge {
public:
  explicit GripperCapabilityBridge(rclcpp::Node &node);

  ExecuteResult run(const Command &cmd);
  void cancel();
  const char *lastError() const { return last_error_.c_str(); }

private:
  engineer_auto::gripper_control_node::GripperControlNode node_;
  std::string last_error_;
};

} // namespace step_executor

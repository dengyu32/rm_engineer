#include "step_executor/bridges/gripper_capability_bridge.hpp"

namespace step_executor {

GripperCapabilityBridge::GripperCapabilityBridge(rclcpp::Node &node)
    : node_(node, engineer_auto::gripper_control_node::GripperPresetConfig::load(node)) {}

BridgeResult
GripperCapabilityBridge::runGripperStep(const task_step_library::Step &step,
                                        task_step_library::StepResult *out_result) {
  if (step.type != task_step_library::StepType::Gripper) {
    last_error_ = "unsupported step type for gripper bridge";
    return BridgeResult::Failed;
  }

  node_.setCommand(step.gripper.command);
  last_error_.clear();
  return BridgeResult::Succeeded;
}

void GripperCapabilityBridge::cancel() { node_.cancel(); }

} // namespace step_executor

#include "step_executor/bridges/composite_capability_bridge.hpp"

namespace step_executor {

CompositeCapabilityBridge::CompositeCapabilityBridge(rclcpp::Node &node)
    : arm_bridge_(node), gripper_bridge_(node), vision_bridge_(node) {}

BridgeResult CompositeCapabilityBridge::runStep(const task_step_library::Step &step) {
  switch (step.type) {
  case task_step_library::StepType::ArmMove: {
    const BridgeResult result = arm_bridge_.runArmStep(step);
    if (result == BridgeResult::Failed) {
      last_error_ = arm_bridge_.lastError();
    } else {
      last_error_.clear();
    }
    return result;
  }
  case task_step_library::StepType::Gripper: {
    const BridgeResult result = gripper_bridge_.runGripperStep(step);
    if (result == BridgeResult::Failed) {
      last_error_ = gripper_bridge_.lastError();
    } else {
      last_error_.clear();
    }
    return result;
  }
  case task_step_library::StepType::Guard:
    if (step.guard_id < 0) {
      last_error_ = "guard rejected";
      return BridgeResult::Failed;
    }
    last_error_.clear();
    return BridgeResult::Succeeded;
  case task_step_library::StepType::Delay:
    return BridgeResult::Succeeded;
  default:
    last_error_ = "unsupported step type";
    return BridgeResult::Failed;
  }
}

void CompositeCapabilityBridge::cancel() {
  arm_bridge_.cancel();
  gripper_bridge_.cancel();
  vision_bridge_.cancel();
}

} // namespace step_executor

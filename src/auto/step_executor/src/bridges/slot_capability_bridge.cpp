#include "step_executor/bridges/slot_capability_bridge.hpp"

namespace step_executor {
using task_step_library::StepType;

SlotCapabilityBridge::SlotCapabilityBridge(rclcpp::Node &node)
    : node_(node, engineer_auto::slot_select_node::SlotSelectConfig::load(node)) {}

BridgeResult SlotCapabilityBridge::runSlotStep(const task_step_library::Step &step,
                                               task_step_library::StepResult *out_result) {
  if (step.type != StepType::Slot) {
    last_error_ = "unsupported step type for slot bridge";
    return BridgeResult::Failed;
  }

  switch (step.slot.strategy) {
  case task_step_library::SlotStrategy::SelectSlotToPut:
  case task_step_library::SlotStrategy::SelectSlotToTake: {
    int selected_slot = -1;
    if (!node_.selectSlot(step.slot.strategy, selected_slot)) {
      last_error_ = node_.lastError();
      if (last_error_.empty()) {
        last_error_ = "slot selection failed";
      }
      return BridgeResult::Failed;
    }
    last_selected_slot_ = selected_slot;
    if (out_result) {
      out_result->has_selected_slot = true;
      out_result->selected_slot = selected_slot;
    }
    break;
  }
  case task_step_library::SlotStrategy::LockSlot:
  case task_step_library::SlotStrategy::UnlockSlot:
    if (!node_.applySlotCommand(step.slot.strategy, step.slot.slot_id)) {
      last_error_ = node_.lastError();
      if (last_error_.empty()) {
        last_error_ = "slot command failed";
      }
      return BridgeResult::Failed;
    }
    break;
  default:
    last_error_ = "unsupported slot strategy";
    return BridgeResult::Failed;
  }

  last_error_.clear();
  return BridgeResult::Succeeded;
}

} // namespace step_executor

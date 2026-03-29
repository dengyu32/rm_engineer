#include "auto_bridge/slot_capability_bridge.hpp"

#include "slot_select_node/slot_types.hpp"
#include "task_orchestrator/protocol.hpp"
#include "step_executor/types/step.hpp"

namespace step_executor {

namespace slot_types = engineer_auto::slot_select_node;

SlotCapabilityBridge::SlotCapabilityBridge(rclcpp::Node &node)
    : node_(node, engineer_auto::slot_select_node::SlotSelectConfig::load(node)) {}

ExecuteResult SlotCapabilityBridge::run(const Command &cmd) {
  ExecuteResult result{};

  if (cmd.kind == task_orchestrator::protocol::kSlotSelectKind) {
    const auto *strategy = paramAs<std::string>(cmd, "strategy");
    if (!strategy) {
      result.status = ExecuteStatus::Failed;
      result.error.code = ErrorCode::ValidationError;
      result.error.message = "slot.select missing strategy";
      result.error.retriable = false;
      last_error_ = result.error.message;
      return result;
    }

    slot_types::SlotStrategy slot_strategy =
        slot_types::SlotStrategy::SelectSlotToPut;
    if (*strategy == "put") {
      slot_strategy = slot_types::SlotStrategy::SelectSlotToPut;
    } else if (*strategy == "take") {
      slot_strategy = slot_types::SlotStrategy::SelectSlotToTake;
    } else {
      result.status = ExecuteStatus::Failed;
      result.error.code = ErrorCode::ValidationError;
      result.error.message = "slot.select invalid strategy: " + *strategy;
      result.error.retriable = false;
      last_error_ = result.error.message;
      return result;
    }

    int selected_slot = -1;
    if (!node_.selectSlot(slot_strategy, selected_slot)) {
      last_error_ = node_.lastError();
      if (last_error_.empty()) {
        last_error_ = "slot selection failed";
      }
      result.status = ExecuteStatus::Failed;
      result.error.code = ErrorCode::ExecutionError;
      result.error.message = last_error_;
      result.error.retriable = true;
      return result;
    }

    result.status = ExecuteStatus::Succeeded;
    result.outputs[task_orchestrator::protocol::kSlotId] = selected_slot;
    last_error_.clear();
    return result;
  }

  if (cmd.kind == task_orchestrator::protocol::kSlotLockKind ||
      cmd.kind == task_orchestrator::protocol::kSlotUnlockKind) {
    int slot_id = -1;
    if (!getParam(cmd, "slot_id", slot_id)) {
      result.status = ExecuteStatus::Failed;
      result.error.code = ErrorCode::ValidationError;
      result.error.message = "slot command missing slot_id";
      result.error.retriable = false;
      last_error_ = result.error.message;
      return result;
    }

    const auto strategy = (cmd.kind == task_orchestrator::protocol::kSlotLockKind)
                              ? slot_types::SlotStrategy::LockSlot
                              : slot_types::SlotStrategy::UnlockSlot;
    if (!node_.applySlotCommand(strategy, slot_id)) {
      last_error_ = node_.lastError();
      if (last_error_.empty()) {
        last_error_ = "slot command failed";
      }
      result.status = ExecuteStatus::Failed;
      result.error.code = ErrorCode::ExecutionError;
      result.error.message = last_error_;
      result.error.retriable = true;
      return result;
    }

    result.status = ExecuteStatus::Succeeded;
    last_error_.clear();
    return result;
  }

  result.status = ExecuteStatus::Failed;
  result.error.code = ErrorCode::ValidationError;
  result.error.message = "unsupported command kind for slot";
  result.error.retriable = false;
  last_error_ = result.error.message;
  return result;
}

void SlotCapabilityBridge::cancel() {}

} // namespace step_executor

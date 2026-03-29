#include "auto_bridge/slot_capability_bridge.hpp"

#include "task_orchestrator/protocol.hpp"
#include "step_executor/types/command.hpp"
#include "step_executor/types/execute_result.hpp"

namespace engineer_auto::auto_bridge {

namespace slot_select = engineer_auto::slot_select_node;

using step_executor::Command;
using step_executor::ExecuteResult;
using step_executor::ExecuteStatus;
using step_executor::ErrorCode;

SlotCapabilityBridge::SlotCapabilityBridge(rclcpp::Node &node)
    : node_(node, engineer_auto::slot_select_node::SlotSelectConfig::load(node)) {}

ExecuteResult SlotCapabilityBridge::run(const Command &cmd) {
  ExecuteResult exec{};
  if (cmd.kind == task_orchestrator::protocol::kSlotSelectKind) {
    exec = node_.executeSelect(cmd);
  } else if (cmd.kind == task_orchestrator::protocol::kSlotLockKind) {
    exec = node_.executeLockUnlock(cmd, slot_select::SlotStrategy::LockSlot);
  } else if (cmd.kind == task_orchestrator::protocol::kSlotUnlockKind) {
    exec = node_.executeLockUnlock(cmd, slot_select::SlotStrategy::UnlockSlot);
  } else {
    exec.status = ExecuteStatus::Failed;
    exec.error.code = ErrorCode::ValidationError;
    exec.error.message = "unsupported command kind for slot";
    exec.error.retriable = false;
  }

  if (exec.status == ExecuteStatus::Failed && exec.error.message.empty()) {
    exec.error.message = "slot command failed";
  }

  if (exec.status == ExecuteStatus::Failed) {
    last_error_ = exec.error.message;
  } else {
    last_error_.clear();
  }

  return exec;
}

void SlotCapabilityBridge::cancel() {}

} // namespace engineer_auto::auto_bridge

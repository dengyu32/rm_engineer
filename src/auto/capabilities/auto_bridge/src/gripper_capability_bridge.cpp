// capability layer
#include "auto_bridge/gripper_capability_bridge.hpp"

// task layer
#include "task_orchestrator/protocol.hpp"

// step layer
#include "step_executor/types/command.hpp"
#include "step_executor/types/execute_result.hpp"

// namespace aliases
namespace engineer_auto::auto_bridge {

using step_executor::Command;
using step_executor::ExecuteResult;
using step_executor::ExecuteStatus;
using step_executor::ErrorCode;

GripperCapabilityBridge::GripperCapabilityBridge(rclcpp::Node &node)
    : node_(node, engineer_auto::gripper_control_node::GripperPresetConfig::load(node)) {}

ExecuteResult GripperCapabilityBridge::run(const Command &cmd) {
  ExecuteResult result{};

  // 非GripperKind的命令的情况
  if (cmd.kind != task_orchestrator::protocol::kGripperKind) {
    result.status = ExecuteStatus::Failed;
    result.error.code = ErrorCode::ValidationError;
    result.error.message = "unsupported command kind for gripper";
    result.error.retriable = false;
    last_error_ = result.error.message;
    return result;
  }

  auto exec = node_.execute(cmd);

  if (exec.status == ExecuteStatus::Failed && exec.error.message.empty()) {
    exec.error.message = "gripper command failed";
  }

  if (exec.status == ExecuteStatus::Failed) {
    last_error_ = exec.error.message;
  } else {
    last_error_.clear();
  }

  return exec;
}

void GripperCapabilityBridge::cancel() { node_.cancel(); }

} // namespace engineer_auto::auto_bridge

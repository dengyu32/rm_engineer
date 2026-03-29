#include "auto_bridge/gripper_capability_bridge.hpp"

#include "gripper_control_node/gripper_types.hpp"
#include "task_orchestrator/protocol.hpp"
#include "step_executor/types/step.hpp"

namespace step_executor {

namespace gripper_types = engineer_auto::gripper_control_node;

GripperCapabilityBridge::GripperCapabilityBridge(rclcpp::Node &node)
    : node_(node, engineer_auto::gripper_control_node::GripperPresetConfig::load(node)) {}

ExecuteResult GripperCapabilityBridge::run(const Command &cmd) {
  ExecuteResult result{};

  if (cmd.kind != task_orchestrator::protocol::kGripperKind) {
    result.status = ExecuteStatus::Failed;
    result.error.code = ErrorCode::ValidationError;
    result.error.message = "unsupported command kind for gripper";
    result.error.retriable = false;
    last_error_ = result.error.message;
    return result;
  }

  const auto *action = paramAs<std::string>(cmd, "action");
  if (!action) {
    result.status = ExecuteStatus::Failed;
    result.error.code = ErrorCode::ValidationError;
    result.error.message = "gripper command missing action";
    result.error.retriable = false;
    last_error_ = result.error.message;
    return result;
  }

  gripper_types::GripperCommand command = gripper_types::GripperCommand::OPEN;
  if (*action == "open") {
    command = gripper_types::GripperCommand::OPEN;
  } else if (*action == "close") {
    command = gripper_types::GripperCommand::CLOSE;
  } else {
    result.status = ExecuteStatus::Failed;
    result.error.code = ErrorCode::ValidationError;
    result.error.message = "gripper action invalid: " + *action;
    result.error.retriable = false;
    last_error_ = result.error.message;
    return result;
  }

  node_.setCommand(command);
  result.status = ExecuteStatus::Succeeded;
  last_error_.clear();
  return result;
}

void GripperCapabilityBridge::cancel() { node_.cancel(); }

} // namespace step_executor

// capabilities layer
#include "auto_bridge/arm_capability_bridge.hpp"

// task layer
#include "task_orchestrator/protocol.hpp"

// step layer
#include "step_executor/types/command.hpp"

// namespace aliases
namespace engineer_auto::auto_bridge {

using step_executor::Command;
using step_executor::ExecuteResult;
using step_executor::ExecuteStatus;
using step_executor::ErrorCode;

ArmCapabilityBridge::ArmCapabilityBridge(rclcpp::Node &node)
    : client_(node, engineer_auto::arm_solve_client::ArmSolveClientConfig::load(node)) {}

ExecuteResult ArmCapabilityBridge::run(const Command &cmd) {
  ExecuteResult result{};

  // 非ArmMoveKind的命令的情况
  if (cmd.kind != task_orchestrator::protocol::kArmMoveKind) {
    result.status = ExecuteStatus::Failed;
    result.error.code = ErrorCode::ValidationError;
    result.error.message = "unsupported command kind for arm";
    result.error.retriable = false;
    last_error_ = result.error.message;
    return result;
  }

  // 执行（参数校验在 arm_solve_client 内部）
  const auto exec = client_.execute(cmd);

  // 为何要这么处理？
  const auto pick_error = [&]() -> std::string {
    if (exec.error.has_value()) {
      return *exec.error;
    }
    return client_.lastError();
  };

  // 由于是异步调用，返回Running是合理的,同时使用重入Re-entrant机制，内层只有request变化时才会抢占
  switch (exec.status) {
  case ExecuteStatus::Running:
    result.status = ExecuteStatus::Running;
    last_error_.clear();
    return result;
  case ExecuteStatus::Succeeded:
    result.status = ExecuteStatus::Succeeded;
    last_error_.clear();
    return result;
  case ExecuteStatus::Failed:
    result.status = ExecuteStatus::Failed;
    result.error.code = exec.error_code;
    result.error.message = pick_error();
    if (result.error.message.empty()) {
      result.error.message = "arm command failed";
    }
    result.error.retriable = (exec.error_code != ErrorCode::ValidationError);
    last_error_ = result.error.message;
    return result;
  default:
    result.status = ExecuteStatus::Failed;
    result.error.code = ErrorCode::Unknown;
    result.error.message = "arm command unknown status";
    result.error.retriable = true;
    last_error_ = result.error.message;
    return result;
  }
}

void ArmCapabilityBridge::cancel() { client_.cancel(); }

} // namespace engineer_auto::auto_bridge

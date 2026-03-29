#include "auto_bridge/arm_capability_bridge.hpp"

#include "arm_solve_client/arm_types.hpp"
#include "task_orchestrator/protocol.hpp"
#include "step_executor/types/command.hpp"
#include "step_executor/types/step.hpp"

namespace step_executor {

namespace arm_types = engineer_auto::arm_solve_client;

ArmCapabilityBridge::ArmCapabilityBridge(rclcpp::Node &node)
    : client_(node, engineer_auto::arm_solve_client::ArmSolveClientConfig::load(node)) {}

ExecuteResult ArmCapabilityBridge::run(const Command &cmd) {
  ExecuteResult result{};

  if (cmd.kind != task_orchestrator::protocol::kArmMoveKind) {
    result.status = ExecuteStatus::Failed;
    result.error.code = ErrorCode::ValidationError;
    result.error.message = "unsupported command kind for arm";
    result.error.retriable = false;
    last_error_ = result.error.message;
    return result;
  }

  arm_types::ArmMoveSpec request{};
  if (const auto *pose = paramAs<engineer_interfaces::msg::Pose>(cmd, "target_pose")) {
    request.plan_option = arm_types::PlanOption::NORMAL;
    request.pose = *pose;
  } else if (const auto *joints = paramAs<std::array<float, 6>>(cmd, "target_joints")) {
    request.plan_option = arm_types::PlanOption::JOINTS;
    request.joints = *joints;
  } else if (const auto *vec = paramAs<geometry_msgs::msg::Vector3>(cmd, "target_vector")) {
    request.plan_option = arm_types::PlanOption::CARTESIAN;
    request.vector = *vec;
  } else {
    result.status = ExecuteStatus::Failed;
    result.error.code = ErrorCode::ValidationError;
    result.error.message = "arm command missing target";
    result.error.retriable = false;
    last_error_ = result.error.message;
    return result;
  }

  using Status = engineer_auto::arm_solve_client::CommandStatus;
  const auto exec = client_.execute(request);
  const Status status = exec.status;
  const auto pick_error = [&]() -> std::string {
    if (exec.error.has_value()) {
      return *exec.error;
    }
    return client_.lastError();
  };

  switch (status) {
  case Status::Started:
  case Status::Tracking:
    result.status = ExecuteStatus::Running;
    last_error_.clear();
    return result;
  case Status::Succeeded:
    result.status = ExecuteStatus::Succeeded;
    last_error_.clear();
    return result;
  case Status::StartFailed:
  case Status::Failed:
    result.status = ExecuteStatus::Failed;
    result.error.code = ErrorCode::ExecutionError;
    result.error.message = pick_error();
    if (result.error.message.empty()) {
      result.error.message = "arm command failed";
    }
    result.error.retriable = true;
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

} // namespace step_executor

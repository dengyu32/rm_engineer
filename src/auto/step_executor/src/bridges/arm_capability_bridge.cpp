#include "step_executor/bridges/arm_capability_bridge.hpp"

namespace step_executor {

ArmCapabilityBridge::ArmCapabilityBridge(rclcpp::Node &node)
    : client_(node, engineer_auto::arm_solve_client::ArmSolveClientConfig::load(node)) {}

BridgeResult ArmCapabilityBridge::runArmStep(const task_step_library::Step &step,
                                             task_step_library::StepResult *out_result) {
  if (step.type != task_step_library::StepType::ArmMove) {
    last_error_ = "unsupported step type for arm bridge";
    return BridgeResult::Failed;
  }

  using Status = engineer_auto::arm_solve_client::CommandStatus;
  const Status status = client_.execute(step.arm_move);
  switch (status) {
  case Status::Started:
  case Status::Tracking:
    return BridgeResult::Running;
  case Status::Succeeded:
    last_error_.clear();
    return BridgeResult::Succeeded;
  case Status::StartFailed:
  case Status::Failed:
    last_error_ = client_.lastError();
    if (last_error_.empty()) {
      last_error_ = "arm step failed";
    }
    return BridgeResult::Failed;
  default:
    last_error_ = "arm step unknown status";
    return BridgeResult::Failed;
  }
}

void ArmCapabilityBridge::cancel() { client_.cancel(); }

} // namespace step_executor

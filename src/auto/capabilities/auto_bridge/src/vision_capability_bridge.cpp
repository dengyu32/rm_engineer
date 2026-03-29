#include "auto_bridge/vision_capability_bridge.hpp"

#include <array>

#include "task_orchestrator/protocol.hpp"
#include "step_executor/types/step.hpp"

namespace engineer_auto::auto_bridge {

using step_executor::Command;
using step_executor::ExecuteResult;
using step_executor::ExecuteStatus;
using step_executor::ErrorCode;

VisionCapabilityBridge::VisionCapabilityBridge(rclcpp::Node &node) : client_(node) {}

ExecuteResult VisionCapabilityBridge::run(const Command &cmd) {
  ExecuteResult result{};

  if (cmd.kind != task_orchestrator::protocol::kVisionKind) {
    result.status = ExecuteStatus::Failed;
    result.error.code = ErrorCode::ValidationError;
    result.error.message = "unsupported command kind for vision";
    result.error.retriable = false;
    last_error_ = result.error.message;
    return result;
  }

  engineer_auto::vision_detect_client::VisionDetectionResult detection{};

  if (client_.detect(detection)) {
    result.status = ExecuteStatus::Succeeded;
    const std::array<double, 7> pose{
        detection.pose.x,
        detection.pose.y,
        detection.pose.z,
        detection.pose.qx,
        detection.pose.qy,
        detection.pose.qz,
        detection.pose.qw,
    };
    const std::array<double, 3> vec{
        detection.vector.x,
        detection.vector.y,
        detection.vector.z,
    };
    result.outputs[task_orchestrator::protocol::kVisionPose] = pose;
    result.outputs[task_orchestrator::protocol::kVisionVector] = vec;
    last_error_.clear();
    return result;
  }

  last_error_ = client_.lastError();
  if (last_error_ == "no vision target received yet" || last_error_ == "vision target stale") {
    result.status = ExecuteStatus::Running;
    return result;
  }

  result.status = ExecuteStatus::Failed;
  result.error.code = ErrorCode::ExecutionError;
  result.error.message = last_error_.empty() ? "vision command failed" : last_error_;
  result.error.retriable = true;
  return result;
}

void VisionCapabilityBridge::cancel() {}

} // namespace engineer_auto::auto_bridge

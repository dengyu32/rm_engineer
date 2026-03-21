#include "step_executor/bridges/vision_capability_bridge.hpp"

namespace step_executor {

VisionCapabilityBridge::VisionCapabilityBridge(rclcpp::Node &node) : client_(node) {}

BridgeResult VisionCapabilityBridge::runVisionStep(const task_step_library::Step &,
                                                   task_step_library::StepResult *out_result) {
  engineer_auto::vision_detect_client::VisionDetectionResult detection{};

  if (client_.detect(detection)) {
    if (out_result) {
      out_result->set<task_step_library::SharedKey::VisionPose>(detection.pose);
      out_result->set<task_step_library::SharedKey::VisionVector>(detection.vector);
    }
    last_error_.clear();
    return BridgeResult::Succeeded;
  }

  last_error_ = client_.lastError();
  if (last_error_ == "no vision target received yet" || last_error_ == "vision target stale") {
    return BridgeResult::Running;
  }
  if (last_error_.empty()) {
    last_error_ = "vision step not implemented";
  }
  return BridgeResult::Failed;
}

} // namespace step_executor

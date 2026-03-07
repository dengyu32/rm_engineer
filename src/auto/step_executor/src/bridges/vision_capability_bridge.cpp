#include "step_executor/bridges/vision_capability_bridge.hpp"

namespace step_executor {

VisionCapabilityBridge::VisionCapabilityBridge(rclcpp::Node &node) : client_(node) {}

BridgeResult VisionCapabilityBridge::runVisionStep(const task_step_library::Step &) {
  // Placeholder bridge: the vision path is intentionally left unimplemented for now.
  if (client_.detect()) {
    last_error_.clear();
    return BridgeResult::Succeeded;
  }

  last_error_ = client_.lastError();
  if (last_error_.empty()) {
    last_error_ = "vision step not implemented";
  }
  return BridgeResult::Failed;
}

} // namespace step_executor

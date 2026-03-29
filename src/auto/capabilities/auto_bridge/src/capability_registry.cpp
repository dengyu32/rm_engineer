#include "auto_bridge/capability_registry.hpp"

#include "auto_bridge/arm_capability_bridge.hpp"
#include "auto_bridge/gripper_capability_bridge.hpp"
#include "auto_bridge/slot_capability_bridge.hpp"
#include "auto_bridge/vision_capability_bridge.hpp"

#include "step_executor/registry_bridge.hpp"
#include "task_orchestrator/protocol.hpp"

namespace engineer_auto {

std::shared_ptr<step_executor::ICapabilityBridge> createAutoCapabilityBridge(
    rclcpp::Node &node) {
  auto registry = std::make_shared<step_executor::RegistryBridge>();

  auto register_bridge = [&registry](auto bridge, const std::string &kind, auto run_fn) {
    registry->registerHandler(
        kind,
        [bridge, run_fn](const step_executor::Command &cmd) {
          return (bridge.get()->*run_fn)(cmd);
        },
        [bridge]() { bridge->cancel(); },
        [bridge]() { return bridge->lastError(); });
  };

  auto arm_bridge = std::make_shared<step_executor::ArmCapabilityBridge>(node);
  register_bridge(arm_bridge, task_orchestrator::protocol::kArmMoveKind,
                  &step_executor::ArmCapabilityBridge::run);

  auto gripper_bridge = std::make_shared<step_executor::GripperCapabilityBridge>(node);
  register_bridge(gripper_bridge, task_orchestrator::protocol::kGripperKind,
                  &step_executor::GripperCapabilityBridge::run);

  auto slot_bridge = std::make_shared<step_executor::SlotCapabilityBridge>(node);
  register_bridge(slot_bridge, task_orchestrator::protocol::kSlotSelectKind,
                  &step_executor::SlotCapabilityBridge::run);
  register_bridge(slot_bridge, task_orchestrator::protocol::kSlotLockKind,
                  &step_executor::SlotCapabilityBridge::run);
  register_bridge(slot_bridge, task_orchestrator::protocol::kSlotUnlockKind,
                  &step_executor::SlotCapabilityBridge::run);

  auto vision_bridge = std::make_shared<step_executor::VisionCapabilityBridge>(node);
  register_bridge(vision_bridge, task_orchestrator::protocol::kVisionKind,
                  &step_executor::VisionCapabilityBridge::run);

  return registry;
}

} // namespace engineer_auto

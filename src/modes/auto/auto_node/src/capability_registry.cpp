#include "auto_node/capability_registry.hpp"

#include "auto_library/context_keys.hpp"
#include "step_executor/registry_bridge.hpp"
#include "auto_library/execute_result.hpp"

#include "arm_solve_client/arm_solve_client.hpp"
#include "gripper_control_node/gripper_control_node.hpp"
#include "slot_select_node/slot_select_node.hpp"
#include "vision_detect_client/vision_detect_client.hpp"

#include <stdexcept>
#include <utility>

namespace engineer_auto::auto_node {

namespace armc = engineer_auto::arm_solve_client;
namespace gripc = engineer_auto::gripper_control_node;
namespace slotc = engineer_auto::slot_select_node;
namespace visc = engineer_auto::vision_detect_client;

using core::makeFailed;
using core::KindSpec;

namespace {

void addKind(step_executor::RegistryBridge &bridge,
             core::KindSpecMap &kind_specs,
             KindSpec spec,
             step_executor::RegistryBridge::HandlerFn run_fn,
             step_executor::RegistryBridge::CancelFn cancel_fn = {}) {
  const std::string kind = spec.kind;
  if (!bridge.registerHandler(kind, std::move(run_fn), std::move(cancel_fn))) {
    throw std::runtime_error("duplicate auto handler registration: " + kind);
  }
  if (!kind_specs.emplace(kind, std::move(spec)).second) {
    throw std::runtime_error("duplicate auto kind spec registration: " + kind);
  }
}

void registerArmCapabilities(step_executor::RegistryBridge &bridge,
                             core::KindSpecMap &kind_specs,
                             rclcpp::Node &node) {
  auto arm = std::make_shared<armc::ArmSolveClient>(
      node, armc::ArmSolveClientConfig::load(node));

  addKind(
      bridge,
      kind_specs,
      KindSpec{"arm.move_pose", {"target_pose"}, {"target_pose"}, {}},
      [arm](const auto &cmd) {
        armc::ArmMoveSpec spec;
        std::string error;
        return arm->buildPoseSpec(cmd, spec, error) ? arm->execute(spec)
                                                    : makeFailed(error, false);
      },
      [arm]() { arm->cancel(); });

  addKind(
      bridge,
      kind_specs,
      KindSpec{"arm.move_joints", {"target_joints"}, {"target_joints"}, {}},
      [arm](const auto &cmd) {
        armc::ArmMoveSpec spec;
        std::string error;
        return arm->buildJointsSpec(cmd, spec, error) ? arm->execute(spec)
                                                      : makeFailed(error, false);
      },
      [arm]() { arm->cancel(); });

  addKind(
      bridge,
      kind_specs,
      KindSpec{"arm.move_vector",
               {"target_vector", "target_length"},
               {"target_vector", "target_length"},
               {}},
      [arm](const auto &cmd) {
        armc::ArmMoveSpec spec;
        std::string error;
        return arm->buildVectorSpec(cmd, spec, error) ? arm->execute(spec)
                                                      : makeFailed(error, false);
      },
      [arm]() { arm->cancel(); });
}

void registerGripperCapabilities(step_executor::RegistryBridge &bridge,
                                 core::KindSpecMap &kind_specs,
                                 rclcpp::Node &node) {
  auto gripper = std::make_shared<gripc::GripperControlNode>(
      node, gripc::GripperPresetConfig::load(node));

  addKind(
      bridge,
      kind_specs,
      KindSpec{"gripper.open", {}, {}, {}},
      [gripper](const auto &) { return gripper->executeOpen(); },
      [gripper]() { gripper->cancel(); });

  addKind(
      bridge,
      kind_specs,
      KindSpec{"gripper.close", {}, {}, {}},
      [gripper](const auto &) { return gripper->executeClose(); },
      [gripper]() { gripper->cancel(); });
}

void registerSlotCapabilities(step_executor::RegistryBridge &bridge,
                              core::KindSpecMap &kind_specs,
                              rclcpp::Node &node) {
  auto slot = std::make_shared<slotc::SlotSelectNode>(
      node, slotc::SlotSelectConfig::load(node));

  addKind(
      bridge,
      kind_specs,
      KindSpec{"slot.select_put", {}, {}, {core::keys::kSlotId}},
      [slot](const auto &) { return slot->executeSelectPut(); });

  addKind(
      bridge,
      kind_specs,
      KindSpec{"slot.select_take", {}, {}, {core::keys::kSlotId}},
      [slot](const auto &) { return slot->executeSelectTake(); });

  addKind(
      bridge,
      kind_specs,
      KindSpec{"slot.lock", {"slot_id"}, {"slot_id"}, {}},
      [slot](const auto &cmd) { return slot->executeLock(cmd); });

  addKind(
      bridge,
      kind_specs,
      KindSpec{"slot.unlock", {"slot_id"}, {"slot_id"}, {}},
      [slot](const auto &cmd) { return slot->executeUnlock(cmd); });
}

void registerVisionCapabilities(step_executor::RegistryBridge &bridge,
                                core::KindSpecMap &kind_specs,
                                rclcpp::Node &node) {
  auto vision = std::make_shared<visc::VisionDetectClient>(node);

  addKind(
      bridge,
      kind_specs,
      KindSpec{"vision.detect",
               {"enable"},
               {},
               {core::keys::kVisionPose, core::keys::kVisionVector}},
      [vision](const auto &cmd) { return vision->execute(cmd); },
      [vision]() { vision->cancel(); });
}

} // namespace

void registerAutoCapabilities(step_executor::RegistryBridge &bridge,
                              core::KindSpecMap &kind_specs,
                              rclcpp::Node &node) {
  registerArmCapabilities(bridge, kind_specs, node);
  registerGripperCapabilities(bridge, kind_specs, node);
  registerSlotCapabilities(bridge, kind_specs, node);
  registerVisionCapabilities(bridge, kind_specs, node);
}

} // namespace engineer_auto::auto_node

#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include <engineer_interfaces/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include "shared_data/context.hpp"

namespace task_step_library {

// ARM MOVE

enum class PlanOption : uint8_t {
  NORMAL = 0,
  CARTESIAN = 1,
  JOINTS = 2,
};

enum class TargetSource : uint8_t {
  Fixed = 0,
  SharedPose = 1,
  SharedVector = 2,
  SlotMapped = 3,
};

struct ArmMoveSpec {
  engineer_interfaces::msg::Pose pose{};
  std::array<float, 6> joints{{0.f, 0.f, 0.f, 0.f, 0.f, 0.f}};

  geometry_msgs::msg::Vector3 vector{};

  PlanOption plan_option{PlanOption::NORMAL};
  TargetSource target_source{TargetSource::Fixed};
  SharedKey target_key{SharedKey::VisionPose};
};

/*
struct ArmMoveSpec {
  struct NormalSpec {
    engineer_interfaces::msg::Pose pose{};
  } normal;
  struct CartesianSpec {
    geometry_msgs::msg::Vector3 direction{};
    double distance;
  } cartesian;
  struct JointSpec {
     std::array<float, 6> joints{{0.f, 0.f, 0.f, 0.f, 0.f, 0.f}};
  } joint;
  PlanOption plan_option{PlanOption::NORMAL};
  TargetSource target_source{TargetSource::Fixed};
};
*/

// GRIPPER CONTROL

enum class GripperCommand : uint8_t {
  OPEN = 0,
  CLOSE = 1,
};

struct GripperSpec {
  GripperCommand command{GripperCommand::OPEN};
};

// SLOT STRATEGY

enum class SlotStrategy : uint8_t {
  SelectSlotToPut = 0,
  LockSlot = 1,
  SelectSlotToTake = 2,
  UnlockSlot = 3,
};

struct SlotSpec {
  SlotStrategy strategy{SlotStrategy::SelectSlotToPut};
  int slot_id{-1}; // for lock/unlock specific slot, ignored for select modes
};


// STEP

enum class StepType : uint8_t {
  ArmMove = 0,
  Gripper = 1,
  Delay = 2,
  Guard = 3,
  Slot = 4,
  Vision = 5,
};

struct Step {
  StepType type{StepType::Delay};
  std::string label{};
  int timeout_ms{0};
  int max_retries{0};

  std::vector<SharedKey> inputs{};
  std::vector<SharedKey> outputs{};

  ArmMoveSpec arm_move{};
  GripperSpec gripper{};
  SlotSpec slot{};
  int delay_ms{0};
  int guard_id{0};
};

} // namespace task_step_library

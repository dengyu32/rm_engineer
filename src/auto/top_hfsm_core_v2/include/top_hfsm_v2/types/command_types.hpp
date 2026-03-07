#pragma once

#include <array>
#include <cstdint>

#include <engineer_interfaces/msg/target.hpp>

namespace top_hfsm_v2 {

// ARM COMMAND

enum class PlanOptionType : uint8_t {
  NORMAL = 0,
  CARTESIAN = 1,
  JOINTS = 2,
};

enum class ArmCommandStatus : uint8_t {
  START_FAILED = 0,
  STARTED = 1,
  TRACKING = 2,
  SUCCEEDED = 3,
  FAILED = 4,
};

struct ArmCommandSpec {
  engineer_interfaces::msg::Target pose{};
  std::array<float, 6> joints{{0.f, 0.f, 0.f, 0.f, 0.f, 0.f}};
  PlanOptionType plan_option{PlanOptionType::NORMAL};
};

// GRIPPER COMMAND

enum class GripperCommandType : uint8_t {
  OPEN = 0,
  CLOSE = 1,
};

struct GripperCommandSpec {
  GripperCommandType command{GripperCommandType::OPEN};
};

// STEP COMMAND

enum class CommandStepKind : uint8_t {
  ArmMove = 0,
  Gripper = 1,
  Delay = 2,
  Guard = 3,
};

struct CommandStep {
  CommandStepKind kind{CommandStepKind::Delay};
  int timeout_ms{0};
  int max_retries{0};
  std::string label{};

  ArmCommandSpec arm_cmd{};
  GripperCommandSpec gripper_cmd{};
  int delay_ms{0};
  int guard_id{0};
};

} // namespace top_hfsm_v2

#pragma once

#include <array>
#include <vector>

#include "top_hfsm_v2/types/command_types.hpp"

namespace top_hfsm_v2 {

inline std::vector<CommandStep> makeAutoInitPlan() {
  std::vector<CommandStep> steps(3);

  steps[0].kind = CommandStepKind::ServoPause;
  steps[0].label = "servo_pause";
  steps[0].timeout_ms = 3000;

  steps[1].kind = CommandStepKind::Gripper;
  steps[1].label = "gripper_open";
  steps[1].gripper_cmd.command = GripperCommandType::OPEN;

  steps[2].kind = CommandStepKind::ArmMove;
  steps[2].label = "move_home_joints";
  steps[2].timeout_ms = 8000;
  steps[2].max_retries = 1;
  steps[2].arm_cmd.plan_option = PlanOptionType::JOINTS;
  steps[2].arm_cmd.joints =
      std::array<float, 6>{{0.f, -1.042f, -2.618f, 0.f, 0.f, 0.f}};

  return steps;
}

} // namespace top_hfsm_v2

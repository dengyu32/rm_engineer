#pragma once

#include <array>
#include <vector>

#include "top_hfsm_v2/types/command_types.hpp"

namespace top_hfsm_v2 {

inline std::vector<CommandStep> makeAutoInitPlan() {
  std::vector<CommandStep> steps(2);

  steps[0].kind = CommandStepKind::Gripper;
  steps[0].label = "gripper_open";
  steps[0].gripper_cmd.command = GripperCommandType::OPEN;

  steps[1].kind = CommandStepKind::ArmMove;
  steps[1].label = "move_home_joints";
  steps[1].timeout_ms = 8000;
  steps[1].max_retries = 1;
  steps[1].arm_cmd.plan_option = PlanOptionType::JOINTS;
  steps[1].arm_cmd.joints =
      std::array<float, 6>{{0.f, -1.042f, -2.618f, 0.f, 0.f, 0.f}};

  return steps;
}

} // namespace top_hfsm_v2

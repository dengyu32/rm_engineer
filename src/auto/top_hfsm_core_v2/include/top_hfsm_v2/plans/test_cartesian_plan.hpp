#pragma once

#include <vector>

#include "top_hfsm_v2/core/make_target.hpp"
#include "top_hfsm_v2/types/command_types.hpp"

namespace top_hfsm_v2 {

inline std::vector<CommandStep> makeTestCartesianPlan() {
  std::vector<CommandStep> steps(5);

  steps[0].kind = CommandStepKind::ArmMove;
  steps[0].label = "approach_normal";
  steps[0].timeout_ms = 8000;
  steps[0].max_retries = 1;
  steps[0].arm_cmd.plan_option = PlanOptionType::NORMAL;
  steps[0].arm_cmd.pose = makeTarget(0.44595, -0.086883, 0.61032, -0.17604,
                                     0.69171, -0.39348, 0.57941);

  steps[1].kind = CommandStepKind::Gripper;
  steps[1].label = "gripper_close";
  steps[1].gripper_cmd.command = GripperCommandType::CLOSE;

  steps[2].kind = CommandStepKind::Delay;
  steps[2].label = "gripper_settle";
  steps[2].delay_ms = 200;

  steps[3].kind = CommandStepKind::ArmMove;
  steps[3].label = "take_out_cartesian";
  steps[3].timeout_ms = 8000;
  steps[3].max_retries = 1;
  steps[3].arm_cmd.plan_option = PlanOptionType::CARTESIAN;
  steps[3].arm_cmd.pose = makeTarget(0.54028, 0.19212, 0.27802, -0.17604,
                                     0.69171, -0.39348, 0.57941);

  steps[4].kind = CommandStepKind::Gripper;
  steps[4].label = "gripper_open";
  steps[4].gripper_cmd.command = GripperCommandType::OPEN;

  return steps;
}

} // namespace top_hfsm_v2

#pragma once

#include <vector>

#include "top_hfsm_v2/core/make_target.hpp"
#include "top_hfsm_v2/types/command_types.hpp"

namespace top_hfsm_v2 {

inline std::vector<CommandStep> makeTestCartesianPlan() {
  std::vector<CommandStep> steps(6);

  steps[0].kind = CommandStepKind::ServoPause;
  steps[0].label = "servo_pause";
  steps[0].timeout_ms = 3000;

  steps[1].kind = CommandStepKind::ArmMove;
  steps[1].label = "approach_normal";
  steps[1].timeout_ms = 8000;
  steps[1].max_retries = 1;
  steps[1].arm_cmd.plan_option = PlanOptionType::NORMAL;
  steps[1].arm_cmd.pose = makeTarget(0.44595, -0.086883, 0.61032, -0.17604,
                                     0.69171, -0.39348, 0.57941);

  steps[2].kind = CommandStepKind::Gripper;
  steps[2].label = "gripper_close";
  steps[2].gripper_cmd.command = GripperCommandType::CLOSE;

  steps[3].kind = CommandStepKind::Delay;
  steps[3].label = "gripper_settle";
  steps[3].delay_ms = 200;

  steps[4].kind = CommandStepKind::ArmMove;
  steps[4].label = "take_out_cartesian";
  steps[4].timeout_ms = 8000;
  steps[4].max_retries = 1;
  steps[4].arm_cmd.plan_option = PlanOptionType::CARTESIAN;
  steps[4].arm_cmd.pose = makeTarget(0.54028, 0.19212, 0.27802, -0.17604,
                                     0.69171, -0.39348, 0.57941);

  steps[5].kind = CommandStepKind::Gripper;
  steps[5].label = "gripper_open";
  steps[5].gripper_cmd.command = GripperCommandType::OPEN;

  return steps;
}

} // namespace top_hfsm_v2

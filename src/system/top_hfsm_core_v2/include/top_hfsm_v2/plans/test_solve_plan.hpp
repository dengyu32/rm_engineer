#pragma once

#include <array>

#include "top_hfsm_v2/core/make_target.hpp"
#include "top_hfsm_v2/types/command_types.hpp"

namespace top_hfsm_v2 {

inline std::vector<CommandStep> makeTestSolvePlan() {
  std::vector<CommandStep> steps(6);

  steps[0].kind = CommandStepKind::ServoPause;
  steps[0].label = "servo_pause";
  steps[0].timeout_ms = 3000;

  steps[1].kind = CommandStepKind::ArmMove;
  steps[1].label = "move_normal_pose0";
  steps[1].timeout_ms = 8000;
  steps[1].max_retries = 1;
  steps[1].arm_cmd.plan_option = PlanOptionType::NORMAL;
  steps[1].arm_cmd.pose = makeTarget(0.26235, -0.34444, 0.40285, 0.64923, 0.17886,
                                 0.18983, 0.71447);

  steps[2].kind = CommandStepKind::Gripper;
  steps[2].label = "gripper_close";
  steps[2].gripper_cmd.command = GripperCommandType::CLOSE;

  steps[3].kind = CommandStepKind::Delay;
  steps[3].label = "gripper_settle";
  steps[3].delay_ms = 600;

  steps[4].kind = CommandStepKind::ArmMove;
  steps[4].label = "move_cartesian_pose1";
  steps[4].timeout_ms = 8000;
  steps[4].max_retries = 1;
  steps[4].arm_cmd.plan_option = PlanOptionType::CARTESIAN;
  steps[4].arm_cmd.pose = makeTarget(0.26484, -0.36076, 0.55316, 0.64923, 0.17886,
                                 0.18983, 0.71447);

  steps[5].kind = CommandStepKind::ArmMove;
  steps[5].label = "move_home_joints";
  steps[5].timeout_ms = 8000;
  steps[5].max_retries = 1;
  steps[5].arm_cmd.plan_option = PlanOptionType::JOINTS;
  steps[5].arm_cmd.joints = std::array<float, 6>{{0.f, -1.042f, -2.618f, 0.f, 0.f, 0.f}};

  return steps;
}

} // namespace top_hfsm_v2

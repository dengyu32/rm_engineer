#pragma once

#include <array>
#include <vector>

#include "top_hfsm_v2/core/make_target.hpp"
#include "top_hfsm_v2/types/command_types.hpp"

namespace top_hfsm_v2 {

inline std::vector<CommandStep> makeAutoStorePlan() {
  std::vector<CommandStep> steps(6);

  steps[0].kind = CommandStepKind::ServoPause;
  steps[0].label = "auto_store_servo_pause";
  steps[0].timeout_ms = 3000;

  steps[1].kind = CommandStepKind::ArmMove;
  steps[1].label = "auto_store_move_to_place";
  steps[1].timeout_ms = 8000;
  steps[1].max_retries = 1;
  steps[1].arm_cmd.plan_option = PlanOptionType::NORMAL;
  steps[1].arm_cmd.pose = makeTarget(0.44595, -0.086883, 0.61032, -0.17604,
                                     0.69171, -0.39348, 0.57941);

  steps[2].kind = CommandStepKind::Delay;
  steps[2].label = "auto_store_pose_settle";
  steps[2].delay_ms = 200;

  steps[3].kind = CommandStepKind::Gripper;
  steps[3].label = "auto_store_gripper_open";
  steps[3].gripper_cmd.command = GripperCommandType::OPEN;

  steps[4].kind = CommandStepKind::Delay;
  steps[4].label = "auto_store_release_settle";
  steps[4].delay_ms = 300;

  steps[5].kind = CommandStepKind::ArmMove;
  steps[5].label = "auto_store_move_home";
  steps[5].timeout_ms = 8000;
  steps[5].max_retries = 1;
  steps[5].arm_cmd.plan_option = PlanOptionType::JOINTS;
  steps[5].arm_cmd.joints =
      std::array<float, 6>{{0.f, -1.042f, -2.618f, 0.f, 0.f, 0.f}};

  return steps;
}

} // namespace top_hfsm_v2

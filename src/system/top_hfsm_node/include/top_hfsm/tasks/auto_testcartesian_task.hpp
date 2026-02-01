// ============================================================================
// AutoTestCartesianTask
// ----------------------------------------------------------------------------
// - 以 steps 表复刻原 TESTCARTESIAN 两阶段抓取/拔出
// ============================================================================
#pragma once

#include "top_hfsm/tasks/sequence_task.hpp"
#include "top_hfsm/target_helper.hpp"

namespace top_hfsm {

class AutoTestCartesianTask {
public:
  AutoTestCartesianTask() { buildSteps(); }

  void reset() { seq_.reset(); }

  void update(EMachineContext &ctx, AsyncDispatcher &disp,
              const rclcpp::Time &now) {
    seq_.update(ctx, disp, now);
  }

  bool finished() const { return seq_.finished(); }
  TaskStatus lastStatus() const { return seq_.lastStatus(); }
  const std::string &lastMsg() const { return seq_.lastMsg(); }

private:
  void buildSteps() {
    std::vector<Step> steps;
    steps.resize(6);

    // (0) ServoPause
    steps[0].kind = StepKind::kServoPause;
    steps[0].name = "servo_pause";

    // (1) NORMAL -> close_to
    steps[1].kind = StepKind::kArmGoal;
    steps[1].name = "approach_normal";
    steps[1].timeout_ms = 8000;
    steps[1].retry_max = 1;
    steps[1].arm.use_pose = true;
    steps[1].arm.option = executors::PlanOptionType::NORMAL;
    steps[1].arm.pose =
        makeTarget(0.44595, -0.086883, 0.61032, -0.17604, 0.69171, -0.39348,
                   0.57941);

    // (2) Gripper close
    steps[2].kind = StepKind::kGripperCmd;
    steps[2].name = "gripper_close";
    steps[2].gripper_cmd = 1;

    // (3) Delay settle (optional)
    steps[3].kind = StepKind::kDelayMs;
    steps[3].name = "gripper_settle";
    steps[3].delay_ms = 200;

    // (4) CARTESIAN -> take_out
    steps[4].kind = StepKind::kArmGoal;
    steps[4].name = "take_out_cartesian";
    steps[4].timeout_ms = 8000;
    steps[4].retry_max = 1;
    steps[4].arm.use_pose = true;
    steps[4].arm.option = executors::PlanOptionType::CARTESIAN;
    steps[4].arm.pose =
        makeTarget(0.54028, 0.19212, 0.27802, -0.17604, 0.69171, -0.39348,
                   0.57941);

    // (5) optional open
    steps[5].kind = StepKind::kGripperCmd;
    steps[5].name = "gripper_open";
    steps[5].gripper_cmd = 0;

    seq_.setSteps(steps);
  }

private:
  SequenceTask seq_;
};

} // namespace top_hfsm

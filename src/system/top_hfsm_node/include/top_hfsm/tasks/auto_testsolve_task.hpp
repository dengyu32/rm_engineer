// ============================================================================
// AutoTestSolveTask
// ----------------------------------------------------------------------------
// - 以 steps 表复刻原 TESTSOLVE 四阶段
// ============================================================================
#pragma once

#include "top_hfsm/tasks/sequence_task.hpp"
#include "top_hfsm/target_helper.hpp"

namespace top_hfsm {

class AutoTestSolveTask {
public:
  AutoTestSolveTask() { buildSteps(); }

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

    // (1) NORMAL -> pose0
    steps[1].kind = StepKind::kArmGoal;
    steps[1].name = "move_normal_pose0";
    steps[1].timeout_ms = 8000;
    steps[1].retry_max = 1;
    steps[1].arm.use_pose = true;
    steps[1].arm.option = executors::PlanOptionType::NORMAL;
    steps[1].arm.pose = makeTarget(0.26235, -0.34444, 0.40285, 0.64923,
                                   0.17886, 0.18983, 0.71447);

    // (2) Gripper CLOSE
    steps[2].kind = StepKind::kGripperCmd;
    steps[2].name = "gripper_close";
    steps[2].gripper_cmd = 1;

    // (3) Delay settle
    steps[3].kind = StepKind::kDelayMs;
    steps[3].name = "gripper_settle";
    steps[3].delay_ms = 600;

    // (4) CARTESIAN -> pose1
    steps[4].kind = StepKind::kArmGoal;
    steps[4].name = "move_cartesian_pose1";
    steps[4].timeout_ms = 8000;
    steps[4].retry_max = 1;
    steps[4].arm.use_pose = true;
    steps[4].arm.option = executors::PlanOptionType::CARTESIAN;
    steps[4].arm.pose = makeTarget(0.26484, -0.36076, 0.55316, 0.64923,
                                   0.17886, 0.18983, 0.71447);

    // (5) JOINTS -> init joints
    steps[5].kind = StepKind::kArmGoal;
    steps[5].name = "move_home_joints";
    steps[5].timeout_ms = 8000;
    steps[5].retry_max = 1;
    steps[5].arm.use_pose = false;
    steps[5].arm.option = executors::PlanOptionType::JOINTS;
    steps[5].arm.joints =
        std::array<float, 6>{{0.f, -1.042f, -2.618f, 0.f, 0.f, 0.f}};

    seq_.setSteps(steps);
  }

private:
  SequenceTask seq_;
};

} // namespace top_hfsm

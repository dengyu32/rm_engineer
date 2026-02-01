// ============================================================================
// SequenceTask
// ----------------------------------------------------------------------------
// - 以 steps 表驱动的任务基类
// - 每个 step 都是非阻塞执行，tick 内推进
// - 仅依赖 C++11
// ============================================================================
#pragma once

#include <cstddef>
#include <array>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "top_hfsm/config.hpp"
#include "top_hfsm/context.hpp"
#include "top_hfsm/util/async_dispatcher.hpp"

namespace top_hfsm {

// ----------------------------------------------------------------------------
// ArmGoalSpec：用于 Step 的动作参数
// ----------------------------------------------------------------------------
struct ArmGoalSpec {
  ArmGoalSpec()
      : use_pose(true),
        pose(),
        joints(),
        option(executors::PlanOptionType::NORMAL) {}

  bool use_pose; // true -> pose, false -> joints
  engineer_interfaces::msg::Target pose;
  std::array<float, 6> joints;
  executors::PlanOptionType option;
};

// ----------------------------------------------------------------------------
// Step 定义
// ----------------------------------------------------------------------------
enum class StepKind {
  kArmGoal = 0,
  kGripperCmd,
  kDelayMs,
  kGuard,
  kServoStart,
  kServoPause
};

struct Step {
  StepKind kind;
  int timeout_ms;    // 0 表示不限时
  int retry_max;     // 0 表示不重试
  std::string name;  // 日志标签

  ArmGoalSpec arm;
  int gripper_cmd;   // 0=open, 1=close
  int delay_ms;      // 用于 kDelayMs
  int guard_id;      // 用于 kGuard

  Step()
      : kind(StepKind::kDelayMs),
        timeout_ms(0),
        retry_max(0),
        name(),
        arm(),
        gripper_cmd(0),
        delay_ms(0),
        guard_id(0) {}
};

// ----------------------------------------------------------------------------
// SequenceTask
// ----------------------------------------------------------------------------
class SequenceTask {
public:
  typedef bool (*GuardEvalFn)(int guard_id, const EMachineContext &ctx);

  SequenceTask() : guard_eval_fn_(defaultGuard) { reset(); }

  void setSteps(const std::vector<Step> &steps) { steps_ = steps; }
  void setGuardEvaluator(GuardEvalFn fn) {
    if (fn)
      guard_eval_fn_ = fn;
  }

  void reset() {
    idx_ = 0;
    step_started_ = false;
    sent_once_ = false;
    retry_left_ = 0;
    finished_ = false;
    last_status_ = TaskStatus::Success;
    last_msg_.clear();
    step_t0_ = rclcpp::Time(0, 0, RCL_STEADY_TIME);
  }

  bool finished() const { return finished_; }
  TaskStatus lastStatus() const { return last_status_; }
  const std::string &lastMsg() const { return last_msg_; }

  void update(EMachineContext &ctx, AsyncDispatcher &dispatcher,
              const rclcpp::Time &now) {
    if (finished_)
      return;

    if (idx_ >= steps_.size()) {
      finished_ = true;
      last_status_ = TaskStatus::Success;
      return;
    }

    Step &step = steps_[idx_];

    if (!step_started_) {
      step_started_ = true;
      sent_once_ = false;
      retry_left_ = step.retry_max;
      step_t0_ = now;
      RCLCPP_INFO(ctx.logger, "[SEQ][step=%zu/%zu] enter: %s",
                  idx_, steps_.size(), step.name.c_str());

      if (step.kind == StepKind::kArmGoal && ctx.arm_client) {
        ctx.arm_client->resetSendOnce();
      }
    }

    // timeout check
    if (step.timeout_ms > 0) {
      const double elapsed_ms = (now - step_t0_).seconds() * 1000.0;
      if (elapsed_ms >= static_cast<double>(step.timeout_ms)) {
        handleFailure(ctx, "timeout", TaskStatus::Timeout);
        return;
      }
    }

    switch (step.kind) {
    case StepKind::kArmGoal:
      handleArmGoal(step, ctx);
      break;

    case StepKind::kGripperCmd:
      handleGripper(step, ctx);
      break;

    case StepKind::kDelayMs:
      handleDelay(step, now);
      break;

    case StepKind::kGuard:
      handleGuard(step, ctx);
      break;

    case StepKind::kServoStart:
      handleServo(step, ctx, dispatcher, true);
      break;

    case StepKind::kServoPause:
      handleServo(step, ctx, dispatcher, false);
      break;

    default:
      handleFailure(ctx, "unknown step kind", TaskStatus::Failure);
      break;
    }
  }

private:
  static bool defaultGuard(int, const EMachineContext &) { return true; }

  void nextStep(EMachineContext &ctx) {
    if (steps_[idx_].kind == StepKind::kArmGoal && ctx.arm_client) {
      ctx.arm_client->resetSendOnce();
    }
    ++idx_;
    step_started_ = false;
  }

  void handleFailure(EMachineContext &ctx, const std::string &msg, TaskStatus status) {
    if (retry_left_ > 0) {
      --retry_left_;
      if (steps_[idx_].kind == StepKind::kArmGoal && ctx.arm_client) {
        ctx.arm_client->resetSendOnce();
      }
      sent_once_ = false;
      step_started_ = true;
      step_t0_ = rclcpp::Clock(RCL_STEADY_TIME).now();
      RCLCPP_WARN(ctx.logger,
                  "[SEQ][step=%zu] retry (%d left): %s",
                  idx_, retry_left_, msg.c_str());
      return;
    }

    finished_ = true;
    last_status_ = status;
    last_msg_ = msg;
    RCLCPP_ERROR(ctx.logger,
                 "[SEQ][step=%zu] failed: %s (status=%d) -> IDLE",
                 idx_, msg.c_str(), static_cast<int>(status));
  }

  void handleArmGoal(Step &step, EMachineContext &ctx) {
    if (!ctx.arm_client) {
      handleFailure(ctx, "arm_client null", TaskStatus::Failure);
      return;
    }

    executors::SendResultType res;
    if (!sent_once_) {
      if (step.arm.use_pose) {
        res = ctx.arm_client->sentArmGoal(step.arm.pose, step.arm.option);
      } else {
        res = ctx.arm_client->sentArmGoal(step.arm.joints, step.arm.option);
      }
      sent_once_ = true;
    } else {
      if (step.arm.use_pose) {
        res = ctx.arm_client->sentArmGoal(step.arm.pose, step.arm.option);
      } else {
        res = ctx.arm_client->sentArmGoal(step.arm.joints, step.arm.option);
      }
    }

    switch (res) {
    case executors::SendResultType::SENT:
    case executors::SendResultType::IN_PROGRESS:
      return;
    case executors::SendResultType::SUCCEEDED:
      nextStep(ctx);
      return;
    case executors::SendResultType::SEND_FAILED:
    case executors::SendResultType::FAILED:
      {
        auto actx = ctx.arm_client->activeContext();
        std::string msg = actx ? actx->error_msg : std::string();
        if (msg.empty()) {
          msg = (res == executors::SendResultType::SEND_FAILED) ? "arm goal send failed" : "arm goal failed";
        }
        handleFailure(ctx, msg, TaskStatus::Failure);
      }
      return;
    default:
      handleFailure(ctx, "arm goal unknown", TaskStatus::Failure);
      return;
    }
  }

  void handleGripper(Step &step, EMachineContext &ctx) {
    if (!ctx.gripper_node) {
      handleFailure(ctx, "gripper null", TaskStatus::Failure);
      return;
    }
    const executors::GripperStatusType status =
        step.gripper_cmd == 0 ? executors::GripperStatusType::OPEN
                              : executors::GripperStatusType::CLOSE;
    ctx.gripper_node->setGripperStatus(status);
    nextStep(ctx);
  }

  void handleDelay(Step &step, const rclcpp::Time &now) {
    const double elapsed_ms = (now - step_t0_).seconds() * 1000.0;
    if (elapsed_ms >= static_cast<double>(step.delay_ms)) {
      ++idx_;
      step_started_ = false;
    }
  }

  void handleGuard(Step &step, const EMachineContext &ctx) {
    if (guard_eval_fn_(step.guard_id, ctx)) {
      ++idx_;
      step_started_ = false;
    } else {
      finished_ = true;
      last_status_ = TaskStatus::Failure;
      last_msg_ = "guard rejected";
    }
  }

  void handleServo(Step &step, EMachineContext &ctx, AsyncDispatcher &disp,
                   bool start) {
    if (!ctx.servo_client) {
      handleFailure(ctx, "servo null", TaskStatus::Failure);
      return;
    }
    if (!sent_once_) {
      if (start) {
        ctx.servo_client->requestStartServo(disp);
      } else {
        ctx.servo_client->requestPauseServo(disp);
      }
      sent_once_ = true;
      return;
    }

    if (ctx.servo_client->servoPending()) {
      return;
    }

    const TaskStatus status = ctx.servo_client->lastServoStatus();
    const std::string msg = ctx.servo_client->lastServoMsg();
    if (status == TaskStatus::Success) {
      nextStep(ctx);
      return;
    }
    handleFailure(ctx, msg.empty() ? "servo failed" : msg, status);
  }

private:
  std::vector<Step> steps_;
  GuardEvalFn guard_eval_fn_;

  std::size_t idx_;
  bool step_started_;
  bool sent_once_;
  int retry_left_;
  bool finished_;
  TaskStatus last_status_;
  std::string last_msg_;
  rclcpp::Time step_t0_;
};

} // namespace top_hfsm

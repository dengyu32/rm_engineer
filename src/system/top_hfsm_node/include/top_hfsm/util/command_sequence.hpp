// ============================================================================
// CommandSequence.hpp
// ----------------------------------------------------------------------------
// - 封装Step序列，编排HFSM命令
// ============================================================================
#pragma once

#include <cstddef>
#include <array>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "top_hfsm/context.hpp"
#include "top_hfsm/util/async_dispatcher.hpp"

namespace top_hfsm {

// ============================================================================
// 业务动作参数 : 用于 Step 的动作参数
// ----------------------------------------------------------------------------
// - ArmGoalSpec
// - GrippperGoalSpec
// - GuardID
// - DelayMs 
// ============================================================================

// ----------------------------------------------------------------------------
// ArmGoalSpec：用于 Step 的动作参数
// ----------------------------------------------------------------------------
struct ArmGoalSpec {
  ArmGoalSpec()
      : pose(),
        joints(),
        option(executors::PlanOptionType::NORMAL) {}

  engineer_interfaces::msg::Target pose;
  std::array<float, 6> joints;
  executors::PlanOptionType option;
};

// ----------------------------------------------------------------------------
// GripperGoalSpec: 用于 Step 的夹爪命令参数
// ----------------------------------------------------------------------------
struct GripperGoalSpec {
  int cmd; // 0=open, 1=close

  GripperGoalSpec() : cmd(0) {}
};

// ============================================================================
// Step : 命令序列每一步封装
// ----------------------------------------------------------------------------
// - StepKind 枚举
// - 通用参数：timeout_ms, retry_max, log_name 
// - 业务动作参数
// ============================================================================

// ----------------------------------------------------------------------------
// StepKind 业务动作类型枚举
// ----------------------------------------------------------------------------
enum class StepKind {
  kArmMove = 0,
  kGripperCmd,
  kDelayMs,
  kSafeGuard,
  kServoStart,
  kServoPause
};


struct Step {
  StepKind kind;
  int timeout_ms;    // 0 表示不限时
  int retry_max;     // 0 表示不重试
  std::string log_name;  // 日志标签

  // 业务动作参数
  ArmGoalSpec arm;
  GripperGoalSpec gripper;
  int delay_ms;      // 用于 kDelayMs
  int guard_id;      // 用于 kGuard

  Step()
      : kind(StepKind::kDelayMs),
        timeout_ms(0),
        retry_max(0),
        log_name(),
        arm(),
        gripper(0),
        delay_ms(0),
        guard_id(0) {}
};

// ============================================================================
// CommandSequence : 命令序列类
// ----------------------------------------------------------------------------
// - 类型
//    - GuardEvaluator 检查Guard条件的函数指针
// - 接口
//    - setSteps() 设置Step序列
//    - setGuardEvaluator() 设置Guard条件检查函数
//
//    - reset() 重置序列状态
//    - update() 更新序列状态，执行当前Step
//
//    - finished() 查询序列是否已完成
//    - lastStatus() 查询最后的执行状态
//    - lastMsg() 查询最后的执行消息

// 
// ============================================================================

class CommandSequence {
public:
  using GuardEvaluator = std::function<bool(int guard_id, const EMachineContext &ctx)>;

  CommandSequence() : guard_eval_func_(defaultGuard) { reset(); }

  void setSteps(const std::vector<Step> &steps) { steps_ = steps; }
  void setGuardEvaluator(GuardEvaluator fn) {
    if (fn)
      guard_eval_func_ = fn;
  }

  void reset() {
    step_idx_ = 0;
    step_started_ = false;
    sent_once_ = false;
    retry_left_ = 0;
    all_finished_ = false;
    last_status_ = TaskStatus::Success;
    last_msg_.clear();
    step_t0_ = rclcpp::Time(0, 0, RCL_STEADY_TIME);
  }

  bool finished() const { return all_finished_; }

  TaskStatus lastStatus() const { return last_status_; }

  const std::string &lastMsg() const { return last_msg_; }

  void update(EMachineContext &ctx, AsyncDispatcher &dispatcher,
              const rclcpp::Time &now) {
    if (all_finished_)
      return;

    if (step_idx_ >= steps_.size()) {
      all_finished_ = true;
      last_status_ = TaskStatus::Success;
      return;
    }

    Step &step = steps_[step_idx_];

    // Step 首次 update
    if (!step_started_) {
      step_started_ = true;
      sent_once_ = false;
      retry_left_ = step.retry_max;
      step_t0_ = now;
      RCLCPP_INFO(ctx.logger, "[SEQ][step=%zu/%zu] enter: %s",
                  step_idx_, steps_.size(), step.log_name.c_str());

      if (step.kind == StepKind::kArmMove && ctx.arm_client) {
        ctx.arm_client->resetSendOnce();
      }
    }

    // 检查是否超时
    if (step.timeout_ms > 0) {
      const double elapsed_ms = (now - step_t0_).seconds() * 1000.0;
      if (elapsed_ms >= static_cast<double>(step.timeout_ms)) {
        handleFailure(ctx, "timeout", TaskStatus::Timeout);
        return;
      }
    }

    // 根据 Step 类型执行不同的处理逻辑
    switch (step.kind) {
    case StepKind::kArmMove:
      handleArmGoal(step, ctx);
      break;

    case StepKind::kGripperCmd:
      handleGripper(step, ctx);
      break;

    case StepKind::kDelayMs:
      handleDelay(step, now);
      break;

    case StepKind::kSafeGuard:
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
  // --------------------------------------------------------------------------
  // GuardFunc Guard函数
  //
  // - defaultGuard: 默认Guard函数，始终返回true
  // --------------------------------------------------------------------------
  static bool defaultGuard(int, const EMachineContext &) { return true; }

  void nextStep(EMachineContext &ctx) {
    if (steps_[step_idx_].kind == StepKind::kArmMove && ctx.arm_client) {
      ctx.arm_client->resetSendOnce();
    }
    ++step_idx_;
    step_started_ = false;
  }

  void handleFailure(EMachineContext &ctx, const std::string &msg, TaskStatus status) {
    if (retry_left_ > 0) {
      --retry_left_;
      if (steps_[step_idx_].kind == StepKind::kArmMove && ctx.arm_client) {
        ctx.arm_client->resetSendOnce();
      }
      sent_once_ = false;
      step_started_ = true;
      step_t0_ = rclcpp::Clock(RCL_STEADY_TIME).now();
      RCLCPP_WARN(ctx.logger,
                  "[SEQ][step=%zu] retry (%d left): %s",
                  step_idx_, retry_left_, msg.c_str());
      return;
    }

    all_finished_ = true;  // 一步错，整个序列结束
    last_status_ = status;
    last_msg_ = msg;
    RCLCPP_ERROR(ctx.logger,
                 "[SEQ][step=%zu] failed: %s (status=%d) -> IDLE",
                 step_idx_, msg.c_str(), static_cast<int>(status));
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
    if (guard_eval_func_(step.guard_id, ctx)) {
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
  GuardEvaluator guard_eval_func_; // 存放函数指针

  // 内部运行状态
  std::size_t step_idx_;
  bool step_started_;  // 这一步是否已经开始 每一Step有很多次update
  rclcpp::Time step_t0_; // 当前step开始时间

  bool sent_once_;  
  int retry_left_;  // 当前步骤剩余重试次数
  bool all_finished_; // 整个序列是否已完成（成功或失败）
  TaskStatus last_status_; // ???
  std::string last_msg_;
};

} // namespace top_hfsm

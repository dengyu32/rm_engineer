#include "top_hfsm_v2/core/command_sequence.hpp"

namespace top_hfsm_v2 {

CommandSequence::CommandSequence() : guard_evaluator_(defaultGuard) { reset(); }

void CommandSequence::setGuardEvaluator(GuardEvaluator evaluator) {
  if (evaluator) {
    guard_evaluator_ = std::move(evaluator);
  }
}

void CommandSequence::reset() {
  current_step_index_ = 0;
  current_step_entered_ = false;
  remaining_retries_ = 0;
  is_finished_ = false;
  result_status_ = TaskStatus::Success;
  result_message_.clear();
  current_step_start_time_ = rclcpp::Time(0, 0, RCL_STEADY_TIME);
}

void CommandSequence::update(RuntimeContext &context, const rclcpp::Time &now) {
  if (is_finished_) {
    return;
  }

  if (current_step_index_ >= steps_.size()) {
    is_finished_ = true;
    result_status_ = TaskStatus::Success;
    return;
  }

  const CommandStep &step = steps_[current_step_index_];

  if (!current_step_entered_) {
    current_step_entered_ = true;
    remaining_retries_ = step.max_retries;
    current_step_start_time_ = now;
    RCLCPP_INFO(context.logger, "[COMMAND_SEQUENCE][step=%zu/%zu] enter: %s",
                current_step_index_, steps_.size(), step.label.c_str());
  }

  if (step.timeout_ms > 0) {
    const double elapsed_ms =
        (now - current_step_start_time_).seconds() * 1000.0;
    if (elapsed_ms >= static_cast<double>(step.timeout_ms)) {
      handleStepFailure(context, "timeout", TaskStatus::Timeout, now);
      return;
    }
  }

  switch (step.kind) {
  case CommandStepKind::ArmMove:
    handleArmMove(step, context, now);
    break;
  case CommandStepKind::Gripper:
    handleGripper(step, context);
    break;
  case CommandStepKind::Delay:
    handleDelay(step, now);
    break;
  case CommandStepKind::Guard:
    handleGuard(step, context);
    break;
  case CommandStepKind::ServoStart:
    handleServo(step, context, true, now);
    break;
  case CommandStepKind::ServoPause:
    handleServo(step, context, false, now);
    break;
  default:
    handleStepFailure(context, "unknown step kind", TaskStatus::Failure, now);
    break;
  }
}

bool CommandSequence::defaultGuard(int, const RuntimeContext &) { return true; }

void CommandSequence::advanceToNextStep() {
  ++current_step_index_;
  current_step_entered_ = false;
}

void CommandSequence::handleStepFailure(RuntimeContext &context,
                                        const std::string &message,
                                        TaskStatus status,
                                        const rclcpp::Time &now) {
  if (remaining_retries_ > 0) {
    --remaining_retries_;
    current_step_start_time_ = now;
    RCLCPP_WARN(context.logger,
                "[COMMAND_SEQUENCE][step=%zu] retry (%d left): %s",
                current_step_index_, remaining_retries_, message.c_str());
    return;
  }

  is_finished_ = true;
  result_status_ = status;
  result_message_ = message;
  RCLCPP_ERROR(context.logger,
               "[COMMAND_SEQUENCE][step=%zu] failed: %s (status=%d)",
               current_step_index_, message.c_str(), static_cast<int>(status));
}

void CommandSequence::handleArmMove(const CommandStep &step,
                                    RuntimeContext &context,
                                    const rclcpp::Time &now) {
  if (!context.arm_client) {
    handleStepFailure(context, "arm_client null", TaskStatus::Failure, now);
    return;
  }

  const ArmCommandStatus send_result =
      context.arm_client->execute_arm_command(step.arm_cmd);

  switch (send_result) {
  case ArmCommandStatus::STARTED:
  case ArmCommandStatus::TRACKING:
    return;
  case ArmCommandStatus::SUCCEEDED:
    advanceToNextStep();
    return;
  case ArmCommandStatus::START_FAILED:
  case ArmCommandStatus::FAILED: {
    std::string message = context.arm_client->lastErrorMessage();
    if (message.empty()) {
      message = (send_result == ArmCommandStatus::START_FAILED)
                    ? "arm goal send failed"
                    : "arm goal failed";
    }
    handleStepFailure(context, message, TaskStatus::Failure, now);
    return;
  }
  default:
    handleStepFailure(context, "arm goal unknown", TaskStatus::Failure, now);
    return;
  }
}

void CommandSequence::handleGripper(const CommandStep &step,
                                    RuntimeContext &context) {
  if (!context.gripper_node) {
    is_finished_ = true;
    result_status_ = TaskStatus::Failure;
    result_message_ = "gripper null";
    return;
  }

  context.gripper_node->setGripperPosition(step.gripper_cmd.command);
  advanceToNextStep();
}

void CommandSequence::handleDelay(const CommandStep &step,
                                  const rclcpp::Time &now) {
  const double elapsed_ms = (now - current_step_start_time_).seconds() * 1000.0;
  if (elapsed_ms >= static_cast<double>(step.delay_ms)) {
    advanceToNextStep();
  }
}

void CommandSequence::handleGuard(const CommandStep &step,
                                  RuntimeContext &context) {
  if (guard_evaluator_(step.guard_id, context)) {
    advanceToNextStep();
    return;
  }

  is_finished_ = true;
  result_status_ = TaskStatus::Failure;
  result_message_ = "guard rejected";
}

void CommandSequence::handleServo(const CommandStep &step,
                                  RuntimeContext &context, bool start,
                                  const rclcpp::Time &now) {
  if (!context.servo_client) {
    handleStepFailure(context, "servo_client null", TaskStatus::Failure, now);
    return;
  }

  const ServoCommandResult result =
      start ? context.servo_client->startServo()
            : context.servo_client->pauseServo();

  switch (result.status) {
  case ServoCommandStatus::Started:
  case ServoCommandStatus::InProgress:
    return;
  case ServoCommandStatus::Success:
    advanceToNextStep();
    return;
  case ServoCommandStatus::Timeout:
    handleStepFailure(context,
                      result.message.empty() ? step.label + " timeout"
                                             : result.message,
                      TaskStatus::Timeout, now);
    return;
  case ServoCommandStatus::Failure:
  default:
    handleStepFailure(context,
                      result.message.empty() ? step.label + " failed"
                                             : result.message,
                      TaskStatus::Failure, now);
    return;
  }
}

} // namespace top_hfsm_v2

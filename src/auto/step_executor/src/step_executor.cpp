#include "step_executor/step_executor.hpp"

#include "step_executor/bridges/composite_capability_bridge.hpp"

namespace step_executor {
using namespace task_step_library;

BridgeResult NoopCapabilityBridge::runStep(const Step &step, StepResult *out_result) {
  if (out_result) {
    *out_result = StepResult{};
  }
  if (step.type == StepType::Guard && step.guard_id < 0) {
    return BridgeResult::Failed;
  }
  return BridgeResult::Succeeded;
}

std::shared_ptr<ICapabilityBridge> createDefaultCapabilityBridge(rclcpp::Node &node) {
  return std::make_shared<CompositeCapabilityBridge>(node);
}

StepExecutor::StepExecutor(rclcpp::Logger logger,
                           std::shared_ptr<ICapabilityBridge> bridge)
    : logger_(logger), bridge_(std::move(bridge)) {}

void StepExecutor::start(const TaskPlan &plan) {
  plan_ = plan;
  step_index_ = 0;
  step_entered_ = false;
  retries_left_ = 0;
  step_start_time_ = rclcpp::Time(0, 0, RCL_STEADY_TIME);
  running_ = true;
  finished_ = false;
  ctx_ = RuntimeContext{};
  report_.status = TaskStatus::Running;
  report_.message.clear();
}

void StepExecutor::tick(const rclcpp::Time &now) {
  if (!running_ || finished_) {
    return;
  }

  if (step_index_ >= plan_.steps.size()) {
    running_ = false;
    finished_ = true;
    report_.status = TaskStatus::Success;
    report_.message = "all steps finished";
    return;
  }

  const Step &step = plan_.steps[step_index_];
  Step derived_step;
  std::string derive_error;
  if (!deriveStepFromRuntimeCtx(step, derived_step, derive_error)) {
    fail(TaskStatus::Failure, "step derive failed: " + step.label + " err=" + derive_error);
    return;
  }

  if (!step_entered_) {
    step_entered_ = true;
    retries_left_ = step.max_retries;
    step_start_time_ = now;
    RCLCPP_INFO(logger_, "[STEP_EXECUTOR] enter step=%zu/%zu label=%s",
                step_index_ + 1, plan_.steps.size(), step.label.c_str());
  }

  if (step.timeout_ms > 0) {
    const double elapsed_ms = (now - step_start_time_).seconds() * 1000.0;
    if (elapsed_ms >= static_cast<double>(step.timeout_ms)) {
      fail(TaskStatus::Timeout, "step timeout: " + step.label);
      return;
    }
  }

  if (derived_step.type == StepType::Delay) {
    const double elapsed_ms = (now - step_start_time_).seconds() * 1000.0;
    if (elapsed_ms >= static_cast<double>(derived_step.delay_ms)) {
      enterNextStep();
    }
    return;
  }

  StepResult step_result{};
  const BridgeResult result =
      bridge_ ? bridge_->runStep(derived_step, &step_result) : BridgeResult::Failed;
  if (result == BridgeResult::Succeeded) {
    updateRuntimeCtx(step_result);
    enterNextStep();
    return;
  }

  if (result == BridgeResult::Running) {
    return;
  }

  if (retries_left_ > 0) {
    --retries_left_;
    step_start_time_ = now;
    RCLCPP_WARN(logger_, "[STEP_EXECUTOR] retry step=%s left=%d", step.label.c_str(), retries_left_);
    return;
  }

  const std::string error = bridge_ ? bridge_->lastError() : "bridge missing";
  fail(TaskStatus::Failure,
       error.empty() ? ("step failed: " + step.label)
                     : ("step failed: " + step.label + " err=" + error));
}

void StepExecutor::cancel() {
  if (!running_ || finished_) {
    return;
  }
  if (bridge_) {
    bridge_->cancel();
  }
  running_ = false;
  finished_ = true;
  report_.status = TaskStatus::Canceled;
  report_.message = "canceled";
}

void StepExecutor::reset() {
  plan_ = TaskPlan{};
  step_index_ = 0;
  step_entered_ = false;
  retries_left_ = 0;
  step_start_time_ = rclcpp::Time(0, 0, RCL_STEADY_TIME);
  running_ = false;
  finished_ = false;
  ctx_ = RuntimeContext{};
  report_.status = TaskStatus::Running;
  report_.message.clear();
}

bool StepExecutor::isRunning() const { return running_; }

bool StepExecutor::isFinished() const { return finished_; }

TaskResult StepExecutor::report() const { return report_; }

TaskId StepExecutor::activeTaskId() const { return plan_.task_id; }

bool StepExecutor::deriveStepFromRuntimeCtx(const Step &input, Step &resolved,
                                            std::string &error) const {
  resolved = input;
  error.clear();

  if (input.type == StepType::Slot) {
    const auto strategy = input.slot.strategy;
    const bool needs_slot_id = strategy == SlotStrategy::LockSlot ||
                               strategy == SlotStrategy::UnlockSlot;
    if (!needs_slot_id) {
      return true;
    }

    if (input.slot.slot_id >= 0) {
      return true;
    }

    if (!ctx_.has_selected_slot) {
      error = "selected slot missing for lock/unlock";
      return false;
    }

    resolved.slot.slot_id = ctx_.selected_slot;
    return true;
  }

  if (input.type != StepType::ArmMove) {
    return true;
  }

  switch (input.arm_move.target_source) {
  case TargetSource::Fixed:
    return true;
  case TargetSource::VisionPose:
    if (!ctx_.has_vision_pose) {
      error = "vision pose missing";
      return false;
    }
    resolved.arm_move.pose = ctx_.vision_pose;
    return true;
  case TargetSource::VisionVector:
    if (!ctx_.has_vision_vector) {
      error = "vision vector missing";
      return false;
    }
    resolved.arm_move.vector = ctx_.vision_vector;
    return true;
  case TargetSource::SlotMapped:
    if (!ctx_.has_selected_slot) {
      error = "selected slot missing";
      return false;
    }
    if (ctx_.selected_slot == 0) {
      resolved.arm_move.plan_option = PlanOption::JOINTS;
      resolved.arm_move.joints = std::array<float, 6>{{0.35f, -1.05f, -2.35f, 0.02f, 0.f, 0.f}};
      return true;
    }
    if (ctx_.selected_slot == 1) {
      resolved.arm_move.plan_option = PlanOption::JOINTS;
      resolved.arm_move.joints =
          std::array<float, 6>{{-0.35f, -1.05f, -2.35f, -0.02f, 0.f, 0.f}};
      return true;
    }
    error = "invalid selected slot";
    return false;
  default:
    error = "unsupported arm target source";
    return false;
  }
}

void StepExecutor::updateRuntimeCtx(const StepResult &result) {
  if (result.has_vision_pose) {
    ctx_.has_vision_pose = true;
    ctx_.vision_pose = result.vision_pose;
  }
  if (result.has_vision_vector) {
    ctx_.has_vision_vector = true;
    ctx_.vision_vector = result.vision_vector;
  }
  if (result.has_selected_slot) {
    ctx_.has_selected_slot = true;
    ctx_.selected_slot = result.selected_slot;
  }
}

void StepExecutor::fail(TaskStatus status, const std::string &message) {
  running_ = false;
  finished_ = true;
  report_.status = status;
  report_.message = message;
  RCLCPP_ERROR(logger_, "[STEP_EXECUTOR] %s", message.c_str());
}

void StepExecutor::enterNextStep() {
  ++step_index_;
  step_entered_ = false;
}

} // namespace step_executor

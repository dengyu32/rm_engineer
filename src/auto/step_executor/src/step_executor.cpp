#include "step_executor/step_executor.hpp"

#include "step_executor/bridges/composite_capability_bridge.hpp"

namespace step_executor {
using namespace task_step_library;

BridgeResult NoopCapabilityBridge::runStep(const Step &step) {
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

  if (step.type == StepType::Delay) {
    const double elapsed_ms = (now - step_start_time_).seconds() * 1000.0;
    if (elapsed_ms >= static_cast<double>(step.delay_ms)) {
      enterNextStep();
    }
    return;
  }

  const BridgeResult result = bridge_ ? bridge_->runStep(step) : BridgeResult::Failed;
  if (result == BridgeResult::Succeeded) {
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
  report_.status = TaskStatus::Running;
  report_.message.clear();
}

bool StepExecutor::isRunning() const { return running_; }

bool StepExecutor::isFinished() const { return finished_; }

TaskResult StepExecutor::report() const { return report_; }

TaskId StepExecutor::activeTaskId() const { return plan_.task_id; }

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

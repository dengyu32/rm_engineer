#include "step_executor/step_executor.hpp"

#include <array>
#include <vector>

namespace step_executor {

using core::Binding;
using core::BindingOp;
using core::Command;
using core::ContextKey;
using core::ContextScope;
using core::ExecuteResult;
using core::ExecuteStatus;
using core::Step;
using core::TaskPlan;
using core::TaskResult;
using core::TaskId;
using core::TaskStatus;
using core::Value;
using core::valueAs;

namespace {

const char *scopeName(ContextScope scope) {
  return scope == ContextScope::Persist ? "persist" : "task";
}

std::string formatKeyList(const std::vector<ContextKey> &keys) {
  std::string out;
  for (std::size_t i = 0; i < keys.size(); ++i) {
    if (i > 0) {
      out += ",";
    }
    out += keys[i].name;
    out += "@";
    out += scopeName(keys[i].scope);
  }
  return out;
}

bool hasOutputName(const Step &step, const std::string &name) {
  for (const auto &key : step.outputs) {
    if (key.name == name) {
      return true;
    }
  }
  return false;
}

} // namespace

// 传入 capability bridge 指针
StepExecutor::StepExecutor(rclcpp::Logger logger,
                           std::shared_ptr<ICapabilityBridge> bridge)
    : logger_(logger), bridge_(std::move(bridge)) {}

void StepExecutor::start(const TaskPlan &plan) {
  plan_ = plan;
  step_index_ = 0;
  step_entered_ = false;
  retries_left_ = 0;
  step_start_time_ = rclcpp::Time(0, 0, RCL_STEADY_TIME);
  post_delay_active_ = false;
  post_delay_start_time_ = rclcpp::Time(0, 0, RCL_STEADY_TIME);
  running_ = true;
  finished_ = false;
  context_.clearTask();
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
    const std::string label = step.label.empty() ? step.id : step.label;
    RCLCPP_INFO(logger_, "[STEP_EXECUTOR] enter step=%zu/%zu id=%s label=%s",
                step_index_ + 1,
                plan_.steps.size(),
                step.id.c_str(),
                label.c_str());
    if (!step.inputs.empty() || !step.outputs.empty()) {
      const std::string inputs = formatKeyList(step.inputs);
      const std::string outputs = formatKeyList(step.outputs);
      RCLCPP_INFO(logger_, "[STEP_EXECUTOR] io inputs=[%s] outputs=[%s]",
                  inputs.c_str(), outputs.c_str());
    }
  }

  if (post_delay_active_) {
    if (step.post_delay_ms < 0) {
      fail(TaskStatus::Failure, "post delay invalid duration_ms: " + step.id);
      return;
    }
    const double elapsed_ms = (now - post_delay_start_time_).seconds() * 1000.0;
    if (elapsed_ms >= static_cast<double>(step.post_delay_ms)) {
      post_delay_active_ = false;
      enterNextStep();
    }
    return;
  }

  if (step.timeout_ms > 0) {
    const double elapsed_ms = (now - step_start_time_).seconds() * 1000.0;
    if (elapsed_ms >= static_cast<double>(step.timeout_ms)) {
      if (retries_left_ > 0) {
        --retries_left_;
        if (bridge_) {
          bridge_->cancel();
        }
        step_start_time_ = now;
        RCLCPP_WARN(logger_, "[STEP_EXECUTOR] timeout retry step=%s left=%d",
                    step.id.c_str(), retries_left_);
        return;
      }
      fail(TaskStatus::Timeout, "step timeout: " + step.id);
      return;
    }
  }

  if (!step.inputs.empty()) {
    for (const auto &key : step.inputs) {
      if (!context_.has(key)) {
        fail(TaskStatus::Failure,
             "step missing input: " + step.id + " key=" + key.name);
        return;
      }
    }
  }

  Command exec_cmd = step.command;
  std::string error;
  if (!applyBindings(step, exec_cmd, error)) {
    fail(TaskStatus::Failure, "step binding failed: " + step.id + " err=" + error);
    return;
  }

  ExecuteResult result{};
  if (bridge_) {
    result = bridge_->run(exec_cmd);
  } else {
    result.status = ExecuteStatus::Failed;
    result.error.message = "bridge missing";
    result.error.retriable = false;
  }

  if (result.status == ExecuteStatus::Running) {
    return;
  }

  if (result.status == ExecuteStatus::Succeeded) {
    if (!applyOutputs(step, result, error)) {
      fail(TaskStatus::Failure, "step output failed: " + step.id + " err=" + error);
      return;
    }
    if (step.post_delay_ms > 0) {
      post_delay_active_ = true;
      post_delay_start_time_ = now;
      return;
    }
    enterNextStep();
    return;
  }

  const bool can_retry = result.error.retriable && retries_left_ > 0;
  if (can_retry) {
    --retries_left_;
    step_start_time_ = now;
    RCLCPP_WARN(logger_, "[STEP_EXECUTOR] retry step=%s left=%d",
                step.id.c_str(), retries_left_);
    return;
  }

  std::string message = result.error.message;
  if (message.empty() && bridge_) {
    const char *bridge_error = bridge_->lastError();
    if (bridge_error) {
      message = bridge_error;
    }
  }
  if (message.empty()) {
    message = "step failed";
  }
  fail(TaskStatus::Failure, "step failed: " + step.id + " err=" + message);
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
  post_delay_active_ = false;
  post_delay_start_time_ = rclcpp::Time(0, 0, RCL_STEADY_TIME);
  running_ = false;
  finished_ = false;
  context_.clearTask();
  report_.status = TaskStatus::Running;
  report_.message.clear();
}

bool StepExecutor::isRunning() const { return running_; }

bool StepExecutor::isFinished() const { return finished_; }

TaskResult StepExecutor::report() const { return report_; }

TaskId StepExecutor::activeTaskId() const { return plan_.task_id; }

std::size_t StepExecutor::currentStepIndex() const { return step_index_; }

std::size_t StepExecutor::totalSteps() const { return plan_.steps.size(); }

std::string StepExecutor::currentStepLabel() const {
  if (step_index_ >= plan_.steps.size()) {
    return std::string();
  }
  const Step &step = plan_.steps[step_index_];
  if (!step.label.empty()) {
    return step.label;
  }
  return step.id;
}

bool StepExecutor::applyBindings(const Step &step, Command &cmd, std::string &error) const {
  for (const auto &binding : step.bindings) {
    Value value;
    if (!context_.get(binding.from, value)) {
      error = "missing binding source: " + binding.from.name;
      return false;
    }
    switch (binding.op) {
      case BindingOp::Direct:
        cmd.params[binding.to_param] = value;
        break;
      case BindingOp::IndexToJointsTable: {
        if (!binding.joints_table || binding.joints_table_size == 0) {
          error = "binding table missing: " + binding.from.name;
          return false;
        }
        const int64_t *slot_id = valueAs<int64_t>(value);
        if (!slot_id) {
          error = "binding type mismatch (expected int64): " + binding.from.name;
          return false;
        }
        if (*slot_id < 0 ||
            static_cast<size_t>(*slot_id) >= binding.joints_table_size) {
          error = "binding index out of range: " + binding.from.name;
          return false;
        }
        const auto &row = binding.joints_table[*slot_id];
        cmd.params[binding.to_param] = std::vector<float>(row.begin(), row.end());
        break;
      }
      default:
        error = "unknown binding op: " + binding.from.name;
        return false;
    }
  }
  error.clear();
  return true;
}

bool StepExecutor::applyOutputs(const Step &step, const ExecuteResult &result,
                                std::string &error) {
  for (const auto &key : step.outputs) {
    auto it = result.outputs.find(key.name);
    if (it == result.outputs.end()) {
      error = "output missing: " + key.name;
      return false;
    }
    if (!context_.set(key, it->second)) {
      error = "output type mismatch: " + key.name;
      return false;
    }
  }

  for (const auto &entry : result.outputs) {
    if (!hasOutputName(step, entry.first)) {
      RCLCPP_WARN(logger_, "[STEP_EXECUTOR] output not declared: %s", entry.first.c_str());
    }
  }

  error.clear();
  return true;
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
  post_delay_active_ = false;
}

} // namespace step_executor

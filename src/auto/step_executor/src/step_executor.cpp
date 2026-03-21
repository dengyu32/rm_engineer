#include "step_executor/step_executor.hpp"

#include <vector>

#include "step_executor/bridges/arm_capability_bridge.hpp"
#include "step_executor/bridges/gripper_capability_bridge.hpp"
#include "step_executor/bridges/registry_capability_bridge.hpp"
#include "step_executor/bridges/slot_capability_bridge.hpp"
#include "step_executor/bridges/vision_capability_bridge.hpp"

namespace step_executor {
using namespace task_step_library;

namespace {

const char *sharedKeyName(SharedKey key) {
  switch (key) {
  case SharedKey::VisionPose:
    return "VisionPose";
  case SharedKey::VisionVector:
    return "VisionVector";
  case SharedKey::SelectedSlot:
    return "SelectedSlot";
  default:
    return "Unknown";
  }
}

std::string formatKeyList(const std::vector<SharedKey> &keys) {
  std::string out;
  for (std::size_t i = 0; i < keys.size(); ++i) {
    if (i > 0) {
      out += ",";
    }
    out += sharedKeyName(keys[i]);
  }
  return out;
}

} // namespace

BridgeResult NoopCapabilityBridge::runStep(const Step &step, StepResult *out_result) {
  if (out_result) {
    *out_result = StepResult{};
  }
  if (step.type == StepType::Guard && step.guard_id < 0) {
    return BridgeResult::Failed;
  }
  return BridgeResult::Succeeded;
}

std::shared_ptr<RegistryCapabilityBridge> createDefaultRegistryBridge(rclcpp::Node &node) {
  auto registry = std::make_shared<RegistryCapabilityBridge>();

  auto arm_bridge = std::make_shared<ArmCapabilityBridge>(node);
  registry->registerHandler(
      StepType::ArmMove,
      [arm_bridge](const Step &step, StepResult *out_result) {
        return arm_bridge->runArmStep(step, out_result);
      },
      [arm_bridge]() { arm_bridge->cancel(); },
      [arm_bridge]() { return arm_bridge->lastError(); });

  auto gripper_bridge = std::make_shared<GripperCapabilityBridge>(node);
  registry->registerHandler(
      StepType::Gripper,
      [gripper_bridge](const Step &step, StepResult *out_result) {
        return gripper_bridge->runGripperStep(step, out_result);
      },
      [gripper_bridge]() { gripper_bridge->cancel(); },
      [gripper_bridge]() { return gripper_bridge->lastError(); });

  auto slot_bridge = std::make_shared<SlotCapabilityBridge>(node);
  registry->registerHandler(
      StepType::Slot,
      [slot_bridge](const Step &step, StepResult *out_result) {
        return slot_bridge->runSlotStep(step, out_result);
      },
      [slot_bridge]() { slot_bridge->cancel(); },
      [slot_bridge]() { return slot_bridge->lastError(); });

  auto vision_bridge = std::make_shared<VisionCapabilityBridge>(node);
  registry->registerHandler(
      StepType::Vision,
      [vision_bridge](const Step &step, StepResult *out_result) {
        return vision_bridge->runVisionStep(step, out_result);
      },
      [vision_bridge]() { vision_bridge->cancel(); },
      [vision_bridge]() { return vision_bridge->lastError(); });

  registry->registerHandler(
      StepType::Guard,
      [](const Step &step, StepResult *out_result) {
        if (out_result) {
          *out_result = StepResult{};
        }
        return step.guard_id < 0 ? BridgeResult::Failed : BridgeResult::Succeeded;
      },
      []() {},
      []() { return "guard rejected"; });

  return registry;
}

std::shared_ptr<ICapabilityBridge> createDefaultCapabilityBridge(rclcpp::Node &node) {
  return createDefaultRegistryBridge(node);
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
  data_.clear();
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
    if (!step.inputs.empty() || !step.outputs.empty()) {
      const std::string inputs = formatKeyList(step.inputs);
      const std::string outputs = formatKeyList(step.outputs);
      RCLCPP_INFO(logger_, "[STEP_EXECUTOR] io inputs=[%s] outputs=[%s]",
                  inputs.c_str(), outputs.c_str());
    }
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
                    step.label.c_str(), retries_left_);
        return;
      }
      fail(TaskStatus::Timeout, "step timeout: " + step.label);
      return;
    }
  }

  if (!step.inputs.empty()) {
    for (const auto key : step.inputs) {
      if (!data_.has(key)) {
        fail(TaskStatus::Failure,
             "step missing input: " + step.label + " key=" + sharedKeyName(key));
        return;
      }
    }
  }

  Step derived_step;
  std::string derive_error;
  if (!deriveStepFromSharedData(step, derived_step, derive_error)) {
    fail(TaskStatus::Failure, "step derive failed: " + step.label + " err=" + derive_error);
    return;
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
    applyStepResult(step_result);
    if (!step.outputs.empty()) {
      for (const auto key : step.outputs) {
        if (!data_.has(key)) {
          fail(TaskStatus::Failure,
               "step output missing: " + step.label + " key=" + sharedKeyName(key));
          return;
        }
      }
    }
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
  data_.clear();
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
  return plan_.steps[step_index_].label;
}

bool StepExecutor::deriveStepFromSharedData(const Step &input, Step &resolved,
                                            std::string &error) const {
  resolved = input;
  error.clear();

  if (input.type == StepType::Slot) {
    const auto strategy = input.slot.strategy;
    const bool needs_slot_id = (strategy == SlotStrategy::LockSlot) ||
                               (strategy == SlotStrategy::UnlockSlot);
    if (!needs_slot_id) {
      return true;
    }

    if (input.slot.slot_id >= 0) {
      return true;
    }

    int slot = -1;
    if (!data_.get<SharedKey::SelectedSlot>(slot)) {
      error = "selected slot missing for lock/unlock";
      return false;
    }
    resolved.slot.slot_id = slot;
    return true;
  }

  if (input.type != StepType::ArmMove) {
    return true;
  }

  return resolver_.resolve(input.arm_move, data_, resolved.arm_move, error);
}

void StepExecutor::applyStepResult(const StepResult &result) {
  for (const auto &update : result.updates) {
    if (!data_.set(update.key, update.value)) {
      RCLCPP_WARN(logger_, "[STEP_EXECUTOR] shared data type mismatch key=%d",
                  static_cast<int>(update.key));
    }
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

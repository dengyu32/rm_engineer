#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "task_step_library/context.hpp"
#include "task_step_library/step.hpp"

namespace step_executor {

enum class BridgeResult {
  Running = 0,
  Succeeded = 1,
  Failed = 2,
};

class ICapabilityBridge {
public:
  virtual ~ICapabilityBridge() = default;

  virtual BridgeResult runStep(const task_step_library::Step &step,
                               task_step_library::StepResult *out_result) = 0;
  virtual void cancel() = 0;
  virtual const char *lastError() const = 0;
};

class NoopCapabilityBridge : public ICapabilityBridge {
public:
  BridgeResult runStep(const task_step_library::Step &step,
                       task_step_library::StepResult *out_result) override;
  void cancel() override {}
  const char *lastError() const override { return ""; }
};

std::shared_ptr<ICapabilityBridge> createDefaultCapabilityBridge(rclcpp::Node &node);

} // namespace step_executor

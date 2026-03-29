#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "step_executor/types/command.hpp"
#include "step_executor/types/execute_result.hpp"

namespace step_executor {

class ICapabilityBridge {
public:
  virtual ~ICapabilityBridge() = default;

  virtual ExecuteResult run(const Command &cmd) = 0;
  virtual void cancel() = 0;
  virtual const char *lastError() const = 0;
};

class NoopCapabilityBridge : public ICapabilityBridge {
public:
  ExecuteResult run(const Command &cmd) override;
  void cancel() override {}
  const char *lastError() const override { return ""; }
};

class RegistryBridge;

} // namespace step_executor

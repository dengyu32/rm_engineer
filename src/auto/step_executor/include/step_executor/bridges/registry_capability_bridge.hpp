#pragma once

#include <functional>
#include <string>
#include <unordered_map>

#include "step_executor/capability_bridge.hpp"

namespace step_executor {

class RegistryCapabilityBridge : public ICapabilityBridge {
public:
  using HandlerFn =
      std::function<BridgeResult(const task_step_library::Step &,
                                 task_step_library::StepResult *)>;
  using CancelFn = std::function<void()>;
  using ErrorFn = std::function<const char *()>;

  bool registerHandler(task_step_library::StepType type,
                       HandlerFn run,
                       CancelFn cancel = {},
                       ErrorFn error = {});

  BridgeResult runStep(const task_step_library::Step &step,
                       task_step_library::StepResult *out_result) override;
  void cancel() override;
  const char *lastError() const override { return last_error_.c_str(); }

private:
  struct Handler {
    HandlerFn run;
    CancelFn cancel;
    ErrorFn error;
  };

  struct StepTypeHash {
    std::size_t operator()(task_step_library::StepType type) const noexcept {
      return static_cast<std::size_t>(type);
    }
  };

  std::unordered_map<task_step_library::StepType, Handler, StepTypeHash> handlers_;
  std::string last_error_;
};

} // namespace step_executor

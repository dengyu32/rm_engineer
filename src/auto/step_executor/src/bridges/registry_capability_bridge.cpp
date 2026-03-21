#include "step_executor/bridges/registry_capability_bridge.hpp"

#include <utility>

namespace step_executor {

bool RegistryCapabilityBridge::registerHandler(task_step_library::StepType type,
                                               HandlerFn run,
                                               CancelFn cancel,
                                               ErrorFn error) {
  if (!run) {
    return false;
  }
  auto it = handlers_.find(type);
  if (it != handlers_.end()) {
    return false;
  }
  handlers_.emplace(type, Handler{std::move(run), std::move(cancel), std::move(error)});
  return true;
}

BridgeResult RegistryCapabilityBridge::runStep(const task_step_library::Step &step,
                                               task_step_library::StepResult *out_result) {
  if (out_result) {
    *out_result = task_step_library::StepResult{};
  }
  const auto it = handlers_.find(step.type);
  if (it == handlers_.end()) {
    last_error_ = "unsupported step type";
    return BridgeResult::Failed;
  }
  const BridgeResult result = it->second.run(step, out_result);
  if (result == BridgeResult::Failed) {
    if (it->second.error) {
      const char *err = it->second.error();
      last_error_ = err ? err : "";
    }
    if (last_error_.empty()) {
      last_error_ = "step failed";
    }
  } else if (result == BridgeResult::Succeeded) {
    last_error_.clear();
  }
  return result;
}

void RegistryCapabilityBridge::cancel() {
  for (auto &entry : handlers_) {
    if (entry.second.cancel) {
      entry.second.cancel();
    }
  }
}

} // namespace step_executor

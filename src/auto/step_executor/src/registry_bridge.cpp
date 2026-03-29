#include "step_executor/registry_bridge.hpp"

#include <utility>

namespace step_executor {

bool RegistryBridge::registerHandler(const std::string &kind,
                                     HandlerFn run,
                                     CancelFn cancel,
                                     ErrorFn error) {
  if (!run || kind.empty()) {
    return false;
  }
  auto it = handlers_.find(kind);
  if (it != handlers_.end()) {
    return false;
  }
  handlers_.emplace(kind, Handler{std::move(run), std::move(cancel), std::move(error)});
  return true;
}

ExecuteResult RegistryBridge::run(const Command &cmd) {
  const auto it = handlers_.find(cmd.kind);
  if (it == handlers_.end()) {
    last_error_ = "unsupported command kind";
    ExecuteResult result{};
    result.status = ExecuteStatus::Failed;
    result.error.code = ErrorCode::ValidationError;
    result.error.message = last_error_;
    result.error.retriable = false;
    return result;
  }

  ExecuteResult result = it->second.run(cmd);
  if (result.status == ExecuteStatus::Failed) {
    if (result.error.message.empty() && it->second.error) {
      const char *err = it->second.error();
      if (err) {
        result.error.message = err;
      }
    }
    if (result.error.message.empty()) {
      result.error.message = "command failed";
    }
    last_error_ = result.error.message;
  } else if (result.status == ExecuteStatus::Succeeded) {
    last_error_.clear();
  }
  return result;
}

void RegistryBridge::cancel() {
  for (auto &entry : handlers_) {
    if (entry.second.cancel) {
      entry.second.cancel();
    }
  }
}

} // namespace step_executor

#pragma once

#include <functional>
#include <string>
#include <unordered_map>

#include "step_executor/types/capability_bridge.hpp"

namespace step_executor {

class RegistryBridge : public ICapabilityBridge {
public:
  using HandlerFn = std::function<ExecuteResult(const Command &)>;
  using CancelFn = std::function<void()>;
  using ErrorFn = std::function<const char *()>;

  bool registerHandler(const std::string &kind,
                       HandlerFn run,
                       CancelFn cancel = {},
                       ErrorFn error = {});

  ExecuteResult run(const Command &cmd) override;
  void cancel() override;
  const char *lastError() const override { return last_error_.c_str(); }

private:
  struct Handler {
    HandlerFn run;
    CancelFn cancel;
    ErrorFn error;
  };

  std::unordered_map<std::string, Handler> handlers_{};
  std::string last_error_;
};

} // namespace step_executor

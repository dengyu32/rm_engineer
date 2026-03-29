#pragma once

#include <functional>
#include <string>
#include <unordered_map>

#include "step_executor/types/command.hpp"
#include "step_executor/types/execute_result.hpp"

namespace step_executor {

// ============================================================================
//  ICapabilityBridge
// ----------------------------------------------------------------------------
//  - capability 执行接口
//  - RegistryBridge / NoopBridge 的统一基类
// ============================================================================

class ICapabilityBridge {
public:
  virtual ~ICapabilityBridge() = default;

  virtual ExecuteResult run(const Command &cmd) = 0;
  virtual void cancel() = 0;
  virtual const char *lastError() const = 0;
};

// ============================================================================
//  RegistryBridge
// ----------------------------------------------------------------------------
//  - kind -> capability handler 路由
//  - 注册/调用/取消统一入口
// ============================================================================

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

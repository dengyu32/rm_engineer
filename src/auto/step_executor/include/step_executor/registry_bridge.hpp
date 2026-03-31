// registry_bridge.hpp 
// 作为 capability handler 的注册中心, 提供复用注册能力方法

// 注册方法


#pragma once

#include <functional>
#include <string>
#include <unordered_map>

// step layer
#include "auto_library/command.hpp"
#include "auto_library/execute_result.hpp"

namespace step_executor {

// ============================================================================
//  ICapabilityBridge
// ----------------------------------------------------------------------------
//  - capability 执行接口
//  - RegistryBridge 的统一基类
// ============================================================================

class ICapabilityBridge {
public:
  virtual ~ICapabilityBridge() = default;

  virtual core::ExecuteResult run(const core::Command &cmd) = 0;
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
  // 三个注册函数指针
  using HandlerFn = std::function<core::ExecuteResult(const core::Command &)>;
  using CancelFn = std::function<void()>;
  using ErrorFn = std::function<const char *()>;

  bool registerHandler(const std::string &kind,
                       HandlerFn run,
                       CancelFn cancel = {},
                       ErrorFn error = {});

  core::ExecuteResult run(const core::Command &cmd) override;
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

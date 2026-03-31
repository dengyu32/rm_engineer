#include "step_executor/registry_bridge.hpp"

// TO OPTIMIZE:
/*
  1. 目前 cancel 全局取消
  2. last_error_ 需要取消
*/

#include <utility>

namespace step_executor {

// 主要注册函数
bool RegistryBridge::registerHandler(const std::string &kind,
                                     HandlerFn run,
                                     CancelFn cancel,
                                     ErrorFn error) {
  if (!run || kind.empty()) {
    return false;
  }
  // 防止重复注册
  auto it = handlers_.find(kind);
  if (it != handlers_.end()) {
    return false;
  }
  // 就地构造 unordered_map
  // key: 字符串 kind 
  // value: Handler 结构体, 具体能力方法
  // emplace 翻译为 安置,安放
  handlers_.emplace(kind, Handler{std::move(run), std::move(cancel), std::move(error)});
  return true;
}

// run 主要运行函数
core::ExecuteResult RegistryBridge::run(const core::Command &cmd) {
  const auto it = handlers_.find(cmd.kind);
  // 没有注册这个 kind 的 handler
  if (it == handlers_.end()) {
    last_error_ = "Capability Not Registered";
    core::ExecuteResult result{};
    result.status = core::ExecuteStatus::Failed;
    result.error.message = last_error_;
    result.error.retriable = false;
    return result;
  }
  // 调用注册的 handler run 函数
  core::ExecuteResult result = it->second.run(cmd);
  if (result.status == core::ExecuteStatus::Failed) {
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
  } else if (result.status == core::ExecuteStatus::Succeeded) {
    last_error_.clear();
  }
  return result;
}

// 目前 cancel 是全局取消 之后有并行能力时需要改进
void RegistryBridge::cancel() {
  for (auto &entry : handlers_) {
    if (entry.second.cancel) {
      entry.second.cancel();
    }
  }
}

} // namespace step_executor

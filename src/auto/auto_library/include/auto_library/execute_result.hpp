#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>

#include "auto_library/value.hpp"

namespace core {

// ============================================================================
//  ExecuteResult
// ----------------------------------------------------------------------------
//  - capability 执行结果
//  - Running / Succeeded / Failed
//  - Failed 时携带 ErrorInfo
// ============================================================================

enum class ExecuteStatus : uint8_t {
  Running = 0,
  Succeeded = 1,
  Failed = 2,
};

struct ErrorInfo {
  std::string message{};
  bool retriable{false};
  std::string detail{};
};

struct ExecuteResult {
  ExecuteStatus status{ExecuteStatus::Succeeded};
  std::unordered_map<std::string, Value> outputs{};
  ErrorInfo error{};
};

// 辅助函数 构造不同类型的 ExecuteResult
inline ExecuteResult makeSucceeded() {
  ExecuteResult result{};
  result.status = ExecuteStatus::Succeeded;
  return result;
}

inline ExecuteResult makeRunning() {
  ExecuteResult result{};
  result.status = ExecuteStatus::Running;
  return result;
}

inline ExecuteResult makeFailed(std::string message, bool retriable = false,
                                std::string detail = {}) {
  ExecuteResult result{};
  result.status = ExecuteStatus::Failed;
  result.error.message = std::move(message);
  result.error.retriable = retriable;
  result.error.detail = std::move(detail);
  return result;
}

} // namespace core

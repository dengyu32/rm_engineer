#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>

#include "auto_library/value.hpp"

namespace step_executor {

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

} // namespace step_executor

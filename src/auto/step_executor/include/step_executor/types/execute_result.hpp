#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>

#include "step_executor/types/value.hpp"

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

enum class ErrorCode : uint8_t {
  ValidationError = 0,
  NotReady = 1,
  TransportError = 2,
  ExecutionError = 3,
  Timeout = 4,
  Canceled = 5,
  Unknown = 6,
};

struct ErrorInfo {
  ErrorCode code{ErrorCode::Unknown};
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

#pragma once

#include <cstdint>
#include <functional>
#include <map>
#include <string>

namespace error_code_utils {

// -----------------------------------------------------------------------------
// Basic Types
// -----------------------------------------------------------------------------

enum class Severity {
  Warning = 0,
  Recoverable = 1,
  Fatal = 2,
};

enum class RecommendedAction {
  None = 0,
  Retry = 1,
  Reset = 2,
  Degrade = 3,
  Abort = 4,
};

struct ErrorCode {
  uint16_t domain{0};
  uint16_t code{0};

  uint32_t value() const {
    return (static_cast<uint32_t>(domain) << 16) | static_cast<uint32_t>(code);
  }

  bool operator==(const ErrorCode& other) const {
    return domain == other.domain && code == other.code;
  }

  bool operator!=(const ErrorCode& other) const {
    return !(*this == other);
  }
};

using Context = std::map<std::string, std::string>;

struct ErrorState {
  ErrorCode code{};
  Severity severity{Severity::Warning};
  bool active{false};
  int count{0};
  int64_t first_seen_ms{0};
  int64_t last_seen_ms{0};
  uint64_t instance_id{0};
  RecommendedAction action{RecommendedAction::None};
  Context last_context{};
  int priority{0};
};

}  // namespace error_code_utils

namespace std {

template <>
struct hash<error_code_utils::ErrorCode> {
  size_t operator()(const error_code_utils::ErrorCode& code) const noexcept {
    return static_cast<size_t>(code.value());
  }
};

}  // namespace std

#pragma once

#include <cstdint>
#include <unordered_map>

#include "error_code_utils/types.hpp"

namespace error_code_utils {

// -----------------------------------------------------------------------------
// Policy & Rules
// -----------------------------------------------------------------------------

enum class ClearPolicy {
  AckOnly = 0,
  AutoHealthyMs = 1,
  AckOrHealthyMs = 2,
};

struct Rule {
  int64_t window_ms{0};
  int threshold{0};
  bool latch{true};
  int priority{0};
  ClearPolicy clear_policy{ClearPolicy::AckOrHealthyMs};
  int64_t healthy_clear_ms{0};
  RecommendedAction action{RecommendedAction::None};
};

struct SeverityPolicy {
  Rule warning{};
  Rule recoverable{};
  Rule fatal{};
  std::unordered_map<ErrorCode, Rule> overrides{};

  SeverityPolicy() {
    warning.window_ms = 5000;
    warning.threshold = 10;
    warning.latch = true;
    warning.clear_policy = ClearPolicy::AckOrHealthyMs;
    warning.healthy_clear_ms = 2000;
    warning.priority = 30;
    warning.action = RecommendedAction::None;

    recoverable.window_ms = 2000;
    recoverable.threshold = 3;
    recoverable.latch = true;
    recoverable.clear_policy = ClearPolicy::AckOrHealthyMs;
    recoverable.healthy_clear_ms = 2000;
    recoverable.priority = 60;
    recoverable.action = RecommendedAction::Retry;

    fatal.window_ms = 1000;
    fatal.threshold = 1;
    fatal.latch = true;
    fatal.clear_policy = ClearPolicy::AckOrHealthyMs;
    fatal.healthy_clear_ms = 2000;
    fatal.priority = 100;
    fatal.action = RecommendedAction::Abort;
  }

  Rule rule_for(const ErrorCode& code, Severity sev) const {
    const auto it = overrides.find(code);
    if (it != overrides.end()) {
      return it->second;
    }
    switch (sev) {
      case Severity::Warning:
        return warning;
      case Severity::Recoverable:
        return recoverable;
      case Severity::Fatal:
        return fatal;
    }
    return warning;
  }
};

}  // namespace error_code_utils

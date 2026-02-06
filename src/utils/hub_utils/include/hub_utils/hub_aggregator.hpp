#pragma once

#include <algorithm>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "error_code_utils/types.hpp"

namespace hub_utils {

// -----------------------------------------------------------------------------
// Hub Aggregator Core
// -----------------------------------------------------------------------------

class HubAggregatorCore {
 public:
  void add_snapshot(const std::string& module_name,
                    const std::vector<error_code_utils::ErrorState>& states) {
    snapshots_[module_name] = states;
  }

  std::vector<error_code_utils::ErrorState> global_snapshot() const {
    std::unordered_map<error_code_utils::ErrorCode, error_code_utils::ErrorState> merged;

    for (const auto& kv : snapshots_) {
      for (const auto& state : kv.second) {
        if (!state.active) {
          continue;
        }
        auto it = merged.find(state.code);
        if (it == merged.end()) {
          merged.emplace(state.code, state);
        } else {
          it->second = pick_duplicate(it->second, state);
        }
      }
    }

    std::vector<error_code_utils::ErrorState> out;
    out.reserve(merged.size());
    for (const auto& kv : merged) {
      out.push_back(kv.second);
    }
    return out;
  }

  std::optional<error_code_utils::ErrorState> top_priority_error() const {
    auto snapshot = global_snapshot();
    if (snapshot.empty()) {
      return std::nullopt;
    }

    const auto best_it = std::max_element(
        snapshot.begin(), snapshot.end(),
        [](const error_code_utils::ErrorState& a, const error_code_utils::ErrorState& b) {
          return compare_priority(a, b) < 0;
        });

    return *best_it;
  }

 private:
  static int severity_rank(error_code_utils::Severity s) {
    switch (s) {
      case error_code_utils::Severity::Warning:
        return 0;
      case error_code_utils::Severity::Recoverable:
        return 1;
      case error_code_utils::Severity::Fatal:
        return 2;
    }
    return 0;
  }

  static int compare_priority(const error_code_utils::ErrorState& a,
                              const error_code_utils::ErrorState& b) {
    if (a.priority != b.priority) {
      return a.priority - b.priority;
    }
    const int sev_diff = severity_rank(a.severity) - severity_rank(b.severity);
    if (sev_diff != 0) {
      return sev_diff;
    }
    if (a.last_seen_ms != b.last_seen_ms) {
      return (a.last_seen_ms > b.last_seen_ms) ? 1 : -1;
    }
    return 0;
  }

  static error_code_utils::ErrorState pick_duplicate(const error_code_utils::ErrorState& a,
                                                     const error_code_utils::ErrorState& b) {
    if (a.last_seen_ms != b.last_seen_ms) {
      return (a.last_seen_ms > b.last_seen_ms) ? a : b;
    }
    const int sev_a = severity_rank(a.severity);
    const int sev_b = severity_rank(b.severity);
    if (sev_a != sev_b) {
      return (sev_a > sev_b) ? a : b;
    }
    if (a.priority != b.priority) {
      return (a.priority > b.priority) ? a : b;
    }
    return a;
  }

  std::unordered_map<std::string, std::vector<error_code_utils::ErrorState>> snapshots_{};
};

}  // namespace hub_utils

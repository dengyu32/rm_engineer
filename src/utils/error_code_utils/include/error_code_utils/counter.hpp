#pragma once

#include <cstdint>
#include <deque>

#include "error_code_utils/policy.hpp"
#include "error_code_utils/types.hpp"

namespace error_code_utils {

// -----------------------------------------------------------------------------
// Sliding Window Counter + Latch + ACK (internal utility)
// -----------------------------------------------------------------------------

class ErrorCounter {
 public:
  bool bump(const Rule& rule,
            const ErrorCode& code,
            Severity sev,
            int64_t now_ms,
            uint64_t new_instance_id_if_edge,
            Context ctx = {}) {
    (void)code;
    trim(rule, now_ms);
    timestamps_ms_.push_back(now_ms);
    first_seen_ms_ = timestamps_ms_.empty() ? 0 : timestamps_ms_.front();
    last_seen_ms_ = now_ms;
    severity_ = sev;
    last_context_ = std::move(ctx);

    const int count = static_cast<int>(timestamps_ms_.size());
    if (!active_ && count >= rule.threshold) {
      active_ = true;
      instance_id_ = new_instance_id_if_edge;
      return true;
    }

    if (active_ && !rule.latch && count < rule.threshold) {
      clear_state();
    }
    return false;
  }

  void tick(const Rule& rule, int64_t now_ms) {
    trim(rule, now_ms);

    if (active_ && uses_auto_clear(rule)) {
      if (now_ms - last_seen_ms_ >= rule.healthy_clear_ms) {
        clear_state();
      }
    }
  }

  void ack() {
    clear_state();
  }

  ErrorState to_state(const Rule& rule, const ErrorCode& code) const {
    ErrorState state{};
    state.code = code;
    state.severity = severity_;
    state.active = active_;
    state.count = static_cast<int>(timestamps_ms_.size());
    state.first_seen_ms = first_seen_ms_;
    state.last_seen_ms = last_seen_ms_;
    state.instance_id = instance_id_;
    state.action = rule.action;
    state.last_context = last_context_;
    state.priority = rule.priority;
    return state;
  }

  bool active() const { return active_; }
  uint64_t instance_id() const { return instance_id_; }
  Severity last_severity() const { return severity_; }

 private:
  void trim(const Rule& rule, int64_t now_ms) {
    const int64_t cutoff = now_ms - rule.window_ms;
    while (!timestamps_ms_.empty() && timestamps_ms_.front() < cutoff) {
      timestamps_ms_.pop_front();
    }
    if (!timestamps_ms_.empty()) {
      first_seen_ms_ = timestamps_ms_.front();
    } else if (!active_) {
      first_seen_ms_ = 0;
      last_seen_ms_ = 0;
    }
  }

  bool uses_auto_clear(const Rule& rule) const {
    return rule.clear_policy == ClearPolicy::AutoHealthyMs ||
           rule.clear_policy == ClearPolicy::AckOrHealthyMs;
  }

  void clear_state() {
    active_ = false;
    timestamps_ms_.clear();
    first_seen_ms_ = 0;
    last_seen_ms_ = 0;
    instance_id_ = 0;
    last_context_.clear();
  }

  std::deque<int64_t> timestamps_ms_{};
  bool active_{false};
  int64_t first_seen_ms_{0};
  int64_t last_seen_ms_{0};
  uint64_t instance_id_{0};
  Severity severity_{Severity::Warning};
  Context last_context_{};
};

}  // namespace error_code_utils

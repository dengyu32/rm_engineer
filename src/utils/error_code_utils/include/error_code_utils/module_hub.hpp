#pragma once

#include <cstdint>
#include <unordered_map>
#include <vector>

#include "error_code_utils/clock.hpp"
#include "error_code_utils/counter.hpp"
#include "error_code_utils/policy.hpp"
#include "error_code_utils/types.hpp"

namespace error_code_utils {

// -----------------------------------------------------------------------------
// Module Hub: manage per-module error states
// -----------------------------------------------------------------------------

class ModuleHub {
 public:
  ModuleHub(IClock& clock, SeverityPolicy policy, uint16_t domain_id)
      : clock_(clock), policy_(std::move(policy)), domain_id_(domain_id) {}

  bool bump(uint16_t code, Severity sev, Context ctx = {}) {
    const ErrorCode ec{domain_id_, code};
    Rule rule = policy_.rule_for(ec, sev);
    auto& counter = counters_[ec];

    const uint64_t candidate_id = instance_gen_ + 1;
    const bool edge = counter.bump(rule, ec, sev, clock_.now_ms(), candidate_id, std::move(ctx));
    if (edge) {
      instance_gen_ = candidate_id;
    }
    return edge;
  }

  void ack(uint16_t code) {
    const ErrorCode ec{domain_id_, code};
    auto it = counters_.find(ec);
    if (it != counters_.end()) {
      it->second.ack();
    }
  }

  void ack_instance(uint64_t instance_id) {
    for (auto& kv : counters_) {
      if (kv.second.active() && kv.second.instance_id() == instance_id) {
        kv.second.ack();
      }
    }
  }

  void tick() {
    const int64_t now_ms = clock_.now_ms();
    for (auto& kv : counters_) {
      const Rule rule = policy_.rule_for(kv.first, kv.second.last_severity());
      kv.second.tick(rule, now_ms);
    }
  }

  std::vector<ErrorState> snapshot_active() const {
    std::vector<ErrorState> out;
    out.reserve(counters_.size());
    for (const auto& kv : counters_) {
      const Rule rule = policy_.rule_for(kv.first, kv.second.last_severity());
      const ErrorState state = kv.second.to_state(rule, kv.first);
      if (state.active) {
        out.push_back(state);
      }
    }
    return out;
  }

  std::vector<ErrorState> snapshot_all() const {
    std::vector<ErrorState> out;
    out.reserve(counters_.size());
    for (const auto& kv : counters_) {
      const Rule rule = policy_.rule_for(kv.first, kv.second.last_severity());
      out.push_back(kv.second.to_state(rule, kv.first));
    }
    return out;
  }

 private:
  IClock& clock_;
  SeverityPolicy policy_;
  uint16_t domain_id_{0};
  std::unordered_map<ErrorCode, ErrorCounter> counters_{};
  uint64_t instance_gen_{0};
};

}  // namespace error_code_utils

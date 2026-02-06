#include <cassert>

#include "error_code_utils/clock.hpp"
#include "error_code_utils/module_hub.hpp"
#include "error_code_utils/policy.hpp"

using namespace error_code_utils;

int main() {
  FakeClock clock;
  SeverityPolicy policy;

  Rule rule{};
  rule.window_ms = 100;
  rule.threshold = 2;
  rule.latch = true;
  rule.clear_policy = ClearPolicy::AckOrHealthyMs;
  rule.healthy_clear_ms = 50;
  rule.priority = 10;

  const ErrorCode code{1, 7};
  policy.overrides[code] = rule;

  ModuleHub hub(clock, policy, 1);

  bool edge = hub.bump(7, Severity::Warning);
  assert(!edge);
  edge = hub.bump(7, Severity::Warning);
  assert(edge);

  auto active = hub.snapshot_active();
  assert(active.size() == 1);
  const uint64_t instance_id = active[0].instance_id;
  assert(instance_id == 1);

  hub.ack_instance(instance_id);
  active = hub.snapshot_active();
  assert(active.empty());

  // Re-activate should increment instance id
  hub.bump(7, Severity::Warning);
  hub.bump(7, Severity::Warning);
  active = hub.snapshot_active();
  assert(active.size() == 1);
  assert(active[0].instance_id == 2);

  // Auto-clear after healthy timeout
  clock.advance_ms(100);
  hub.tick();
  active = hub.snapshot_active();
  assert(active.empty());

  return 0;
}

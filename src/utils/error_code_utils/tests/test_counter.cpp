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
  rule.threshold = 3;
  rule.latch = true;
  rule.clear_policy = ClearPolicy::AckOnly;
  rule.healthy_clear_ms = 0;
  rule.priority = 10;

  const ErrorCode code{1, 1};
  policy.overrides[code] = rule;

  ModuleHub hub(clock, policy, 1);

  const bool edge1 = hub.bump(1, Severity::Warning);
  clock.advance_ms(50);
  const bool edge2 = hub.bump(1, Severity::Warning);
  clock.advance_ms(40);
  const bool edge3 = hub.bump(1, Severity::Warning);

  assert(!edge1);
  assert(!edge2);
  assert(edge3);

  auto states = hub.snapshot_all();
  assert(states.size() == 1);
  assert(states[0].active);
  assert(states[0].count == 3);

  clock.advance_ms(110);
  hub.tick();
  states = hub.snapshot_all();
  assert(states[0].active);
  assert(states[0].count == 0);

  hub.ack(1);
  states = hub.snapshot_all();
  assert(!states[0].active);
  assert(states[0].count == 0);

  rule.clear_policy = ClearPolicy::AutoHealthyMs;
  rule.healthy_clear_ms = 50;
  policy.overrides[code] = rule;

  clock.advance_ms(100);
  const bool edge4 = hub.bump(1, Severity::Warning);
  clock.advance_ms(40);
  const bool edge5 = hub.bump(1, Severity::Warning);
  clock.advance_ms(10);
  const bool edge6 = hub.bump(1, Severity::Warning);
  assert(edge6);
  assert(!edge4);
  assert(!edge5);

  clock.advance_ms(51);
  hub.tick();
  states = hub.snapshot_all();
  assert(!states[0].active);

  return 0;
}

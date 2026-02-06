#include <cassert>

#include "hub_utils/hub_aggregator.hpp"

using error_code_utils::ErrorCode;
using error_code_utils::ErrorState;
using error_code_utils::Severity;

int main() {
  hub_utils::HubAggregatorCore aggregator;

  ErrorState a1;
  a1.code = ErrorCode{1, 1};
  a1.severity = Severity::Recoverable;
  a1.active = true;
  a1.last_seen_ms = 100;
  a1.priority = 60;

  ErrorState a2 = a1;
  a2.last_seen_ms = 200;

  ErrorState b1;
  b1.code = ErrorCode{1, 2};
  b1.severity = Severity::Fatal;
  b1.active = true;
  b1.last_seen_ms = 150;
  b1.priority = 100;

  aggregator.add_snapshot("module_a", {a1, b1});
  aggregator.add_snapshot("module_b", {a2});

  auto global = aggregator.global_snapshot();
  assert(global.size() == 2);

  // Duplicate ErrorCode should keep latest last_seen_ms
  for (const auto& s : global) {
    if (s.code == ErrorCode{1, 1}) {
      assert(s.last_seen_ms == 200);
    }
  }

  const auto top = aggregator.top_priority_error();
  assert(top.has_value());
  const ErrorCode expected_top{1, 2};
  assert(top->code == expected_top);

  return 0;
}

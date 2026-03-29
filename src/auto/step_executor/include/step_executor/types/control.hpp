#pragma once

#include <cstdint>

#include "step_executor/types/context.hpp"

namespace step_executor {

enum class ControlKind : uint8_t {
  Delay = 0,
  Guard = 1,
};

struct ControlStep {
  ControlKind kind{ControlKind::Delay};

  int delay_ms{0};

  ContextKey guard_key{};
  bool require_present{true};
};

} // namespace step_executor

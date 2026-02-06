# error_code_utils

Pure C++17 core utilities for error state consistency (no ROS dependencies).

## Components
- types.hpp: ErrorCode, ErrorState, Severity, RecommendedAction
- clock.hpp: IClock, SteadyClock, FakeClock
- policy.hpp: Rule, SeverityPolicy, ClearPolicy
- counter.hpp: ErrorCounter（内部实现，建议仅通过 ModuleHub 使用）
- module_hub.hpp: ModuleHub (per-module error state manager)

## Quick Example
```cpp
#include "error_code_utils/module_hub.hpp"
#include "error_code_utils/clock.hpp"

error_code_utils::SteadyClock clock;
error_code_utils::SeverityPolicy policy;
error_code_utils::ModuleHub hub(clock, policy, 1);

hub.bump(42, error_code_utils::Severity::Recoverable);
const auto active = hub.snapshot_active();
```

## Build Tests
```bash
cmake -S . -B build -DERROR_CODE_UTILS_BUILD_TESTS=ON
cmake --build build
ctest --test-dir build
```

# log_utils

Header-only colored logging utilities based on vendored spdlog (bundled fmt).

## CMake usage

```cmake
find_package(log_utils REQUIRED)

target_link_libraries(your_target PRIVATE log_utils)
```

## Code usage

```cpp
#include <log_utils/log.hpp>

log_utils::init_console_logger("solve_core");
LOGI("hello {}", 123);
```

Notes:
- Header-only (SPDLOG_HEADER_ONLY)
- Uses bundled fmt from spdlog (no system fmt dependency)
- ANSI color output enabled by default

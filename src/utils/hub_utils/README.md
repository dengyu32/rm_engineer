# hub_utils

Pure C++17 core utilities for aggregating error states (no ROS dependencies).

## Components
- hub_aggregator.hpp: HubAggregatorCore

## Quick Example
```cpp
#include "hub_utils/hub_aggregator.hpp"

hub_utils::HubAggregatorCore hub;

// module snapshots collected elsewhere
hub.add_snapshot("module_a", snapshot_a);

auto global = hub.global_snapshot();
auto top = hub.top_priority_error();
```

## Build Tests
```bash
cmake -S . -B build -DHUB_UTILS_BUILD_TESTS=ON
cmake --build build
ctest --test-dir build
```

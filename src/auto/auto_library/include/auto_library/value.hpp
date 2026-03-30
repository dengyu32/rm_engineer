#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <variant>

namespace step_executor {

// ============================================================================
//  Value
// ----------------------------------------------------------------------------
//  - StepExecutor 可复用的通用值类型
//  - 不包含具体业务结构体
// ============================================================================

using Value = std::variant<
    bool,
    int64_t,
    double,
    std::string,
    std::array<double, 3>,  // vector
    std::array<double, 7>,  // pose
    std::array<float, 6>    // joints
    >;

} // namespace step_executor

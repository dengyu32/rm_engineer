#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <variant>

namespace core {

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
    std::vector<double>
    >;

// 统一访问接口，减少业务层重复 std::get_if.
// 获取 Value 中的具体类型指针，如果类型不匹配返回 nullptr
template <typename T>
inline const T *valueAs(const Value &value) {
  return std::get_if<T>(&value);
}

} // namespace core

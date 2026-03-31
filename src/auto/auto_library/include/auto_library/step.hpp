#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "auto_library/command.hpp"
#include "auto_library/context.hpp"

namespace core {

// ============================================================================
//  Step
// ----------------------------------------------------------------------------
//  - 统一为 Command Step
//  - inputs/outputs/bindings 显式定义数据流
// ============================================================================

enum class BindingOp : uint8_t {
  Direct = 0,
  IndexToJointsTable = 1,
};

struct Binding {
  ContextKey from{};  // 绑定着一个 value
  std::string to_param{};   // 注入到 command.param 的参数名 , 同时也绑定着一个 value
  BindingOp op{BindingOp::Direct};
  const std::array<float, 6> *joints_table{nullptr};
  size_t joints_table_size{0};
};

struct Step {
  std::string id{};
  std::string label{};

  Command command{};
  std::vector<ContextKey> inputs{};  // 绑定输入的 value keys
  std::vector<ContextKey> outputs{};  // 绑定输出的 value keys
  std::vector<Binding> bindings{};

  int post_delay_ms{0};
  int timeout_ms{0};
  int max_retries{0};
};

// 修改原因: 统一 Binding 构造，减少业务层样板代码.
inline Binding bindDirect(ContextKey from, std::string to_param) {
  Binding binding{};
  binding.from = std::move(from);
  binding.to_param = std::move(to_param);
  binding.op = BindingOp::Direct;
  return binding;
}

} // namespace core

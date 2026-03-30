#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "auto_library/command.hpp"
#include "auto_library/context.hpp"

namespace step_executor {

// ============================================================================
//  Step
// ----------------------------------------------------------------------------
//  - Command Step：执行 command
//  - Control Step：执行控制逻辑
//  - inputs/outputs/bindings 显式定义数据流
// ============================================================================

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

enum class StepType : uint8_t {
  Command = 0,
  Control = 1,
};

enum class BindingOp : uint8_t {
  Direct = 0,
  IndexToJointsTable = 1,
};

struct Binding {
  ContextKey from{};
  std::string to_param{};
  BindingOp op{BindingOp::Direct};
  const std::array<float, 6> *joints_table{nullptr};
  size_t joints_table_size{0};
};

struct Step {
  std::string id{};
  std::string label{};

  StepType type{StepType::Command};
  Command command{};
  ControlStep control{};
  std::vector<ContextKey> inputs{};
  std::vector<ContextKey> outputs{};
  std::vector<Binding> bindings{};

  int timeout_ms{0};
  int max_retries{0};
};

} // namespace step_executor

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "step_executor/types/command.hpp"
#include "step_executor/types/control.hpp"
#include "step_executor/types/context.hpp"

namespace step_executor {

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

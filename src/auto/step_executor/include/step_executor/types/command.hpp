#pragma once

#include <string>
#include <unordered_map>

#include "step_executor/types/value.hpp"

namespace step_executor {

// ============================================================================
//  Command
// ----------------------------------------------------------------------------
//  - Step 层唯一执行对象
//  - kind + params（不做业务解释）
// ============================================================================

struct Command {
  std::string kind{};
  std::unordered_map<std::string, Value> params{};
};

template <typename T>
const T *paramAs(const Command &cmd, const std::string &name) {
  auto it = cmd.params.find(name);
  if (it == cmd.params.end()) {
    return nullptr;
  }
  return std::get_if<T>(&it->second);
}

template <typename T>
bool getParam(const Command &cmd, const std::string &name, T &out) {
  const T *value = paramAs<T>(cmd, name);
  if (!value) {
    return false;
  }
  out = *value;
  return true;
}

} // namespace step_executor

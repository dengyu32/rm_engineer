#pragma once

#include <any>
#include <string>
#include <unordered_map>

namespace step_executor {

struct Command {
  std::string kind{};
  std::unordered_map<std::string, std::any> params{};
};

template <typename T>
const T *paramAs(const Command &cmd, const std::string &name) {
  auto it = cmd.params.find(name);
  if (it == cmd.params.end()) {
    return nullptr;
  }
  return std::any_cast<T>(&it->second);
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

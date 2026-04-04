/*
- 统一执行对象：所有 Step 最终都转换成 Command，StepExecutor 只认识它，不理解业务。
  - 能力层入口：每个 capability 的 execute 都只接收 Command，因此能力层只关心参数解析与执行。
  - 解耦三层：Task 层拼装 Command，Step 层转发 Command，Capability 层解释 Command。三层耦合点只有这一种结
    构。
  - 显式数据注入：Command.params 是动态键值表（variant），配合 Binding 把上下文数据注入到具体参数字段。
  - 复用与扩展：新增能力只需新增 kind + 约定参数字段，不需要改 StepExecutor。

  command.hpp 是 Task/Step/Capability 三层之间唯一的运行时协议。
*/

#pragma once

#include <string>
#include <unordered_map>
#include <utility>

#include "auto_library/value.hpp"

namespace core {

// ============================================================================
//  Command
// ----------------------------------------------------------------------------
//  - Step 层唯一执行对象
//  - kind + params（不做业务解释）
// ============================================================================

struct Command {
  std::string kind{};
  std::unordered_map<std::string, Value> params{}; // 哈希表 
};

// paramAs 
// 根据name 从哈希表中查找参数 返回 value 指针
template <typename T>
const T *paramAs(const Command &cmd, const std::string &name) {
  auto it = cmd.params.find(name);
  if (it == cmd.params.end()) {
    return nullptr;
  }
  return std::get_if<T>(&it->second);
}

// requireParam
// 调用 paramAs 获取 value 指针,如果不存在就设置 err 错误信息
template <typename T>
const T *requireParam(const Command &cmd, const std::string &name, std::string &err) {
  const T *value = paramAs<T>(cmd, name);
  if (!value) {
    err = "missing param: " + name;
  }
  return value;
}

// getParam
// 调用 paramAs 获取 value 指针,如果存在就解引用复制给 out
template <typename T>
bool getParam(const Command &cmd, const std::string &name, T &out) {
  const T *value = paramAs<T>(cmd, name);
  if (!value) {
    return false;
  }
  out = *value;
  return true;
}

// 函数重载 构造 Command.params 的不同方式
inline void setParam(Command &cmd, const std::string &name, Value value) {
  cmd.params[name] = std::move(value);
}

inline void setParam(Command &cmd, const std::string &name, const std::string &value) {
  cmd.params[name] = value;
}

inline void setParam(Command &cmd, const std::string &name, std::string &&value) {
  cmd.params[name] = std::move(value);
}

inline void setParam(Command &cmd, const std::string &name, const char *value) {
  cmd.params[name] = std::string(value);
}

} // namespace core

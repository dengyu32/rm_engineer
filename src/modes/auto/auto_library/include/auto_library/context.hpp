#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <unordered_map>

#include "auto_library/value.hpp"

namespace core {

// ============================================================================
//  Context
// ----------------------------------------------------------------------------
//  - 显式共享数据表（Key -> Value）
//  - 支持 Task / Persist 两种 scope
// ============================================================================

// contextScope
// 现在只用到 task
enum class ContextScope : uint8_t {
  Task = 0,
  Persist = 1,
};

// contextKey
// 其实只是一个 string
struct ContextKey {
  std::string name{};
  ContextScope scope{ContextScope::Task};
};

// taskKey / persistKey
// 构造 ContextKey 的函数
inline ContextKey taskKey(std::string name) {
  return ContextKey{std::move(name), ContextScope::Task};
}

inline ContextKey persistKey(std::string name) {
  return ContextKey{std::move(name), ContextScope::Persist};
}

inline bool operator==(const ContextKey &lhs, const ContextKey &rhs) {
  return lhs.scope == rhs.scope && lhs.name == rhs.name;
}

// contextStore

class ContextStore {
public:
  void clearTask() { task_.clear(); }

  bool has(const ContextKey &key) const { return findEntry(key) != nullptr; }

  // 取 value
  bool get(const ContextKey &key, Value &out) const {
    const Entry *entry = findEntry(key);
    if (!entry) {
      return false;
    }
    out = entry->value;
    return true;
  }

  // 存 value
  bool set(const ContextKey &key, const Value &value) {
    // 找到地图
    auto &map = mapFor(key.scope);
    // 类型索引是否一致
    const size_t type_index = value.index();
    auto it = map.find(key.name);
    if (it != map.end()) {
      if (it->second.type_index != type_index) {
        return false;
      }
      it->second.value = value;
      it->second.type_index = type_index;
      return true;
    }
    // 没有找到就直接构造一个 entry 插入
    map.emplace(key.name, Entry{value, type_index});
    return true;
  }

private:
  // entry 包含 value 和 值的类型索引
  struct Entry {
    Value value;
    size_t type_index{0};
  };

  // 根据 key 查找 entry 指针
  const Entry *findEntry(const ContextKey &key) const {
    const auto &map = mapFor(key.scope);
    auto it = map.find(key.name);
    if (it == map.end()) {
      return nullptr;
    }
    return &it->second;
  }

  // 根据 scope 返回对应的哈希表引用
  std::unordered_map<std::string, Entry> &mapFor(ContextScope scope) {
    return scope == ContextScope::Persist ? persist_ : task_;
  }

  const std::unordered_map<std::string, Entry> &mapFor(ContextScope scope) const {
    return scope == ContextScope::Persist ? persist_ : task_;
  }

  // 两个哈希表 task_ / persist_ 分别存储不同 scope 的数据
  std::unordered_map<std::string, Entry> task_{};
  std::unordered_map<std::string, Entry> persist_{};
};

} // namespace core

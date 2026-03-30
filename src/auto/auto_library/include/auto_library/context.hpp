#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <unordered_map>

#include "auto_library/value.hpp"

namespace step_executor {

// ============================================================================
//  Context
// ----------------------------------------------------------------------------
//  - 显式共享数据表（Key -> Value）
//  - 支持 Task / Persist 两种 scope
// ============================================================================

enum class ContextScope : uint8_t {
  Task = 0,
  Persist = 1,
};

struct ContextKey {
  std::string name{};
  ContextScope scope{ContextScope::Task};
};

inline bool operator==(const ContextKey &lhs, const ContextKey &rhs) {
  return lhs.scope == rhs.scope && lhs.name == rhs.name;
}

class ContextStore {
public:
  void clearTask() { task_.clear(); }

  bool has(const ContextKey &key) const { return findEntry(key) != nullptr; }

  bool get(const ContextKey &key, Value &out) const {
    const Entry *entry = findEntry(key);
    if (!entry) {
      return false;
    }
    out = entry->value;
    return true;
  }

  bool set(const ContextKey &key, const Value &value) {
    auto &map = mapFor(key.scope);
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
    map.emplace(key.name, Entry{value, type_index});
    return true;
  }

private:
  struct Entry {
    Value value;
    size_t type_index{0};
  };

  const Entry *findEntry(const ContextKey &key) const {
    const auto &map = mapFor(key.scope);
    auto it = map.find(key.name);
    if (it == map.end()) {
      return nullptr;
    }
    return &it->second;
  }

  std::unordered_map<std::string, Entry> &mapFor(ContextScope scope) {
    return scope == ContextScope::Persist ? persist_ : task_;
  }

  const std::unordered_map<std::string, Entry> &mapFor(ContextScope scope) const {
    return scope == ContextScope::Persist ? persist_ : task_;
  }

  std::unordered_map<std::string, Entry> task_{};
  std::unordered_map<std::string, Entry> persist_{};
};

} // namespace step_executor

#pragma once

#include <any>
#include <cstdint>
#include <string>
#include <typeinfo>
#include <unordered_map>

namespace step_executor {

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

  bool get(const ContextKey &key, std::any &out) const {
    const Entry *entry = findEntry(key);
    if (!entry) {
      return false;
    }
    out = entry->value;
    return true;
  }

  bool set(const ContextKey &key, const std::any &value) {
    auto &map = mapFor(key.scope);
    const std::type_info &type = value.type();
    auto it = map.find(key.name);
    if (it != map.end()) {
      if (it->second.type && *(it->second.type) != type) {
        return false;
      }
      it->second.value = value;
      it->second.type = &type;
      return true;
    }
    map.emplace(key.name, Entry{value, &type});
    return true;
  }

private:
  struct Entry {
    std::any value;
    const std::type_info *type{nullptr};
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

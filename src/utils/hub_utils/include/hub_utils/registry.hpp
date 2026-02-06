#pragma once

//==============================================================
// hub_utils/registry.hpp
// Lightweight registry for init tasks.
//==============================================================

#include <functional>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <vector>

namespace hub_utils {

//==============================================================
// Registry
// - Register init tasks by id, then run in registration order.
// - If id repeats, the latest entry replaces the previous one.
// - init_all throws std::runtime_error if any init throws.
//==============================================================

template <class Id>
class Registry {
 public:
  using InitFn = std::function<void()>;

  void add(const Id& id, InitFn fn) {
    auto it = index_.find(id);
    if (it == index_.end()) {
      tasks_.push_back(std::move(fn));
      index_.emplace(id, tasks_.size() - 1);
      return;
    }
    tasks_[it->second] = std::move(fn);
  }

  bool has(const Id& id) const {
    return index_.find(id) != index_.end();
  }

  void init_all() {
    try {
      for (auto& fn : tasks_) {
        if (fn) {
          fn();
        }
      }
    } catch (const std::exception& e) {
      throw std::runtime_error(std::string("Registry init_all failed: ") + e.what());
    }
  }

  void clear() {
    tasks_.clear();
    index_.clear();
  }

 private:
  std::vector<InitFn> tasks_;
  std::unordered_map<Id, size_t> index_;
};

}  // namespace hub_utils

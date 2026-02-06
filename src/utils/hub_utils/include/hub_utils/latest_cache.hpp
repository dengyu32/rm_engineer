#pragma once

//==============================================================
// hub_utils/latest_cache.hpp
// Thread-safe latest-value cache.
//==============================================================

#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <utility>

namespace hub_utils {

//==============================================================
// LatestCache
// - Stores only the most recent value.
// - latest_copy throws std::runtime_error if empty.
//==============================================================

template <class T>
class LatestCache {
 public:
  void update(const T& v) {
    std::lock_guard<std::mutex> lock(mu_);
    latest_ = v;
    last_update_tp_ = std::chrono::steady_clock::now();
  }

  void update(T&& v) {
    std::lock_guard<std::mutex> lock(mu_);
    latest_ = std::move(v);
    last_update_tp_ = std::chrono::steady_clock::now();
  }

  bool has() const {
    std::lock_guard<std::mutex> lock(mu_);
    return latest_.has_value();
  }

  T latest_copy() const {
    std::lock_guard<std::mutex> lock(mu_);
    if (!latest_) {
      throw std::runtime_error("LatestCache is empty");
    }
    return *latest_;
  }

  std::shared_ptr<const T> latest_ptr() const {
    std::lock_guard<std::mutex> lock(mu_);
    if (!latest_) {
      return nullptr;
    }
    return std::make_shared<T>(*latest_);
  }

  std::optional<std::chrono::steady_clock::time_point> last_update_tp() const {
    std::lock_guard<std::mutex> lock(mu_);
    return last_update_tp_;
  }

 private:
  mutable std::mutex mu_;
  std::optional<T> latest_;
  std::optional<std::chrono::steady_clock::time_point> last_update_tp_;
};

}  // namespace hub_utils

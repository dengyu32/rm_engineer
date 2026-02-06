#pragma once

//==============================================================
// hub_utils/inbox.hpp
// Thread-safe inbox: push from callbacks, drain from worker.
//==============================================================

// 收件箱

#include <deque>
#include <mutex>
#include <utility>
#include <vector>

namespace hub_utils {

//==============================================================
// Inbox
// - max_depth == 0 means unbounded.
// - If max_depth is exceeded, the oldest item is dropped.
//==============================================================

template <class T>
class Inbox {
 public:
  explicit Inbox(size_t max_depth = 0) : max_depth_(max_depth) {}

  void push(const T& v) {
    std::lock_guard<std::mutex> lock(mu_);
    if (max_depth_ > 0 && queue_.size() >= max_depth_) {
      queue_.pop_front();
    }
    queue_.push_back(v);
  }

  void push(T&& v) {
    std::lock_guard<std::mutex> lock(mu_);
    if (max_depth_ > 0 && queue_.size() >= max_depth_) {
      queue_.pop_front();
    }
    queue_.push_back(std::move(v));
  }

  std::vector<T> drain() {
    std::lock_guard<std::mutex> lock(mu_);
    std::vector<T> out;
    out.reserve(queue_.size());
    while (!queue_.empty()) {
      out.push_back(std::move(queue_.front()));
      queue_.pop_front();
    }
    return out;
  }

  size_t size() const {
    std::lock_guard<std::mutex> lock(mu_);
    return queue_.size();
  }

 private:
  size_t max_depth_{0};
  mutable std::mutex mu_;
  std::deque<T> queue_;
};

}  // namespace hub_utils

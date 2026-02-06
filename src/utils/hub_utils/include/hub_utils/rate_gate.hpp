#pragma once

//==============================================================
// hub_utils/rate_gate.hpp
// Rate limiting and deduplication gates.
//==============================================================

#include <chrono>
#include <deque>
#include <mutex>
#include <unordered_map>
#include <utility>

namespace hub_utils {

//==============================================================
// RateGate
// - Sliding 1-second window.
// - allow(now) returns true if within max_per_sec.
//==============================================================

class RateGate {
 public:
  explicit RateGate(size_t max_per_sec) : max_per_sec_(max_per_sec) {}

  bool allow(std::chrono::steady_clock::time_point now) {
    std::lock_guard<std::mutex> lock(mu_);
    prune_(now);
    if (max_per_sec_ == 0) {
      ++dropped_;
      return false;
    }
    if (timestamps_.size() >= max_per_sec_) {
      ++dropped_;
      return false;
    }
    timestamps_.push_back(now);
    return true;
  }

  size_t dropped() const {
    std::lock_guard<std::mutex> lock(mu_);
    return dropped_;
  }

 private:
  void prune_(std::chrono::steady_clock::time_point now) {
    const auto window = std::chrono::seconds(1);
    while (!timestamps_.empty() && (now - timestamps_.front()) >= window) {
      timestamps_.pop_front();
    }
  }

  size_t max_per_sec_{0};
  mutable std::mutex mu_;
  std::deque<std::chrono::steady_clock::time_point> timestamps_;
  size_t dropped_{0};
};

//==============================================================
// DedupGate
// - Suppress same key within a time window.
//==============================================================

template <class Key>
class DedupGate {
 public:
  explicit DedupGate(std::chrono::milliseconds window) : window_(window) {}

  bool should_accept(const Key& key, std::chrono::steady_clock::time_point now) {
    std::lock_guard<std::mutex> lock(mu_);
    prune_(now);
    auto it = last_seen_.find(key);
    if (it != last_seen_.end() && (now - it->second) < window_) {
      ++suppressed_;
      return false;
    }
    last_seen_[key] = now;
    return true;
  }

  size_t suppressed() const {
    std::lock_guard<std::mutex> lock(mu_);
    return suppressed_;
  }

 private:
  void prune_(std::chrono::steady_clock::time_point now) {
    for (auto it = last_seen_.begin(); it != last_seen_.end(); ) {
      if ((now - it->second) >= window_) {
        it = last_seen_.erase(it);
      } else {
        ++it;
      }
    }
  }

  std::chrono::milliseconds window_{0};
  mutable std::mutex mu_;
  std::unordered_map<Key, std::chrono::steady_clock::time_point> last_seen_;
  size_t suppressed_{0};
};

}  // namespace hub_utils

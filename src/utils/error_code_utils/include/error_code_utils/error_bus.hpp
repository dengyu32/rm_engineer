#pragma once

#include <deque>
#include <mutex>
#include <vector>

#include "error_code_utils/error.hpp"

namespace error_code_utils {

class ErrorBus {
public:
  void publish(const Error &err) {
    std::lock_guard<std::mutex> lock(mutex_);
    queue_.push_back(err);
  }

  void drain(std::vector<Error> &out) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (queue_.empty()) {
      return;
    }
    out.reserve(out.size() + queue_.size());
    while (!queue_.empty()) {
      out.push_back(queue_.front());
      queue_.pop_front();
    }
  }

private:
  std::mutex mutex_;
  std::deque<Error> queue_;
};

} // namespace error_code_utils

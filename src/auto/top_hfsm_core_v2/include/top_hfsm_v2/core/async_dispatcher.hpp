#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace top_hfsm_v2 {

enum class TaskStatus : uint8_t {
  Success = 0,
  Timeout = 1,
  Failure = 2,
};

class AsyncDispatcher {
public:
  struct Result {
    std::string tag;
    TaskStatus status{TaskStatus::Failure};
    std::string message;
    int64_t done_ms{0};
  };

  using JobFn = std::function<Result()>; // 无参数函数返回Result

  AsyncDispatcher();
  ~AsyncDispatcher();

  bool start();

  void stop();

  void enqueue(const std::string &tag, const JobFn &fn);

  void poll(std::vector<Result> &out_results);

private:
  struct JobItem {
    JobItem(const std::string &tag_in, const JobFn &fn_in)
        : tag(tag_in), fn(fn_in) {}
    std::string tag;
    JobFn fn;
  };

  void workerLoop();

  static int64_t currentTimeMs();

  std::atomic<bool> running_;
  std::thread worker_thread_;

  std::mutex job_mutex_;
  std::condition_variable job_cv_;
  std::deque<JobItem> jobs_;

  std::mutex result_mutex_;
  std::vector<Result> results_;
};

} // namespace top_hfsm_v2

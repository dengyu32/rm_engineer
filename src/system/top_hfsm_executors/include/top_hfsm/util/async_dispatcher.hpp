// ============================================================================
// AsyncDispatcher
// ----------------------------------------------------------------------------
// - 单后台线程执行可能阻塞的任务（如服务调用）
// - tick 线程通过 poll 读取结果，避免定时器被阻塞
// - 仅依赖 C++11 标准库
// ============================================================================
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

namespace top_hfsm {

// ----------------------------------------------------------------------------
// TaskStatus：统一状态（success / timeout / failure）
// ----------------------------------------------------------------------------
enum class TaskStatus : uint8_t {
  Success = 0,
  Timeout = 1,
  Failure = 2,
};

// ----------------------------------------------------------------------------
// AsyncDispatcher：后台任务调度器
// ----------------------------------------------------------------------------
class AsyncDispatcher {
public:
  struct Result {
    std::string tag;
    TaskStatus status{TaskStatus::Failure};
    std::string message;
    int64_t done_ms{0}; // steady_clock 毫秒
  };

  typedef std::function<Result()> JobFn;

  AsyncDispatcher() : running_(false) {}
  ~AsyncDispatcher() { stop(); }

  bool start() {
    bool expected = false;
    if (!running_.compare_exchange_strong(expected, true))
      return false;
    worker_thread_ = std::thread(&AsyncDispatcher::workerLoop, this);
    return true;
  }

  void stop() {
    if (!running_.load())
      return;
    running_.store(false);
    {
      std::lock_guard<std::mutex> lk(job_mutex_);
      // wake up
    }
    job_cv_.notify_all();
    if (worker_thread_.joinable())
      worker_thread_.join();
  }

  // 提交任务：立即返回
  void enqueue(const std::string &tag, const JobFn &fn) {
    if (!running_.load())
      return;
    {
      std::lock_guard<std::mutex> lk(job_mutex_);
      jobs_.push_back(JobItem(tag, fn));
    }
    job_cv_.notify_one();
  }

  // 非阻塞取出全部已完成结果
  void poll(std::vector<Result> &out_results) {
    std::vector<Result> tmp;
    {
      std::lock_guard<std::mutex> lk(result_mutex_);
      tmp.swap(results_);
    }
    out_results.insert(out_results.end(), tmp.begin(), tmp.end());
  }

private:
  struct JobItem {
    JobItem(const std::string &t, const JobFn &f) : tag(t), fn(f) {}
    std::string tag;
    JobFn fn;
  };

  void workerLoop() {
    while (true) {
      JobItem item("", JobFn());
      {
        std::unique_lock<std::mutex> lk(job_mutex_);
        job_cv_.wait(lk, [this]() {
          return !running_.load() || !jobs_.empty();
        });
        if (!running_.load() && jobs_.empty())
          break;
        if (!jobs_.empty()) {
          item = jobs_.front();
          jobs_.pop_front();
        } else {
          continue;
        }
      }

      Result result;
      result.tag = item.tag;
      try {
        result = item.fn();
        if (result.tag.empty())
          result.tag = item.tag;
      } catch (const std::exception &e) {
        result.status = TaskStatus::Failure;
        result.message = e.what();
      } catch (...) {
        result.status = TaskStatus::Failure;
        result.message = "unknown exception";
      }

      result.done_ms = currentTimeMs();

      {
        std::lock_guard<std::mutex> lk(result_mutex_);
        results_.push_back(result);
      }
    }
  }

  static int64_t currentTimeMs() {
    typedef std::chrono::steady_clock clock_t;
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               clock_t::now().time_since_epoch())
        .count();
  }

private:
  std::atomic<bool> running_;
  std::thread worker_thread_;

  std::mutex job_mutex_;
  std::condition_variable job_cv_;
  std::deque<JobItem> jobs_;

  std::mutex result_mutex_;
  std::vector<Result> results_;
};

} // namespace top_hfsm

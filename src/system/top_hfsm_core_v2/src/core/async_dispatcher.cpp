#include "top_hfsm_v2/core/async_dispatcher.hpp"

#include <exception>
#include <utility>

namespace top_hfsm_v2 {

AsyncDispatcher::AsyncDispatcher() : running_(false) {}

AsyncDispatcher::~AsyncDispatcher() { stop(); }

bool AsyncDispatcher::start() {
  bool expected = false;
  if (!running_.compare_exchange_strong(expected, true)) {
    return false;
  }
  worker_thread_ = std::thread(&AsyncDispatcher::workerLoop, this);
  return true;
}

void AsyncDispatcher::stop() {
  if (!running_.load()) {
    return;
  }
  running_.store(false);
  {
    std::lock_guard<std::mutex> lock(job_mutex_);
  }
  job_cv_.notify_all();
  if (worker_thread_.joinable()) {
    worker_thread_.join();
  }
}

void AsyncDispatcher::enqueue(const std::string &tag, const JobFn &fn) {
  if (!running_.load()) {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(job_mutex_);
    jobs_.push_back(JobItem(tag, fn));
  }
  job_cv_.notify_one();
}

void AsyncDispatcher::poll(std::vector<Result> &out_results) {
  std::vector<Result> tmp;
  {
    std::lock_guard<std::mutex> lock(result_mutex_);
    tmp.swap(results_);
  }
  out_results.insert(out_results.end(), tmp.begin(), tmp.end());
}

void AsyncDispatcher::workerLoop() {
  while (true) {
    JobItem item("", JobFn());
    {
      std::unique_lock<std::mutex> lock(job_mutex_);
      job_cv_.wait(lock, [this]() {
        return !running_.load() || !jobs_.empty();
      });
      if (!running_.load() && jobs_.empty()) {
        break;
      }
      if (jobs_.empty()) {
        continue;
      }
      item = std::move(jobs_.front());
      jobs_.pop_front();
    }

    Result result;
    result.tag = item.tag;
    try {
      result = item.fn();
      if (result.tag.empty()) {
        result.tag = item.tag;
      }
    } catch (const std::exception &e) {
      result.status = TaskStatus::Failure;
      result.message = e.what();
    } catch (...) {
      result.status = TaskStatus::Failure;
      result.message = "unknown exception";
    }

    result.done_ms = currentTimeMs();

    {
      std::lock_guard<std::mutex> lock(result_mutex_);
      results_.push_back(std::move(result));
    }
  }
}

int64_t AsyncDispatcher::currentTimeMs() {
  using clock_t = std::chrono::steady_clock;
  return std::chrono::duration_cast<std::chrono::milliseconds>(
             clock_t::now().time_since_epoch())
      .count();
}

} // namespace top_hfsm_v2

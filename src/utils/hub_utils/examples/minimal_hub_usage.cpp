// Minimal, ROS-agnostic usage examples for hub_utils.

#include <chrono>
#include <string>

#include "hub_utils/inbox.hpp"
#include "hub_utils/latest_cache.hpp"
#include "hub_utils/rate_gate.hpp"
#include "hub_utils/registry.hpp"

namespace {

enum class TopicId { kPose, kError };

struct ErrorEvent {
  int code{0};
  std::string msg;
};

struct Msg {
  double x{0.0};
};

}  // namespace

int main() {
  // Registry: register init tasks and run later.
  hub_utils::Registry<TopicId> registry;
  registry.add(TopicId::kPose, []() {
    // init pose subscriber
  });
  registry.add(TopicId::kError, []() {
    // init error subscriber
  });
  registry.init_all();

  // Inbox: push in callback, drain in worker.
  hub_utils::Inbox<ErrorEvent> error_inbox(128);
  error_inbox.push(ErrorEvent{1, "overheat"});
  auto batch = error_inbox.drain();

  // LatestCache: update in callback, read latest in worker.
  hub_utils::LatestCache<Msg> pose_cache;
  pose_cache.update(Msg{1.23});
  Msg latest = pose_cache.latest_copy();

  // RateGate + DedupGate: flood protection.
  hub_utils::RateGate rate_gate(10);  // allow 10 per second
  hub_utils::DedupGate<int> dedup_gate(std::chrono::milliseconds(200));

  auto now = std::chrono::steady_clock::now();
  if (rate_gate.allow(now) && dedup_gate.should_accept(1, now)) {
    // accept event
  }

  (void)batch;
  (void)latest;
  return 0;
}

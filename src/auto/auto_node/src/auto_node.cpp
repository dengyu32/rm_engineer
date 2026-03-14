#include "auto_node/auto_node.hpp"

#include <chrono>
#include <functional>

namespace engineer_auto {

using namespace task_step_library;

AutoNode::AutoNode(const rclcpp::NodeOptions &options)
    : rclcpp::Node("auto_node", options),
      config_(AutoNodeConfig::load(*this)),
      logger_(this->get_logger()),
      executor_(this->get_logger(), step_executor::createDefaultCapabilityBridge(*this)) {
  latest_task_id_.store(TaskId::IDLE, std::memory_order_relaxed);

  intent_sub_ = this->create_subscription<engineer_interfaces::msg::Intent>(
      config_.intent_cmd_topic, rclcpp::QoS(10),
      std::bind(&AutoNode::intentCallback, this, std::placeholders::_1));
  feedback_pub_ = this->create_publisher<engineer_interfaces::msg::Intent>(
      config_.intent_fb_topic, rclcpp::QoS(10));

  timer_ = this->create_wall_timer(std::chrono::milliseconds(config_.update_period_ms),
                                   std::bind(&AutoNode::tick, this));

  RCLCPP_INFO(logger_, "\n%s", config_.summary().c_str());
  publishFeedback(TaskId::IDLE, TaskFinishCode::Running);
  RCLCPP_INFO(logger_, "[AUTO_NODE] started");
}

void AutoNode::intentCallback(engineer_interfaces::msg::Intent::ConstSharedPtr msg) {
  TaskId task_id = TaskId::IDLE;
  if (!toTaskId(msg->intent_id, task_id)) {
    RCLCPP_WARN(logger_, "[AUTO_NODE] unsupported intent_id=%u", msg->intent_id);
    return;
  }
  latest_task_id_.store(task_id, std::memory_order_relaxed);
}

void AutoNode::tick() {
  const TaskId latest = latest_task_id_.load(std::memory_order_relaxed);
  if (latest != applied_task_id_) {
    handleIntent(latest);
    applied_task_id_ = latest;
  }

  if (executor_.isRunning()) {
    static rclcpp::Clock steady_clock(RCL_STEADY_TIME);
    executor_.tick(steady_clock.now());
  }

  if (executor_.isFinished()) {
    const TaskResult report = executor_.report();
    const TaskId done_task_id = executor_.activeTaskId();
    const TaskFinishCode finish_code =
        report.status == TaskStatus::Success ? TaskFinishCode::Finished
                                             : TaskFinishCode::Aborted;

    publishFeedback(done_task_id, finish_code);
    RCLCPP_INFO(logger_, "[AUTO_NODE] task done status=%u msg=%s",
                static_cast<unsigned>(report.status), report.message.c_str());

    executor_.reset();
  }
}

void AutoNode::handleIntent(TaskId task_id) {
  if (executor_.isRunning()) {
    executor_.cancel();
  }

  if (task_id == TaskId::IDLE) {
    // IDLE 状态返回 Running 表明 IDLE 正常运行
    publishFeedback(TaskId::IDLE, TaskFinishCode::Running);
    return;
  }

  TaskRequest req;
  req.task_id = task_id;
  const auto plan = orchestrator_.plan(req);
  if (!plan) {
    publishFeedback(task_id, TaskFinishCode::Aborted);
    RCLCPP_WARN(logger_, "[AUTO_NODE] no plan for task=%u",
                static_cast<unsigned>(task_id));
    return;
  }

  executor_.start(*plan);
  publishFeedback(task_id, TaskFinishCode::Running);
}

void AutoNode::publishFeedback(TaskId task_id, TaskFinishCode code) {
  engineer_interfaces::msg::Intent msg;
  msg.stamp = this->now();
  msg.intent_id = static_cast<uint8_t>(task_id);
  msg.intent_finish = static_cast<uint8_t>(code);
  feedback_pub_->publish(msg);
}

bool AutoNode::toTaskId(uint8_t raw, TaskId &out) {
  TaskId tmp = static_cast<TaskId>(raw);
  if (!is_supported_task(tmp)) {
    return false;
  }
  out = tmp;
  return true;
}

} // namespace engineer_auto

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(engineer_auto::AutoNode)

#include "auto_node/auto_node.hpp"
#include "auto_node/capability_registry.hpp"

#include <chrono>
#include <functional>
#include <cstdio>

namespace engineer_auto {

using namespace task_orchestrator;

// ============================================================================
//  CTOR
// ============================================================================

AutoNode::AutoNode(const rclcpp::NodeOptions &options)
    : rclcpp::Node("auto_node", options),
      config_(AutoNodeConfig::load(*this)),
      logger_(this->get_logger()),
      executor_(this->get_logger(), auto_node::createAutoCapabilityBridge(*this)) {
  latest_task_id_.store(TaskId::IDLE, std::memory_order_relaxed);

  initRosInterfaces();

  timer_ = this->create_wall_timer(std::chrono::milliseconds(config_.update_period_ms),
                                   std::bind(&AutoNode::tick, this));
  status_timer_ = this->create_wall_timer(std::chrono::milliseconds(config_.status_period_ms),
                                   std::bind(&AutoNode::statusTick, this));

  RCLCPP_INFO(logger_, "\n%s", config_.summary().c_str());
  publishFeedback(TaskId::IDLE, core::TaskStatus::Running);
  RCLCPP_INFO(logger_, "[AUTO_NODE] started");
}

// ============================================================================
//  ROS
// ============================================================================

void AutoNode::initRosInterfaces(){
  intent_sub_ = this->create_subscription<engineer_interfaces::msg::Intent>(
      config_.intent_cmd_topic, rclcpp::QoS(10),
      std::bind(&AutoNode::intentCallback, this, std::placeholders::_1));
  feedback_pub_ = this->create_publisher<engineer_interfaces::msg::Intent>(
      config_.intent_fb_topic, rclcpp::QoS(10));
  status_pub_ = this->create_publisher<std_msgs::msg::String>(
      config_.auto_status_topic, rclcpp::QoS(10));
}

// ============================================================================
//  ROS CallBack
// ============================================================================

void AutoNode::intentCallback(engineer_interfaces::msg::Intent::ConstSharedPtr msg) {
  TaskId task_id = TaskId::IDLE;
  if (!toTaskId(msg->intent_id, task_id)) {
    RCLCPP_WARN_THROTTLE(
      logger_,
      *this->get_clock(),
      1000,  // 毫秒，5秒最多打印一次
      "[AUTO_NODE] unsupported intent_id=%u",
      msg->intent_id
    );
    return;
  }
  latest_task_id_.store(task_id, std::memory_order_relaxed);
}

void AutoNode::tick() {
  // 读取 latest_task_id_ 是原子化操作，applied_task_id_只在tick中使用，不是共享变量
  // 1. 处理得到的 task_id_ ，executor_ 开始工作
  const TaskId latest = latest_task_id_.load(std::memory_order_relaxed);
  if (latest != applied_task_id_) {
    std::fputs("\n", stdout);
    std::fputs("=================== AUTO NODE TASK START ===================\n", stdout);
    std::fflush(stdout);
    RCLCPP_INFO(logger_,
                " [AUTO_NODE] START task=%s (id=%u)",
                task_orchestrator::task_name(latest),
                static_cast<unsigned>(latest));
    handleIntent(latest);
    applied_task_id_ = latest;
  }

  // 2. 如果task_id_没有被更新，同时executor_正在执行，驱动executor_,并更新当前status_text
  if (executor_.isRunning()) {
    static rclcpp::Clock steady_clock(RCL_STEADY_TIME);
    executor_.tick(steady_clock.now());
    TaskId running_task_id = TaskId::IDLE;
    const uint8_t running_raw = executor_.activeTaskId();
    toTaskId(running_raw, running_task_id);
    const std::size_t step_index = executor_.currentStepIndex();
    const std::size_t step_total = executor_.totalSteps();
    const std::string step_label = executor_.currentStepLabel();
    std::ostringstream oss;
    oss << "task=" << task_orchestrator::task_name(running_task_id)
        << " step=" << (step_index + 1) << "/" << step_total
        << " label=" << step_label
        << " status=running";
    status_text_ = oss.str();
  }

  // 3. 如果executor_结束，发布 feedback，并更新当前status_text_
  // TO REQUEST: 这里只在这打印一次feedback是否有所不妥？
  if (executor_.isFinished()) {
    const core::TaskResult report = executor_.report();
    TaskId done_task_id = TaskId::IDLE;
    const uint8_t done_raw = executor_.activeTaskId();
    toTaskId(done_raw, done_task_id);
    publishFeedback(done_task_id, report.status);
    RCLCPP_INFO(logger_, "[AUTO_NODE] task done status=%u msg=%s",
                static_cast<unsigned>(report.status), report.message.c_str());
    std::fputs("=================== AUTO NODE TASK END =====================\n", stdout);
    std::fflush(stdout);

    std::ostringstream oss;
    oss << "task=" << task_orchestrator::task_name(done_task_id)
        << " step=" << (executor_.currentStepIndex() + 1) << "/" << executor_.totalSteps()
        << " label=" << executor_.currentStepLabel()
        << " status=" << (report.status == core::TaskStatus::Success ? "finished" : "failed");
    if (!report.message.empty()) {
      oss << " reason=" << report.message;
    }
    status_text_ = oss.str();

    executor_.reset();
  }
}

// TO REQUEST: 状态发布写成 定时器？还是 出现问题或者更新的时候？
void AutoNode::statusTick() {
  publishStatus(status_text_);
}

// ============================================================================
//  BI Func 
// ============================================================================

void AutoNode::handleIntent(TaskId task_id) {
  if (executor_.isRunning()) {
    executor_.cancel();
  }

  if (task_id == TaskId::IDLE) {
    // IDLE 状态返回 Running 表明 IDLE 正常运行
    publishFeedback(TaskId::IDLE, core::TaskStatus::Running);
    status_text_ = "task=IDLE status=idle";
    return;
  }

  const auto plan = orchestrator_.plan(task_id);
  if (!plan) {
    publishFeedback(task_id, core::TaskStatus::Failure);
    RCLCPP_WARN(logger_, "[AUTO_NODE] no plan for task=%u",
                static_cast<unsigned>(task_id));
    return;
  }

  executor_.start(*plan);
  publishFeedback(task_id, core::TaskStatus::Running);
}

void AutoNode::publishFeedback(TaskId task_id, core::TaskStatus status) {
  engineer_interfaces::msg::Intent msg;
  msg.stamp = this->now();
  msg.intent_id = static_cast<uint8_t>(task_id);
  msg.intent_finish = static_cast<uint8_t>(status);
  feedback_pub_->publish(msg);
}

void AutoNode::publishStatus(const std::string &text) {
  if (!status_pub_) {
    return;
  }
  std_msgs::msg::String msg;
  msg.data = text;
  status_pub_->publish(msg);
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

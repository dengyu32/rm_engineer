#pragma once

// Rely

//< C++
#include <atomic>
#include <memory>
#include <string>

//< ROS 2
#include <rclcpp/rclcpp.hpp>

//< Engineer Interfaces
#include <engineer_interfaces/msg/intent.hpp>

//< Other Modules
#include "step_executor/step_executor.hpp"
#include "task_orchestrator/task_orchestrator.hpp"
#include "task_step_library/task.hpp"

namespace engineer_auto {

// ============================================================================
//  AutoNodeConfig
// ----------------------------------------------------------------------------
//  - intent_cmd_topic: 订阅的 Intent 命令话题
//  - intent_fb_topic: 发布的 Intent 反馈话题
//  - update_period_ms: 定时器周期，用于定期检查和执行任务
// ============================================================================

struct AutoNodeConfig {
  std::string intent_cmd_topic{"/hfsm/intent_commands"};
  std::string intent_fb_topic{"/hfsm/intent_feedback"};
  int update_period_ms{20};

  static AutoNodeConfig load(rclcpp::Node &node);
};

// ============================================================================
//  AutoNode
// ----------------------------------------------------------------------------
//  - 订阅 Intent 命令，解析并执行对应的任务
//  - 定期检查当前任务状态，执行任务步骤
//  - 发布任务执行反馈
// ============================================================================

class AutoNode : public rclcpp::Node {
public:
  explicit AutoNode(const rclcpp::NodeOptions &options);

private:
  //< ROS 2 Callback
  void intentCallback(engineer_interfaces::msg::Intent::ConstSharedPtr msg);
  
  //< Timer Callback
  void tick();

  void handleIntent(task_step_library::TaskId task_id);
  void publishFeedback(task_step_library::TaskId task_id,
                       task_step_library::TaskFinishCode code);

  static bool toTaskId(uint8_t raw, task_step_library::TaskId &out);

private:
  AutoNodeConfig config_;
  rclcpp::Logger logger_;

  task_orchestrator::TaskOrchestrator orchestrator_;
  step_executor::StepExecutor executor_;

  // 下游状态，内部不能修改，只能通过intentCallback更新
  std::atomic<task_step_library::TaskId> latest_task_id_;
  task_step_library::TaskId applied_task_id_{task_step_library::TaskId::IDLE};

  rclcpp::Subscription<engineer_interfaces::msg::Intent>::SharedPtr intent_sub_;
  rclcpp::Publisher<engineer_interfaces::msg::Intent>::SharedPtr feedback_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace engineer_auto

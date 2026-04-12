#pragma once

// Rely

//< C++
#include <atomic>
#include <memory>
#include <string>

//< ROS 2
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

//< Engineer Interfaces
#include <engineer_interfaces/msg/intent.hpp>

//< Other Modules
#include "auto_node/auto_node_config.hpp"
#include "auto_library/method.hpp"
#include "step_executor/registry_bridge.hpp"
#include "step_executor/step_executor.hpp"
#include "task_orchestrator/task_orchestrator.hpp"
#include "task_orchestrator/protocol.hpp"

namespace engineer_auto {

// ============================================================================
//  AutoNode
// ----------------------------------------------------------------------------
//  - 订阅 Intent 命令，解析并执行对应的任务
//  - 定期检查当前任务状态，执行任务步骤
//  - 发布任务执行反馈
//  - 定时发布当前状态
// ============================================================================

class AutoNode : public rclcpp::Node {
public:
  explicit AutoNode(const rclcpp::NodeOptions &options);

private:
  //< ROS 2 Init
  void initRosInterfaces();

  //< ROS 2 Callback
  void intentCallback(engineer_interfaces::msg::Intent::ConstSharedPtr msg);
  
  //< Timer Callback
  void tick();
  void statusTick();

  //< Built-in Function
  void handleIntent(task_orchestrator::TaskId task_id);
  void publishFeedback(task_orchestrator::TaskId task_id,
                       core::TaskStatus status);
  void publishStatus(const std::string &text);

  static bool toTaskId(uint8_t raw, task_orchestrator::TaskId &out);

private:
  AutoNodeConfig config_;
  rclcpp::Logger logger_;

  core::KindSpecMap kind_specs_{};
  std::shared_ptr<step_executor::RegistryBridge> bridge_;
  task_orchestrator::TaskOrchestrator orchestrator_;
  step_executor::StepExecutor executor_;

  // 下游状态，内部不能修改，只能通过intentCallback更新
  std::atomic<task_orchestrator::TaskId> latest_task_id_;
  task_orchestrator::TaskId applied_task_id_{task_orchestrator::TaskId::IDLE};

  rclcpp::Subscription<engineer_interfaces::msg::Intent>::SharedPtr intent_sub_;
  rclcpp::Publisher<engineer_interfaces::msg::Intent>::SharedPtr feedback_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;

  std::string status_text_{"task=IDLE status=idle"};
};

} // namespace engineer_auto

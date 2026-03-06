#pragma once

// ROS2
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>

// C++
#include <mutex>

// ROS messages
#include <engineer_interfaces/msg/gripper_command.hpp>

#include "top_hfsm_v2/config.hpp"
#include "top_hfsm_v2/core/executor_interfaces.hpp"
#include "top_hfsm_v2/types/command_types.hpp"

namespace top_hfsm_v2::executors {

// ============================================================================
//  GripperControlNode
// ----------------------------------------------------------------------------
//  - ROS2 发布夹爪命令的轻量包装
//  - 维护定时器，按当前目标位置周期发布命令
//  - 提供线程安全的 setGripperPosition 接口
// ============================================================================
class GripperControlNode : public top_hfsm_v2::GripperExecutor {
public:
  // -----------------------------------------------------------------------
  //  Lifecycle
  // -----------------------------------------------------------------------
  GripperControlNode(rclcpp::Node &node, const GripperConfig &config);

  // -----------------------------------------------------------------------
  //  Control
  // -----------------------------------------------------------------------
  void setGripperPosition(GripperCommandType command) override; // 默认为零，也就是打开
  void publish_timer_callback();
private:
  // Node handles
  rclcpp::Node &node_;
  GripperConfig config_;
  rclcpp::Logger logger_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  // Runtime state
  double target_position_{0.0};
  std::mutex target_position_mutex_;
  rclcpp::Publisher<engineer_interfaces::msg::GripperCommand>::SharedPtr
      gripper_cmd_pub_;
};

} // namespace top_hfsm_v2::executors

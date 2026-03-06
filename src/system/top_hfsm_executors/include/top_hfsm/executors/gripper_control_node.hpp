#pragma once

// ROS2
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>

// C++
#include <mutex>
#include <ratio>

// ROS messages
#include <engineer_interfaces/msg/gripper_command.hpp>

#include "top_hfsm/config.hpp"

namespace top_hfsm::executors {

// ============================================================================
//  GripperStatusType
// ----------------------------------------------------------------------------
//  - 夹爪动作状态枚举
// ============================================================================
enum class GripperStatusType {
  OPEN = 0,  // 打开
  CLOSE = 1  // 闭合
};

// ============================================================================
//  GripperControlNode
// ----------------------------------------------------------------------------
//  - ROS2 发布夹爪命令的轻量包装
//  - 维护定时器，按当前状态周期发布命令
//  - 提供线程安全的 setGripperStatus 接口
// ============================================================================
class GripperControlNode {
public:
  // -----------------------------------------------------------------------
  //  Lifecycle
  // -----------------------------------------------------------------------
  GripperControlNode(rclcpp::Node &node, const GripperConfig &config);

  // -----------------------------------------------------------------------
  //  Control
  // -----------------------------------------------------------------------
  void setGripperStatus(GripperStatusType status); // 默认为零，也就是打开
  void publish_timer_callback();
private:
  // Node handles
  rclcpp::Node &node_;
  GripperConfig config_;
  rclcpp::Logger logger_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  // Runtime state
  GripperStatusType gripper_status_{GripperStatusType::OPEN};
  std::mutex gripper_status_mutex_;
  rclcpp::Publisher<engineer_interfaces::msg::GripperCommand>::SharedPtr
      gripper_cmd_pub_;
};

} // namespace top_hfsm::executors

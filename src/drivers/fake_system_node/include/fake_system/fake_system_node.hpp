#pragma once

// ROS2
#include <rclcpp/rclcpp.hpp>

// utils
#include "error_code_utils/error_bus.hpp"

// C++
#include <mutex>
#include <vector>

// ROS messages
#include <engineer_interfaces/msg/hfsm_intent.hpp>
#include <engineer_interfaces/msg/joint.hpp>
#include <engineer_interfaces/msg/joints.hpp>
#include <engineer_interfaces/msg/gripper_command.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "fake_system/config.hpp"

namespace fake_system {

// ============================================================================
//  FakeSystemNode
// ----------------------------------------------------------------------------
//  - 提供伪造关节/夹爪状态的 ROS2 节点，便于上层联调
//  - 依赖 rclcpp 定时器与发布/订阅接口
//  - 订阅关节与夹爪指令，生成 JointState / Joints / HFSMIntent 输出
//  - 维护简易内存态，替代真实硬件反馈
// ============================================================================

class FakeSystemNode : public rclcpp::Node {
public:
  // -----------------------------------------------------------------------
  //  Lifecycle
  // -----------------------------------------------------------------------
  explicit FakeSystemNode(const rclcpp::NodeOptions &options);
  void set_error_bus(const std::shared_ptr<error_code_utils::ErrorBus> &bus);

private:
  // -----------------------------------------------------------------------
  //  ROS interfaces
  // -----------------------------------------------------------------------
  void initRosInterfaces();

  // -----------------------------------------------------------------------
  //  ROS callbacks
  // -----------------------------------------------------------------------
  void execute(const engineer_interfaces::msg::Joints::SharedPtr msg);
  void execute(const engineer_interfaces::msg::GripperCommand::SharedPtr msg);
  void publish_timer_callback();
  void publish_error(const error_code_utils::Error &err) const;

  // Parameters / states
  FakeSystemConfig config_;
  rclcpp::Logger logger_;
  std::vector<double> fake_joint_positions_;
  double fake_gripper_position_{0.0};
  std::mutex fake_mutex_; // 保护仿真关节/夹爪状态

  // ROS interfaces
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;
  rclcpp::Publisher<engineer_interfaces::msg::Joints>::SharedPtr
      joint_states_custom_pub_;
  rclcpp::Publisher<engineer_interfaces::msg::Joints>::SharedPtr
      joint_states_verbose_pub_;
  rclcpp::Publisher<engineer_interfaces::msg::HFSMIntent>::SharedPtr
      hfsm_intent_pub_; // 人为调度（schedule）状态机意图
  rclcpp::Subscription<engineer_interfaces::msg::Joints>::SharedPtr
      joint_cmd_sub_;
  rclcpp::Subscription<engineer_interfaces::msg::GripperCommand>::SharedPtr
      gripper_cmd_sub;

  std::shared_ptr<error_code_utils::ErrorBus> error_bus_;
};
} // namespace fake_system

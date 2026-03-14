#pragma once

// ROS2
#include <rclcpp/rclcpp.hpp>

// C++
#include <array>
#include <mutex>
#include <vector>
#include <atomic>
#include <string>

// ROS messages
#include <engineer_interfaces/msg/intent.hpp>
#include <engineer_interfaces/msg/joint.hpp>
#include <engineer_interfaces/msg/joints.hpp>
#include <engineer_interfaces/msg/gripper.hpp>
#include <engineer_interfaces/msg/slots.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "fake_system/config.hpp"

namespace fake_system {

// ============================================================================
//  FakeSystemNode
// ----------------------------------------------------------------------------
//  - 提供伪造关节/夹爪状态的 ROS2 节点，便于上层联调
//  - 依赖 rclcpp 定时器与发布/订阅接口
//  - 订阅关节与夹爪指令，生成 JointState / Joints 输出
//  - 人为终端发布 IntentStatus，模拟调度状态机意图
//  - 维护简易内存态，替代真实硬件反馈
// ============================================================================

class FakeSystemNode : public rclcpp::Node {
public:
  // -----------------------------------------------------------------------
  //  Lifecycle
  // -----------------------------------------------------------------------
  explicit FakeSystemNode(const rclcpp::NodeOptions &options);

private:
  // -----------------------------------------------------------------------
  //  ROS interfaces
  // -----------------------------------------------------------------------
  void initRosInterfaces();

  // -----------------------------------------------------------------------
  //  ROS callbacks
  // -----------------------------------------------------------------------
  void execute(const engineer_interfaces::msg::Joints::SharedPtr msg);
  void execute(const engineer_interfaces::msg::Gripper::SharedPtr msg);
  void execute(const engineer_interfaces::msg::Slots::SharedPtr msg);
  void intent_feedback_callback(const engineer_interfaces::msg::Intent::SharedPtr msg);
  void publish_timer_callback();
  void publish_slot_states_now();
  void publish_error(int code, const char *name, const std::string &message) const;

  // Parameters / states
  FakeSystemConfig config_;
  rclcpp::Logger logger_;

  std::vector<double> fake_joint_positions_;
  double fake_gripper_position_{0.0};
  std::array<bool, 2> fake_slot_status_{{false, false}};
  std::mutex fake_mutex_; // 保护仿真关节/夹爪状态

  // 动态配置 runtime_config_
  std::atomic<uint8_t> fake_intent_id_{0};

  // 动态回调句柄
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  // ROS interfaces
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;
  rclcpp::Publisher<engineer_interfaces::msg::Joints>::SharedPtr joint_states_custom_pub_;
  rclcpp::Publisher<engineer_interfaces::msg::Joints>::SharedPtr joint_states_verbose_pub_;
  rclcpp::Publisher<engineer_interfaces::msg::Intent>::SharedPtr intent_cmd_pub_; // 人为调度（schedule）状态机意图
  rclcpp::Publisher<engineer_interfaces::msg::Slots>::SharedPtr slot_states_pub_;

  rclcpp::Subscription<engineer_interfaces::msg::Intent>::SharedPtr intent_fb_sub_; // 状态机反馈
  rclcpp::Subscription<engineer_interfaces::msg::Joints>::SharedPtr joint_cmd_sub_;
  rclcpp::Subscription<engineer_interfaces::msg::Gripper>::SharedPtr gripper_cmd_sub;
  rclcpp::Subscription<engineer_interfaces::msg::Slots>::SharedPtr slot_cmd_sub_;
};
} // namespace fake_system

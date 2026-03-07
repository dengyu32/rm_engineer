#pragma once

#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <engineer_interfaces/msg/gripper_command.hpp>

#include "task_step_library/types.hpp"

namespace engineer_auto::gripper_control_node {

struct GripperControlConfig {
  std::string gripper_cmd_topic{"/gripper_commands"};
  int publish_period_ms{20};
  double open_position{0.0};
  double close_position{0.03};

  static GripperControlConfig load(rclcpp::Node &node);
};

class GripperControlNode {
public:
  explicit GripperControlNode(rclcpp::Node &node, const GripperControlConfig &config);

  void setCommand(task_step_library::GripperCommand command);
  void cancel();

private:
  void onTimer();

private:
  rclcpp::Node &node_;
  rclcpp::Logger logger_;
  GripperControlConfig config_;

  rclcpp::Publisher<engineer_interfaces::msg::GripperCommand>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex target_mutex_;
  double target_position_{0.0};
};

} // namespace engineer_auto::gripper_control_node

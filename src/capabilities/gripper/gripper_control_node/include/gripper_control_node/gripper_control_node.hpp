#pragma once

#include <mutex>

#include <rclcpp/rclcpp.hpp>

#include <engineer_interfaces/msg/gripper.hpp>

#include "gripper_control_node/gripper_config.hpp"
#include "gripper_control_node/gripper_types.hpp"
#include "auto_library/command.hpp"
#include "auto_library/execute_result.hpp"

namespace engineer_auto::gripper_control_node {

class GripperControlNode {
public:
  explicit GripperControlNode(rclcpp::Node &node, const GripperPresetConfig &config);

  core::ExecuteResult executeOpen();
  core::ExecuteResult executeClose();
  void setCommand(GripperCommand command);
  void cancel();

private:
  void onTimer();

private:
  rclcpp::Node &node_;
  rclcpp::Logger logger_;
  GripperPresetConfig config_;

  rclcpp::Publisher<engineer_interfaces::msg::Gripper>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex target_mutex_;
  double target_position_{0.0};
};

} // namespace engineer_auto::gripper_control_node

#include "gripper_control_node/gripper_control_node.hpp"

#include <chrono>

namespace engineer_auto::gripper_control_node {

GripperControlNode::GripperControlNode(rclcpp::Node &node,
                                       const GripperPresetConfig &config)
    : node_(node), logger_(node.get_logger()), config_(config) {
  RCLCPP_INFO(logger_, "\n%s", config_.summary().c_str());
  target_position_ = config_.gripper_open_position;
  pub_ = node_.create_publisher<engineer_interfaces::msg::Gripper>(
      config_.gripper_cmd_topic, rclcpp::QoS(10));
  timer_ = node_.create_wall_timer(
      std::chrono::milliseconds(config_.gripper_publish_period_ms),
      std::bind(&GripperControlNode::onTimer, this));

  RCLCPP_INFO(logger_, "[GRIPPER_CONTROL] started topic=%s",
              config_.gripper_cmd_topic.c_str());
}

void GripperControlNode::setCommand(task_step_library::GripperCommand command) {
  std::scoped_lock lock(target_mutex_);
  target_position_ = command == task_step_library::GripperCommand::CLOSE
                         ? config_.gripper_close_position
                         : config_.gripper_open_position;
}

void GripperControlNode::cancel() { setCommand(task_step_library::GripperCommand::OPEN); }

void GripperControlNode::onTimer() {
  double pos = 0.0;
  {
    std::scoped_lock lock(target_mutex_);
    pos = target_position_;
  }

  engineer_interfaces::msg::Gripper msg;
  msg.position = pos;
  pub_->publish(msg);
}

} // namespace engineer_auto::gripper_control_node

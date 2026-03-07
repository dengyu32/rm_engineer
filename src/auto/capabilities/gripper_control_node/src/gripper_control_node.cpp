#include "gripper_control_node/gripper_control_node.hpp"

#include <chrono>

namespace engineer_auto::gripper_control_node {

GripperControlConfig GripperControlConfig::load(rclcpp::Node &node) {
  GripperControlConfig cfg;
  node.declare_parameter("gripper_cmd_topic", cfg.gripper_cmd_topic);
  node.declare_parameter("gripper_publish_period_ms", cfg.publish_period_ms);
  node.declare_parameter("gripper_open_position", cfg.open_position);
  node.declare_parameter("gripper_close_position", cfg.close_position);
  node.get_parameter("gripper_cmd_topic", cfg.gripper_cmd_topic);
  node.get_parameter("gripper_publish_period_ms", cfg.publish_period_ms);
  node.get_parameter("gripper_open_position", cfg.open_position);
  node.get_parameter("gripper_close_position", cfg.close_position);
  return cfg;
}

GripperControlNode::GripperControlNode(rclcpp::Node &node,
                                       const GripperControlConfig &config)
    : node_(node), logger_(node.get_logger()), config_(config) {
  target_position_ = config_.open_position;
  pub_ = node_.create_publisher<engineer_interfaces::msg::GripperCommand>(
      config_.gripper_cmd_topic, rclcpp::QoS(10));
  timer_ = node_.create_wall_timer(
      std::chrono::milliseconds(config_.publish_period_ms),
      std::bind(&GripperControlNode::onTimer, this));

  RCLCPP_INFO(logger_, "[GRIPPER_CONTROL] started topic=%s",
              config_.gripper_cmd_topic.c_str());
}

void GripperControlNode::setCommand(task_step_library::GripperCommand command) {
  std::scoped_lock lock(target_mutex_);
  target_position_ = command == task_step_library::GripperCommand::CLOSE
                         ? config_.close_position
                         : config_.open_position;
}

void GripperControlNode::cancel() { setCommand(task_step_library::GripperCommand::OPEN); }

void GripperControlNode::onTimer() {
  double pos = 0.0;
  {
    std::scoped_lock lock(target_mutex_);
    pos = target_position_;
  }

  engineer_interfaces::msg::GripperCommand msg;
  msg.target_position = pos;
  pub_->publish(msg);
}

} // namespace engineer_auto::gripper_control_node

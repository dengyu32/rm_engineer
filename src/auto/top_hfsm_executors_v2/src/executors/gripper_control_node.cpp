// Executors
#include "top_hfsm_v2/executors/gripper_control_node.hpp"

// ROS messages
#include <engineer_interfaces/msg/detail/gripper_command__struct.hpp>

// C++
#include <mutex>

namespace top_hfsm_v2::executors {

namespace {

constexpr double kGripperOpenPosition = 0.0;
constexpr double kGripperClosePosition = 0.03;

} // namespace

// ============================================================================
// ctor
// ============================================================================

GripperControlNode::GripperControlNode(rclcpp::Node &node, const GripperConfig &config)
    : node_(node), config_(config), logger_(node.get_logger()) {
  gripper_cmd_pub_ =
      node_.create_publisher<engineer_interfaces::msg::GripperCommand>(
          config_.gripper_cmd_topic, rclcpp::QoS(10));
  publish_timer_ = node_.create_wall_timer(
      std::chrono::milliseconds(config_.gripper_publish_period_ms),
      std::bind(&GripperControlNode::publish_timer_callback, this));

  RCLCPP_INFO(logger_, "[GRIPPER_CONTROL] started");
}

// ============================================================================
//  Control
// ============================================================================

void GripperControlNode::setGripperPosition(GripperCommandType command) {
  std::scoped_lock lock(target_position_mutex_);
  target_position_ = (command == GripperCommandType::CLOSE)
                         ? kGripperClosePosition
                         : kGripperOpenPosition;
}

// ============================================================================
//  Timers
// ============================================================================

void GripperControlNode::publish_timer_callback() {
  double target_position_copy = 0.0;
  {
    std::scoped_lock lock(target_position_mutex_);
    target_position_copy = target_position_;
  }

  engineer_interfaces::msg::GripperCommand msg;
  msg.target_position = target_position_copy;
  gripper_cmd_pub_->publish(msg);
}

} // namespace top_hfsm_v2::executors

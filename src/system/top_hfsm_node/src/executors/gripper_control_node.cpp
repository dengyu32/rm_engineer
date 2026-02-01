// Executors
#include "top_hfsm/executors/gripper_control_node.hpp"
#include "param_utils/param_snapshot.hpp"

// ROS messages
#include <engineer_interfaces/msg/detail/gripper_command__struct.hpp>

// C++
#include <mutex>

namespace top_hfsm::executors {

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

  param_utils::LogSnapshot(node_.get_logger(), config_.params_snapshot,
                           "[GRIPPER_CONTROL][param] ");
  RCLCPP_INFO(logger_, "[GRIPPER_CONTROL] started");
}

// ============================================================================
//  Control
// ============================================================================

void GripperControlNode::setGripperStatus(GripperStatusType status) {
  std::scoped_lock lock(gripper_status_mutex_);
  gripper_status_ = status; // 直接覆盖状态，定时器下一次发布即可
}

// ============================================================================
//  Timers
// ============================================================================

void GripperControlNode::publish_timer_callback() {
  GripperStatusType status_copy;
  {
    std::scoped_lock lock(gripper_status_mutex_);
    status_copy = gripper_status_;
  }

  engineer_interfaces::msg::GripperCommand msg;
  msg.gripper_command = (status_copy == GripperStatusType::CLOSE) ? 1 : 0;
  gripper_cmd_pub_->publish(msg);
}

} // namespace top_hfsm::executors

#include "gripper_control_node/gripper_control_node.hpp"

#include <chrono>

namespace engineer_auto::gripper_control_node {

using step_executor::Command;
using step_executor::ErrorCode;
using step_executor::ExecuteResult;
using step_executor::ExecuteStatus;
using step_executor::paramAs;

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

ExecuteResult GripperControlNode::execute(const Command &cmd) {
  ExecuteResult result{};

  const auto *action = paramAs<std::string>(cmd, "action");
  if (!action) {
    result.status = ExecuteStatus::Failed;
    result.error.code = ErrorCode::ValidationError;
    result.error.message = "gripper command missing action";
    result.error.retriable = false;
    return result;
  }

  GripperCommand command = GripperCommand::OPEN;
  if (*action == "open") {
    command = GripperCommand::OPEN;
  } else if (*action == "close") {
    command = GripperCommand::CLOSE;
  } else {
    result.status = ExecuteStatus::Failed;
    result.error.code = ErrorCode::ValidationError;
    result.error.message = "gripper action invalid: " + *action;
    result.error.retriable = false;
    return result;
  }

  setCommand(command);
  result.status = ExecuteStatus::Succeeded;
  return result;
}

void GripperControlNode::setCommand(GripperCommand command) {
  std::scoped_lock lock(target_mutex_);
  target_position_ = command == GripperCommand::CLOSE
                         ? config_.gripper_close_position
                         : config_.gripper_open_position;
}

void GripperControlNode::cancel() { setCommand(GripperCommand::OPEN); }

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

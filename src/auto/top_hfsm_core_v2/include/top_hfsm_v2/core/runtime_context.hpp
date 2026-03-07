#pragma once

#include <rclcpp/rclcpp.hpp>

#include "top_hfsm_v2/core/executor_interfaces.hpp"

namespace top_hfsm_v2 {

struct RuntimeContext {
  ArmExecutor *arm_client{nullptr};
  GripperExecutor *gripper_node{nullptr};
  rclcpp::Logger logger{rclcpp::get_logger("top_hfsm_v2")};
};

} // namespace top_hfsm_v2

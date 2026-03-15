#pragma once

#include <rclcpp/rclcpp.hpp>

#include <sstream>
#include <stdexcept>
#include <string>

#include "params_utils/core/params.hpp"

namespace params_utils {

struct GripperResetConfig {
  std::string gripper_cmd_topic{"/gripper_commands"};
  double gripper_open_position{0.0};
  double gripper_close_position{0.03};

  static void Load(rclcpp::Node &node, GripperResetConfig &cfg) {
    using params_utils::detail::declare_get;
    declare_get(node, "gripper_cmd_topic", cfg.gripper_cmd_topic);
    declare_get(node, "gripper_open_position", cfg.gripper_open_position);
    declare_get(node, "gripper_close_position", cfg.gripper_close_position);
    cfg.validate();
  }

  void validate() const {}

  std::string summary() const {
    std::ostringstream oss;
    oss << " Gripper:\n";
    oss << "   - gripper_cmd_topic        : " << gripper_cmd_topic << "\n";
    oss << "   - gripper_open_position    : " << gripper_open_position << "\n";
    oss << "   - gripper_close_position   : " << gripper_close_position << "\n";
    return oss.str();
  }
};

}  // namespace params_utils

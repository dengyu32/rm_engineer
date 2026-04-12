#pragma once

#include <rclcpp/rclcpp.hpp>

#include <sstream>
#include <string>

#include "robot_config/robot_config.hpp"

namespace engineer_auto::gripper_control_node {

struct GripperPresetConfig : public params_utils::GripperResetConfig {
  int gripper_publish_period_ms{20};

  static GripperPresetConfig load(rclcpp::Node &node) {
    GripperPresetConfig cfg;
    params_utils::GripperResetConfig::Load(node, cfg);
    params_utils::detail::declare_get_checked(
        node, "gripper_publish_period_ms", cfg.gripper_publish_period_ms,
        [](int v) { return v > 0; },
        "must be > 0");
    cfg.validate();
    return cfg;
  }

  void validate() const { params_utils::GripperResetConfig::validate(); }

  std::string summary() const {
    std::ostringstream oss;
    oss << "=============================================================================\n";
    oss << " GripperControl Configuration\n\n";
    oss << " Timing:\n";
    oss << "   - gripper_publish_period_ms : " << gripper_publish_period_ms << "\n\n";
    oss << params_utils::GripperResetConfig::summary();
    oss << "=============================================================================\n";
    return oss.str();
  }
};

} // namespace engineer_auto::gripper_control_node

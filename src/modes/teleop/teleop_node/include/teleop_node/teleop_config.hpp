#pragma once

#include <rclcpp/rclcpp.hpp>

#include <sstream>
#include <string>

#include "robot_config/robot_config.hpp"

namespace engineer_teleop {

// ============================================================================
//  TeleopConfig
// ----------------------------------------------------------------------------
//  - 话题 / MoveIt / Teleop 参数
// ============================================================================

struct TeleopConfig : public params_utils::IntentResetConfig,
                      public params_utils::JointResetConfig,
                      public params_utils::MoveItResetConfig {
  double deadband_rad{1e-4};
  int teleop_intent_id{11};

  static TeleopConfig load(rclcpp::Node &node) {
    TeleopConfig cfg;

    params_utils::IntentResetConfig::Load(node, cfg);
    params_utils::JointResetConfig::Load(node, cfg);
    params_utils::MoveItResetConfig::Load(node, cfg);

    params_utils::detail::declare_get(node, "command_deadband_rad", cfg.deadband_rad);
    params_utils::detail::declare_get_checked(
        node, "teleop_intent_id", cfg.teleop_intent_id,
        [](int v) { return v >= 0 && v <= 255; },
        "must be in [0, 255]");

    cfg.validate();
    return cfg;
  }

  void validate() const {
    params_utils::IntentResetConfig::validate();
    params_utils::JointResetConfig::validate();
    params_utils::MoveItResetConfig::validate();
  }

  std::string summary() const {
    std::ostringstream oss;
    oss << "=============================================================================\n";
    oss << " Teleop Configuration\n\n";
    oss << " Teleop:\n";
    oss << "   - command_deadband_rad : " << deadband_rad << "\n";
    oss << "   - teleop_intent_id     : " << teleop_intent_id << "\n\n";
    oss << params_utils::IntentResetConfig::summary();
    oss << params_utils::JointResetConfig::summary();
    oss << params_utils::MoveItResetConfig::summary();
    oss << "=============================================================================\n";
    return oss.str();
  }
};

} // namespace engineer_teleop

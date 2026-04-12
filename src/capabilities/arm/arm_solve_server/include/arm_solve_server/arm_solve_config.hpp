#pragma once

#include <rclcpp/rclcpp.hpp>

#include <sstream>
#include <string>

#include "robot_config/robot_config.hpp"

namespace arm_solve {

// ============================================================================
//  ArmSolveConfig
// ----------------------------------------------------------------------------
//  - 服务端参数配置，包括：
//  - 动作通信接口名称
//  - JointResetConfig
// ============================================================================

struct ArmSolveConfig : public params_utils::JointResetConfig
{
  std::string arm_action_name{ "move_arm" };

  static ArmSolveConfig Load(rclcpp::Node& node)
  {
    ArmSolveConfig cfg;

    params_utils::JointResetConfig::Load(node, cfg);

    using params_utils::detail::declare_get_checked;

    declare_get_checked(
        node, "arm_action_name", cfg.arm_action_name, [](const std::string v) { return !v.empty(); },
        "must not be empty");

    cfg.validate();
    return cfg;
  }

  void validate() const
  {
    params_utils::JointResetConfig::validate();
  }

  std::string summary() const
  {
    std::ostringstream oss;
    oss << "=========\n";
    oss << " ArmSolveServer Configuration\n\n";
    oss << " Service:\n";
    oss << "   - arm_action_name        : " << arm_action_name << "\n\n";
    oss << params_utils::JointResetConfig::summary();
    oss << "=========\n";
    return oss.str();
  }
};

} // namespace arm_solve

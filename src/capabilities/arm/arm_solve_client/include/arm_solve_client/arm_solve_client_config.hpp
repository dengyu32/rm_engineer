#pragma once

#include <rclcpp/rclcpp.hpp>

#include <sstream>
#include <string>

#include "robot_config/robot_config.hpp"

namespace engineer_auto::arm_solve_client {

// ============================================================================
//  ArmSovleClientConfig
// ----------------------------------------------------------------------------
//  - arm_action_name: ArmMove action 名称
//  - arm_server_wait_ms: action server 等待时间
// ============================================================================

struct ArmSolveClientConfig {
  std::string action_name{"move_arm"};
  int server_wait_ms{200};

  static ArmSolveClientConfig load(rclcpp::Node &node) {
    ArmSolveClientConfig cfg;
    params_utils::detail::declare_get_checked(
        node, "arm_action_name", cfg.action_name,
        [](const std::string &v) { return !v.empty(); },
        "must not be empty");
    params_utils::detail::declare_get_checked(
        node, "arm_server_wait_ms", cfg.server_wait_ms,
        [](int v) { return v >= 0; },
        "must be >= 0");
    cfg.validate();
    return cfg;
  }

  void validate() const {}

  std::string summary() const {
    std::ostringstream oss;
    oss << "=============================================================================\n";
    oss << " ArmSolveClient Configuration\n\n";
    oss << " Action:\n";
    oss << "   - arm_action_name      : " << action_name << "\n";
    oss << "   - arm_server_wait_ms   : " << server_wait_ms << "\n";
    oss << "=============================================================================\n";
    return oss.str();
  }
};

} // namespace engineer_auto::arm_solve_client

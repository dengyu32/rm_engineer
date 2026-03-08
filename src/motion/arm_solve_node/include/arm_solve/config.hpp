// Strongly-typed configuration for ArmSolveServer with manual summary.
#pragma once

#include <rclcpp/rclcpp.hpp>

#include <sstream>
#include <stdexcept>
#include <string>

namespace arm_solve {

struct ArmSolveConfig {
  // MoveIt 规划组名称
  std::string group_name{"engineer_arm"};
  // Action 服务名（通信参数）
  std::string arm_action_name{"move_arm"};
  // 关节状态订阅话题（通信参数）
  std::string joint_states_topic{"/joint_states_verbose"};
  // 关节命令发布话题（通信参数）
  std::string joint_cmd_topic{"/joint_commands"};

  // 延迟初始化 MoveGroup 的等待时长（毫秒）
  int late_init_delay_ms{10};
  // 连续两次规划最小间隔（毫秒）
  int plan_min_interval_ms{0};

  //  API 
  static ArmSolveConfig Load(rclcpp::Node& node);
  void validate() const;
  std::string summary() const;
};

inline ArmSolveConfig ArmSolveConfig::Load(rclcpp::Node& node) {
  ArmSolveConfig cfg;

  //  Small helpers 
  auto declare_get = [&](const std::string& name, auto& value) {
    node.declare_parameter(name, value);
    node.get_parameter(name, value);
  };

  auto declare_get_checked = [&](const std::string& name,
                                 auto& value,
                                 auto&& check,
                                 const char* err_msg) {
    node.declare_parameter(name, value);
    node.get_parameter(name, value);
    if (!check(value)) {
      throw std::runtime_error("ArmSolveConfig: " + name + ": " + err_msg);
    }
  };

  //  Required / validated params 
  declare_get_checked(
      "group_name",
      cfg.group_name,
      [](const std::string& v) { return !v.empty(); },
      "must not be empty");

  declare_get_checked(
      "late_init_delay_ms",
      cfg.late_init_delay_ms,
      [](int v) { return v >= 0 && v <= 5000; },
      "must be in [0, 5000]");

  declare_get_checked(
      "plan_min_interval_ms",
      cfg.plan_min_interval_ms,
      [](int v) { return v >= 0 && v <= 5000; },
      "must be in [0, 5000]");

  //  Usually fixed (no check) 
  declare_get("arm_action_name", cfg.arm_action_name);
  declare_get("joint_states_topic", cfg.joint_states_topic);
  declare_get("joint_cmd_topic", cfg.joint_cmd_topic);

  //  Finalize 
  cfg.validate();  // keep only semantic / cross-field checks
  return cfg;
}

inline void ArmSolveConfig::validate() const {
  // 当前仅保留字段范围校验，组合语义由 solve_core/config.hpp 维护。
}

inline std::string ArmSolveConfig::summary() const {
  std::ostringstream oss;
  oss << "=========\n";
  oss << " ArmSolveServer Configuration\n\n";
  oss << " Service:\n";
  oss << "   - arm_action_name        : " << arm_action_name << "\n\n";
  oss << " Planning:\n";
  oss << "   - group_name            : " << group_name << "\n";
  oss << "   - late_init_delay_ms    : " << late_init_delay_ms << "\n";
  oss << "   - plan_min_interval_ms  : " << plan_min_interval_ms << "\n\n";
  oss << " Topics:\n";
  oss << "   - joint_states_topic    : " << joint_states_topic << "\n";
  oss << "   - joint_cmd_topic       : " << joint_cmd_topic << "\n";
  oss << "=========\n";
  return oss.str();
}

}  // namespace arm_solve

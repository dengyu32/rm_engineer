// Strongly-typed configuration for ArmSolveServer with manual summary.
#pragma once

#include <rclcpp/rclcpp.hpp>

#include <sstream>
#include <stdexcept>
#include <string>

namespace arm_solve {

struct ArmSolveConfig {
  // planning params
  std::string group_name;
  double goal_position_tolerance{1e-3};
  double goal_orientation_tolerance{1e-3};
  double planning_time{2.0};
  int num_planning_attempts{5};

  //  ROS names / topics (usually fixed) 
  std::string arm_action_name{"move_arm"};
  std::string joint_states_topic{"/joint_states_verbose"};
  std::string joint_cmd_topic{"/joint_commands"};

  //  Scaling / throttling 
  double max_velocity_scaling{0.6};
  double max_acc_scaling{0.6};

  int late_init_delay_ms{10};
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
      "goal_position_tolerance",
      cfg.goal_position_tolerance,
      [](double v) { return v >= 0.0 && v <= 1.0; },
      "must be in [0, 1]");

  declare_get_checked(
      "goal_orientation_tolerance",
      cfg.goal_orientation_tolerance,
      [](double v) { return v >= 0.0 && v <= 1.0; },
      "must be in [0, 1]");

  declare_get_checked(
      "planning_time",
      cfg.planning_time,
      [](double v) { return v >= 0.0 && v <= 30.0; },
      "must be in [0, 30]");

  declare_get_checked(
      "num_planning_attempts",
      cfg.num_planning_attempts,
      [](int v) { return v >= 1 && v <= 50; },
      "must be in [1, 50]");

  declare_get_checked(
      "max_velocity_scaling",
      cfg.max_velocity_scaling,
      [](double v) { return v >= 0.0 && v <= 1.0; },
      "must be in [0, 1]");

  declare_get_checked(
      "max_acc_scaling",
      cfg.max_acc_scaling,
      [](double v) { return v >= 0.0 && v <= 1.0; },
      "must be in [0, 1]");

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
  // 只保留“组合语义”检查
  if (plan_min_interval_ms > 0 && planning_time <= 0.0) {
    throw std::runtime_error(
        "ArmSolveConfig: plan_min_interval_ms > 0 requires planning_time > 0");
  }
}

inline std::string ArmSolveConfig::summary() const {
  std::ostringstream oss;
  oss << "=========\n";
  oss << " ArmSolveServer Configuration\n\n";
  oss << " Service:\n";
  oss << "   - arm_action_name        : " << arm_action_name << "\n\n";
  oss << " Planning:\n";
  oss << "   - group_name            : " << group_name << "\n";
  oss << "   - planning_time         : " << planning_time << "\n";
  oss << "   - num_planning_attempts : " << num_planning_attempts << "\n";
  oss << "   - goal_position_tolerance : " << goal_position_tolerance << "\n";
  oss << "   - goal_orientation_tolerance : " << goal_orientation_tolerance << "\n\n";
  oss << " Execution:\n";
  oss << "   - max_velocity_scaling  : " << max_velocity_scaling << "\n";
  oss << "   - max_acc_scaling       : " << max_acc_scaling << "\n";
  oss << "   - late_init_delay_ms    : " << late_init_delay_ms << "\n";
  oss << "   - plan_min_interval_ms  : " << plan_min_interval_ms << "\n\n";
  oss << " Topics:\n";
  oss << "   - joint_states_topic    : " << joint_states_topic << "\n";
  oss << "   - joint_cmd_topic       : " << joint_cmd_topic << "\n";
  oss << "=========\n";
  return oss.str();
}

}  // namespace arm_solve

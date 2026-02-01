// Strongly-typed configuration for ArmSolveServer with validation + summary.
#pragma once

#include <rclcpp/rclcpp.hpp>

#include <param_utils/param_snapshot.hpp>

#include <stdexcept>
#include <string>
#include <vector>

namespace arm_solve {

struct ArmSolveConfig {
  // Parameters
  std::string group_name;
  double goal_position_tolerance{1e-3};
  double goal_orientation_tolerance{1e-3};
  double planning_time{2.0};
  int num_planning_attempts{5};

  std::string arm_action_name{"move_arm"};
  std::string joint_states_topic{"/joint_states_verbose"};
  std::string joint_cmd_topic{"/joint_commands"};

  double max_velocity_scaling{0.6};
  double max_acc_scaling{0.6};

  int late_init_delay_ms{10};
  int plan_min_interval_ms{0};

  std::vector<param_utils::ParamKV> params_snapshot{};

  static ArmSolveConfig Load(rclcpp::Node &node) {
    ArmSolveConfig cfg;
    auto declare_get = [&](const std::string& name, auto& value) {
      node.declare_parameter(name, value);
      node.get_parameter(name, value);
    };

    declare_get("group_name", cfg.group_name);
    declare_get("goal_position_tolerance", cfg.goal_position_tolerance);
    declare_get("goal_orientation_tolerance", cfg.goal_orientation_tolerance);
    declare_get("planning_time", cfg.planning_time);
    declare_get("num_planning_attempts", cfg.num_planning_attempts);
    declare_get("arm_action_name", cfg.arm_action_name);
    declare_get("joint_states_topic", cfg.joint_states_topic);
    declare_get("joint_cmd_topic", cfg.joint_cmd_topic);
    declare_get("max_velocity_scaling", cfg.max_velocity_scaling);
    declare_get("max_acc_scaling", cfg.max_acc_scaling);
    declare_get("late_init_delay_ms", cfg.late_init_delay_ms);
    declare_get("plan_min_interval_ms", cfg.plan_min_interval_ms);

    if (cfg.group_name.empty()) {
      throw std::runtime_error("ArmSolveConfig: group_name must not be empty");
    }
    if (cfg.arm_action_name.empty()) {
      throw std::runtime_error("ArmSolveConfig: arm_action_name must not be empty");
    }
    if (cfg.joint_states_topic.empty()) {
      throw std::runtime_error("ArmSolveConfig: joint_states_topic must not be empty");
    }
    if (cfg.joint_cmd_topic.empty()) {
      throw std::runtime_error("ArmSolveConfig: joint_cmd_topic must not be empty");
    }
    if (cfg.goal_position_tolerance < 0.0 || cfg.goal_position_tolerance > 1.0) {
      throw std::runtime_error("ArmSolveConfig: goal_position_tolerance must be in [0, 1]");
    }
    if (cfg.goal_orientation_tolerance < 0.0 || cfg.goal_orientation_tolerance > 1.0) {
      throw std::runtime_error("ArmSolveConfig: goal_orientation_tolerance must be in [0, 1]");
    }
    if (cfg.planning_time < 0.0 || cfg.planning_time > 30.0) {
      throw std::runtime_error("ArmSolveConfig: planning_time must be in [0, 30]");
    }
    if (cfg.num_planning_attempts < 1 || cfg.num_planning_attempts > 50) {
      throw std::runtime_error("ArmSolveConfig: num_planning_attempts must be in [1, 50]");
    }
    if (cfg.max_velocity_scaling < 0.0 || cfg.max_velocity_scaling > 1.0) {
      throw std::runtime_error("ArmSolveConfig: max_velocity_scaling must be in [0, 1]");
    }
    if (cfg.max_acc_scaling < 0.0 || cfg.max_acc_scaling > 1.0) {
      throw std::runtime_error("ArmSolveConfig: max_acc_scaling must be in [0, 1]");
    }
    if (cfg.late_init_delay_ms < 0 || cfg.late_init_delay_ms > 5000) {
      throw std::runtime_error("ArmSolveConfig: late_init_delay_ms must be in [0, 5000]");
    }
    if (cfg.plan_min_interval_ms < 0 || cfg.plan_min_interval_ms > 5000) {
      throw std::runtime_error("ArmSolveConfig: plan_min_interval_ms must be in [0, 5000]");
    }

    cfg.params_snapshot = param_utils::CollectAllParams(node);

    return cfg;
  }
};

} // namespace arm_solve

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
  // 末端位置容差（米）
  double goal_position_tolerance{1e-3};
  // 末端姿态容差（弧度）
  double goal_orientation_tolerance{1e-3};
  // 单次规划超时时间（秒）
  double planning_time{2.0};
  // MoveIt 采样规划尝试次数
  int num_planning_attempts{5};

  // Action 服务名（通信参数）
  std::string arm_action_name{"move_arm"};
  // 关节状态订阅话题（通信参数）
  std::string joint_states_topic{"/joint_states_verbose"};
  // 关节命令发布话题（通信参数）
  std::string joint_cmd_topic{"/joint_commands"};

  // 轨迹速度缩放系数 [0,1]
  double max_velocity_scaling{0.6};
  // 轨迹加速度缩放系数 [0,1]
  double max_acc_scaling{0.6};

  // 延迟初始化 MoveGroup 的等待时长（毫秒）
  int late_init_delay_ms{10};
  // 连续两次规划最小间隔（毫秒）
  int plan_min_interval_ms{0};

  // 笛卡尔直线规划路点数
  int cartesian_num_waypoints{50};
  // 是否启用“方向+步长”采样
  bool cartesian_use_directional_sampling{false};
  // 笛卡尔采样步长（米）
  double cartesian_sample_step_m{0.0};
  // 笛卡尔采样方向 X 分量
  double cartesian_direction_x{0.0};
  // 笛卡尔采样方向 Y 分量
  double cartesian_direction_y{0.0};
  // 笛卡尔采样方向 Z 分量
  double cartesian_direction_z{0.0};

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

  declare_get_checked(
      "cartesian_num_waypoints",
      cfg.cartesian_num_waypoints,
      [](int v) { return v >= 1 && v <= 10000; },
      "must be in [1, 10000]");

  declare_get("cartesian_use_directional_sampling", cfg.cartesian_use_directional_sampling);

  declare_get_checked(
      "cartesian_sample_step_m",
      cfg.cartesian_sample_step_m,
      [](double v) { return v >= 0.0 && v <= 10.0; },
      "must be in [0, 10]");

  declare_get_checked(
      "cartesian_direction_x",
      cfg.cartesian_direction_x,
      [](double v) { return v >= -1e6 && v <= 1e6; },
      "must be finite");
  declare_get_checked(
      "cartesian_direction_y",
      cfg.cartesian_direction_y,
      [](double v) { return v >= -1e6 && v <= 1e6; },
      "must be finite");
  declare_get_checked(
      "cartesian_direction_z",
      cfg.cartesian_direction_z,
      [](double v) { return v >= -1e6 && v <= 1e6; },
      "must be finite");

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
  if (cartesian_use_directional_sampling && cartesian_sample_step_m <= 0.0) {
    throw std::runtime_error(
        "ArmSolveConfig: cartesian_sample_step_m must be > 0 when cartesian_use_directional_sampling=true");
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
  oss << " Cartesian:\n";
  oss << "   - cartesian_num_waypoints           : " << cartesian_num_waypoints << "\n";
  oss << "   - cartesian_use_directional_sampling: "
      << (cartesian_use_directional_sampling ? "true" : "false") << "\n";
  oss << "   - cartesian_sample_step_m           : " << cartesian_sample_step_m << "\n";
  oss << "   - cartesian_direction_xyz           : ["
      << cartesian_direction_x << ", "
      << cartesian_direction_y << ", "
      << cartesian_direction_z << "]\n\n";
  oss << " Topics:\n";
  oss << "   - joint_states_topic    : " << joint_states_topic << "\n";
  oss << "   - joint_cmd_topic       : " << joint_cmd_topic << "\n";
  oss << "=========\n";
  return oss.str();
}

}  // namespace arm_solve

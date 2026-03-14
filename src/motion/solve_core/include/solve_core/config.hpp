// Strongly-typed configuration for SolveCore with manual summary.
#pragma once

#include <sstream>
#include <stdexcept>
#include <string>

namespace solve_core {

struct SolveCoreConfig {
  // MoveIt planning defaults（不走外部请求）
  double goal_position_tolerance{1e-3};
  double goal_orientation_tolerance{1e-3};
  double planning_time{2.0};
  int num_planning_attempts{5};
  double max_velocity_scaling{1.0};
  double max_acc_scaling{1.0};

  // 是否开启目标姿态 roll 采样
  bool limit_enable_target_pose_sampling{false};
  // roll 采样点数量
  int limit_roll_samples{5};
  // roll 采样范围（±弧度）
  double limit_roll_range_rad{0.35};
  // IK 候选过滤后保留前 K 个（-1 表示不过滤）
  int limit_top_k_after_ik{3};
  // 姿态偏差代价权重
  double limit_orientation_weight{1.0};
  // IK 距离代价权重
  double limit_ik_distance_weight{1.0};
  // 关节运动代价权重
  double limit_joint_motion_weight{1.0};

  // 笛卡尔直线规划路点数
  int cartesian_num_waypoints{50};
  // 是否启用“方向+步长”模式
  bool cartesian_use_directional_sampling{true};
  // 每个路点的位移步长（米）
  double cartesian_sample_step_m{0.002};

  // JOINTS 模式单步最大关节变化（弧度）
  double joints_max_step_rad{0.05};

  void validate() const;
  std::string summary() const;
};

inline void SolveCoreConfig::validate() const {
  if (goal_position_tolerance < 0.0 || goal_position_tolerance > 1.0) {
    throw std::runtime_error("SolveCoreConfig: goal_position_tolerance must be in [0, 1]");
  }
  if (goal_orientation_tolerance < 0.0 || goal_orientation_tolerance > 1.0) {
    throw std::runtime_error("SolveCoreConfig: goal_orientation_tolerance must be in [0, 1]");
  }
  if (planning_time < 0.0 || planning_time > 30.0) {
    throw std::runtime_error("SolveCoreConfig: planning_time must be in [0, 30]");
  }
  if (num_planning_attempts < 1 || num_planning_attempts > 50) {
    throw std::runtime_error("SolveCoreConfig: num_planning_attempts must be in [1, 50]");
  }
  if (max_velocity_scaling < 0.0 || max_velocity_scaling > 1.0) {
    throw std::runtime_error("SolveCoreConfig: max_velocity_scaling must be in [0, 1]");
  }
  if (max_acc_scaling < 0.0 || max_acc_scaling > 1.0) {
    throw std::runtime_error("SolveCoreConfig: max_acc_scaling must be in [0, 1]");
  }
  if (limit_top_k_after_ik == 0) {
    throw std::runtime_error(
        "SolveCoreConfig: limit_top_k_after_ik must not be 0 (use -1 for no filter)");
  }
  if (cartesian_num_waypoints <= 0) {
    throw std::runtime_error("SolveCoreConfig: cartesian_num_waypoints must be > 0");
  }
  if (cartesian_use_directional_sampling && cartesian_sample_step_m <= 0.0) {
    throw std::runtime_error(
        "SolveCoreConfig: cartesian_sample_step_m must be > 0 when directional sampling is enabled");
  }
}

inline std::string SolveCoreConfig::summary() const {
  std::ostringstream oss;
  oss << "=========\n";
  oss << " SolveCore Configuration\n\n";
  oss << " MoveIt Defaults:\n";
  oss << "   - goal_position_tolerance           : " << goal_position_tolerance << "\n";
  oss << "   - goal_orientation_tolerance        : " << goal_orientation_tolerance << "\n";
  oss << "   - planning_time                     : " << planning_time << "\n";
  oss << "   - num_planning_attempts             : " << num_planning_attempts << "\n";
  oss << "   - max_velocity_scaling              : " << max_velocity_scaling << "\n";
  oss << "   - max_acc_scaling                   : " << max_acc_scaling << "\n\n";
  oss << " Limit Sampling:\n";
  oss << "   - limit_enable_target_pose_sampling : "
      << (limit_enable_target_pose_sampling ? "true" : "false") << "\n";
  oss << "   - limit_roll_samples                : " << limit_roll_samples << "\n";
  oss << "   - limit_roll_range_rad              : " << limit_roll_range_rad << "\n";
  oss << "   - limit_top_k_after_ik              : " << limit_top_k_after_ik << "\n";
  oss << "   - limit_orientation_weight          : " << limit_orientation_weight << "\n";
  oss << "   - limit_ik_distance_weight          : " << limit_ik_distance_weight << "\n";
  oss << "   - limit_joint_motion_weight         : " << limit_joint_motion_weight << "\n\n";
  oss << " Planners:\n";
  oss << "   - cartesian_num_waypoints           : " << cartesian_num_waypoints << "\n";
  oss << "   - cartesian_use_directional_sampling: "
      << (cartesian_use_directional_sampling ? "true" : "false") << "\n";
  oss << "   - cartesian_sample_step_m           : " << cartesian_sample_step_m << "\n";
  oss << "   - joints_max_step_rad               : " << joints_max_step_rad << "\n";
  oss << "=========\n";
  return oss.str();
}

} // namespace solve_core

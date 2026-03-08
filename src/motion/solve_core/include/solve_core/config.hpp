// Strongly-typed configuration for SolveCore with manual summary.
#pragma once

#include <sstream>
#include <stdexcept>
#include <string>

namespace solve_core {

struct CostOptions {
  double continuity_weight{1.0};           // 连续性权重
  double condition_weight{0.5};            // 雅可比条件数权重
  double hard_condition_threshold{500.0};  // 雅可比条件数阈值
  double hard_penalty{1e6};                // 超阈值惩罚
};

struct IKOptions {
  int max_attempts{50};    // 总尝试次数
  int max_solutions{10};   // 最多收集多少个解
  double timeout{0.02};    // setFromIK timeout (seconds)
  double noise_sigma{0.2}; // 高斯噪声标准差
  double dedup_eps{1e-3};  // 去重阈值（L∞）
};

struct StraightPlannerOptions {
  int num_waypoints{50};               // 直线离散路点数
  bool use_directional_sampling{false}; // 是否启用“方向+步长”模式
  double sample_step_m{0.0};           // 每步位移（米）
  double direction_x{0.0};             // 方向向量 X
  double direction_y{0.0};             // 方向向量 Y
  double direction_z{0.0};             // 方向向量 Z
};

struct SolveCoreConfig {
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
  bool cartesian_use_directional_sampling{false};
  // 每个路点的位移步长（米）
  double cartesian_sample_step_m{0.0};
  // 方向向量 X
  double cartesian_direction_x{0.0};
  // 方向向量 Y
  double cartesian_direction_y{0.0};
  // 方向向量 Z
  double cartesian_direction_z{0.0};

  // JOINTS 模式单步最大关节变化（弧度）
  double joints_max_step_rad{0.05};

  void validate() const;
  std::string summary() const;
};

inline void SolveCoreConfig::validate() const {
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
  oss << "   - cartesian_direction_xyz           : ["
      << cartesian_direction_x << ", "
      << cartesian_direction_y << ", "
      << cartesian_direction_z << "]\n";
  oss << "   - joints_max_step_rad               : " << joints_max_step_rad << "\n";
  oss << "=========\n";
  return oss.str();
}

} // namespace solve_core

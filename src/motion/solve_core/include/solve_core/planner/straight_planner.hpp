#pragma once

#include <optional>
#include <vector>
#include <Eigen/Geometry>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include "solve_core/solve_core.hpp"
#include "solve_core/calculate_tools/cost_func.hpp"

/*
  直线路径规划器
*/
namespace solve_core {

struct StraightPlannerOptions {
  int num_waypoints{50};               // 直线离散路点数
  bool use_directional_sampling{true}; // 是否启用“方向+步长”模式
  double sample_step_m{0.01};           // 每步位移（米）
  double direction_x{0.0};             // 方向向量 X
  double direction_y{0.0};             // 方向向量 Y
  double direction_z{0.0};             // 方向向量 Z
  double joint_interp_step_rad{0.05};  // 相邻点关节线性插值步长（弧度，<=0 表示不插值）
};

class StraightPlanner {
public:
  StraightPlanner(const moveit::core::RobotModelConstPtr& model,
                   const std::string& group_name,
                   const std::string& ee_link);

  std::optional<Trajectory> plan(
      moveit::core::RobotState& start_state,
      const Eigen::Isometry3d& target_pose,
      const StraightPlannerOptions& opt,
      const CostOptions& cost_opt,
      std::vector<std::vector<double>>* joint_path_out = nullptr);

private:
  moveit::core::RobotModelConstPtr robot_model_;
  std::string group_name_;
  std::string ee_link_;
};

} // namespace solve_core

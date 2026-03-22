#pragma once

#include <Eigen/Geometry>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <optional>
#include <vector>

#include "solve_core/adapter.hpp"
#include "solve_core/calculate_tools/cost_func.hpp"

/*
  直线路径规划器
*/
namespace solve_core {

struct StraightPlannerConfigs {
  bool use_directional_sampling{true}; // 是否启用“方向+步长”模式
  double sample_step_m{0.005};          // 每步位移（米）
  double path_length_m{0.0};           // 直线路径总长度（米）
  double direction_x{0.0};             // 方向向量 X
  double direction_y{0.0};             // 方向向量 Y
  double direction_z{0.0};             // 方向向量 Z
  double joint_interp_step_rad{
      0.05}; // 相邻点关节线性插值步长（弧度，<=0 表示不插值）
};

bool buildStraightPlannerConfigs(const Eigen::Isometry3d &start_pose,
                                 const Eigen::Isometry3d &target_pose,
                                 const std::array<double, 3> &direction,
                                 double target_length,
                                 StraightPlannerConfigs &out,
                                 std::string &err);

class StraightPlanner {
public:
  StraightPlanner(const moveit::core::RobotModelConstPtr &model,
                  const std::string &group_name, const std::string &ee_link);

  std::optional<Trajectory>
  plan(moveit::core::RobotState &start_state,
       const Eigen::Isometry3d &target_pose,
       const StraightPlannerConfigs &configs, const CostOptions &cost_opt,
       std::vector<std::vector<double>> *joint_path_out = nullptr);

private:
  moveit::core::RobotModelConstPtr robot_model_;
  std::string group_name_;
  std::string ee_link_;
};

} // namespace solve_core

#pragma once

#include <Eigen/Geometry>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <optional>
#include <string>
#include <vector>

#include "calculator/cost_func.hpp"
#include "collision/self_collision_detector.hpp"
#include "types/types.hpp"

/*
  直线路径规划器
*/
namespace planner {
using types::Pose;
using types::Trajectory;

struct StraightPlannerSettings {
  double sample_step_m{0.005};             // 每步位移（米）
  double alignment_dot_threshold{0.98};    // 允许进入直线拔出的最小共线程度
  double max_joint_jump_rad{0.15};         // 相邻路点单关节最大允许跳变
  double max_rotation_rad{0.35};        // 相对起始姿态允许变化的 rotation 幅度
  int rotation_samples{7};                    // 5DoF 直线规划时的 rotation 采样数
  std::string reference_link{"base_link"}; // target_vector 所在参考系
};

class StraightPlanner {
public:
  StraightPlanner(const moveit::core::RobotModelConstPtr &model,
                  const std::string &group_name, const std::string &ee_link,
                  const StraightPlannerSettings &settings,
                  const collision::SelfCollisionDetector
                      *self_collision_detector = nullptr);

  std::optional<Trajectory> plan(moveit::core::RobotState &start_state,
                                 Eigen::Vector3d &target_vector,
                                 double target_length,
                                 const calculator::CostOptions &cost_opt);

  std::optional<Trajectory> plan_with_5dof_constrain(moveit::core::RobotState &start_state,
                                 Eigen::Vector3d &target_vector,
                                 double target_length,
                                 const calculator::CostOptions &cost_opt);

private:
  moveit::core::RobotModelConstPtr robot_model_;
  std::string group_name_;
  std::string ee_link_;
  StraightPlannerSettings settings_;
  const collision::SelfCollisionDetector *self_collision_detector_{nullptr};
};

} // namespace planner

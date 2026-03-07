#pragma once

#include <optional>
#include <vector>
#include <Eigen/Geometry>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include "solve_core/solve_core.hpp"
#include "solve_core/config.hpp"

/*
  直线路径规划器
*/
namespace solve_core {

class StraightPlanner {
public:
  StraightPlanner(const moveit::core::RobotModelConstPtr& model,
                   const std::string& group_name,
                   const std::string& ee_link);

  std::optional<Trajectory> plan(
      moveit::core::RobotState& start_state,
      const Eigen::Isometry3d& target_pose,
      const StraightPlannerOptions& opt,
      std::vector<std::vector<double>>* joint_path_out = nullptr);

private:
  moveit::core::RobotModelConstPtr robot_model_;
  std::string group_name_;
  std::string ee_link_;
};

} // namespace solve_core
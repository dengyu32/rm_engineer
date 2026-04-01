#pragma once

#include <Eigen/Geometry>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <optional>
#include <vector>


#include "calculator/sample.hpp"
#include "types/types.hpp"

/*
    约束规划器
    对已知目标姿态的任务进行有约束的采样式规划
    根据路径平滑度和雅可比条件数设计代价函数，在采样的基础上用 DP
    找到一条代价最小的路径
*/
namespace planner
{
using types::Pose;
using types::Trajectory;

class LimitPlanner
{
public:
  LimitPlanner(std::shared_ptr<MoveItAdapter> adapter);

  std::optional<Trajectory> plan(const moveit::core::JointModelGroup* jmg_, const std::string ee_link_,
                                 moveit::core::RobotState& start_state, const Eigen::Isometry3d& target_pose,
                                 const SamplingConfigs& sam_configs, std::string& err,
                                 const struct PlannerConfigs& planner_configs,
                                 std::vector<std::vector<double>>* joint_path_out = nullptr);

private:
  std::shared_ptr<MoveItAdapter> adapter_;
};
}  // namespace planner

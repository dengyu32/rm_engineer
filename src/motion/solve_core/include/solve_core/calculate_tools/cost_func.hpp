#pragma once

#include <vector>

#include "solve_core/config.hpp"
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

/*
    规划层的代价函数设计
    通过计算雅可比矩阵条件数和关节空间距离来评估路径点之间的连续性和可行性
*/

namespace solve_core {

class CostFunc {
public:
  CostFunc(const moveit::core::RobotState &reference_state,
           const moveit::core::JointModelGroup *joint_model_group,
           const moveit::core::LinkModel *link_model,
           const CostOptions &options = {});

  double compute(const std::vector<double> &q_from,
                 const std::vector<double> &q_to) const;

private:
  moveit::core::RobotState reference_state_;
  const moveit::core::JointModelGroup *joint_model_group_;
  const moveit::core::LinkModel *link_model_;
  CostOptions options_;
};

} // namespace solve_core

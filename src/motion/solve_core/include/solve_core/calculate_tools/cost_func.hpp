#pragma once

#include <vector>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

/*
    规划层的代价函数设计
    通过计算雅可比矩阵条件数和关节空间距离来评估路径点之间的连续性和可行性
*/

namespace solve_core {

struct CostOptions {
  double continuity_weight{0.3};            // 连续性权重
  double condition_weight{0.5};             // 雅可比条件数权重（放宽）
  double hard_condition_threshold{2000.0};  // 雅可比条件数阈值（放宽）
  double hard_penalty{1e2};                 // 超阈值惩罚（放宽）
};

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

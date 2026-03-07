#include "solve_core/calculate_tools/cost_func.hpp"

#include <cmath>
#include <limits>

#include <Eigen/SVD>
#include <moveit/robot_state/robot_state.h>

namespace solve_core {
namespace {
//雅可比矩阵条件数计算代价
double jacobian_condition(moveit::core::RobotState &st,
                          const moveit::core::JointModelGroup *jmg,
                          const moveit::core::LinkModel *link) {
  Eigen::MatrixXd J;
  st.getJacobian(jmg, link, Eigen::Vector3d::Zero(), J);
  if (J.size() == 0) {
    return 1e9;
  }

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(J);
  auto singular_values = svd.singularValues();  //奇异值
  if (singular_values.size() == 0) {    //奇异值为空，说明雅可比矩阵无效，视为大代价
    return 1e9;
  }

  const double smin = singular_values.minCoeff();
  const double smax = singular_values.maxCoeff();
  if (smin < 1e-8) {
    return 1e9;
  }
  return smax / smin;   //返回条件数
}

//关节空间距离计算连续性代价
double joint_distance_l2(const std::vector<double> &a,
                         const std::vector<double> &b) {
  if (a.size() != b.size()) {
    return std::numeric_limits<double>::infinity();   //尺寸不匹配视为无穷大代价
  }
  double sum = 0.0;
  for (std::size_t i = 0; i < a.size(); ++i) {
    const double d = a[i] - b[i];
    sum += d * d;
  }
  return std::sqrt(sum);    //返回欧几里得距离
}

} // namespace

CostFunc::CostFunc(const moveit::core::RobotState &reference_state,
                   const moveit::core::JointModelGroup *joint_model_group,
                   const moveit::core::LinkModel *link_model,
                   const CostOptions &options)
    : reference_state_(reference_state),
      joint_model_group_(joint_model_group),
      link_model_(link_model),
      options_(options) {}

double CostFunc::compute(const std::vector<double> &q_from,
                         const std::vector<double> &q_to) const {
  const double continuity_cost = joint_distance_l2(q_from, q_to);

  moveit::core::RobotState st_q(reference_state_);
  st_q.setJointGroupPositions(joint_model_group_, q_to);
  st_q.update();

  const double cond = jacobian_condition(st_q, joint_model_group_, link_model_);
  const double cond_penalty =
      (cond > options_.hard_condition_threshold)
          ? options_.hard_penalty
          : (cond * options_.condition_weight);

  return options_.continuity_weight * continuity_cost + cond_penalty;
}

} // namespace solve_core

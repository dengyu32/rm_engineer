#include "executor/test.hpp"

#include <cmath>

namespace solve_executor
{

bool set_vector_with_current(const moveit::core::RobotState& robot_state, const moveit::core::LinkModel* ref_link,
                             const moveit::core::LinkModel* ee_link, Eigen::Vector3d& target_vector, std::string& err)
{
  constexpr double kEps = 1e-9;

  if (!ref_link || !ee_link)
  {
    err = "Reference link or end effector link is null";
    return false;
  }

  const Eigen::Isometry3d T_ref = robot_state.getGlobalLinkTransform(ref_link);
  const Eigen::Isometry3d T_ee = robot_state.getGlobalLinkTransform(ee_link);
  const Eigen::Matrix3d R_ref_ee = T_ref.linear().transpose() * T_ee.linear();
  const Eigen::Vector3d ee_x_in_ref = R_ref_ee.col(0);
  const double norm = ee_x_in_ref.norm();
  if (!std::isfinite(norm) || norm <= kEps)
  {
    err = "Current end effector x-axis is invalid";
    return false;
  }

  target_vector = ee_x_in_ref.normalized();
  return true;
}

}  // namespace solve_executor

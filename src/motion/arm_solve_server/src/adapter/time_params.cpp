#pragma once

/*
    时间参数化，基于moveit的轻量级代码实现
*/

#include "executor.hpp"
namespace arm_solve {

namespace solve_executor {

bool SolveExecutor::time_parameterize_trajectory(
    const moveit::core::RobotState &start_state,
    const std::string              &group_name,
    solve_core::Trajectory         &traj,
    double                          velocity_scaling,
    double                          accel_scaling) {
  if (!move_group_) {
    RCLCPP_ERROR(get_logger(), "MoveGroup not ready");
    return false;
  }
  const auto model = robot_model();
  if (!model) {
    RCLCPP_ERROR(get_logger(), "RobotModel not ready");
    return false;
  }
  const auto *jmg = model->getJointModelGroup(group_name);
  if (!jmg) {
    RCLCPP_ERROR(get_logger(), "JointModelGroup not found");
    return false;
  }
  if (traj.points.empty()) {
    return true;
  }

  robot_trajectory::RobotTrajectory rt(model, group_name);
  moveit::core::RobotState          state(start_state);

  for (const auto &pt : traj.points) {
    if (pt.positions.size() < jmg->getVariableCount()) {
      RCLCPP_ERROR(get_logger(), "Trajectory point size mismatch");
      return false;
    }
    state.setJointGroupPositions(jmg, pt.positions);
    state.update();
    rt.addSuffixWayPoint(state, 0.0);
  }

  trajectory_processing::TimeOptimalTrajectoryGeneration totg;
  const double v_scale = std::clamp(velocity_scaling, 0.01, 1.0);
  const double a_scale = std::clamp(accel_scaling, 0.01, 1.0);
  if (!totg.computeTimeStamps(rt, v_scale, a_scale)) {
    RCLCPP_ERROR(get_logger(), "Time parameterization failed");
    return false;
  }

  traj.joint_names = jmg->getVariableNames();
  traj.points.clear();
  traj.points.reserve(rt.getWayPointCount());
  for (std::size_t i = 0; i < rt.getWayPointCount(); ++i) {
    const moveit::core::RobotState &st = rt.getWayPoint(i);
    solve_core::TrajectoryPoint     p;
    st.copyJointGroupPositions(jmg, p.positions);
    st.copyJointGroupVelocities(jmg, p.velocities);
    p.time_from_start = rt.getWayPointDurationFromStart(i);
    traj.points.push_back(std::move(p));
  }
  return true;
}
} // namespace solve_executor
} // namespace arm_solve
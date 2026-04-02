// #pragma once

// /*
//     时间参数化，基于moveit的轻量级代码实现
// */

// #include "executor.hpp"

// namespace solve_executor
// {

// bool SolveExecutor::time_parameterize_trajectory(const moveit::core::RobotState& start_state,
//                                                  const std::string& group_name, solve_executor::Trajectory& traj,
//                                                  double velocity_scaling, double accel_scaling)
// {
//   if (!move_group_)
//   {
//     RCLCPP_ERROR(get_logger(), "MoveGroup not ready");
//     return false;
//   }
//   const auto model = robot_model();
//   if (!model)
//   {
//     RCLCPP_ERROR(get_logger(), "RobotModel not ready");
//     return false;
//   }
//   const auto* jmg = model->getJointModelGroup(group_name);
//   if (!jmg)
//   {
//     RCLCPP_ERROR(get_logger(), "JointModelGroup not found");
//     return false;
//   }
//   if (traj.points.empty())
//   {
//     return true;
//   }

//   robot_trajectory::RobotTrajectory rt(model, group_name);
//   moveit::core::RobotState state(start_state);

//   for (const auto& pt : traj.points)
//   {
//     if (pt.positions.size() < jmg->getVariableCount())
//     {
//       RCLCPP_ERROR(get_logger(), "Trajectory point size mismatch");
//       return false;
//     }
//     state.setJointGroupPositions(jmg, pt.positions);
//     state.update();
//     rt.addSuffixWayPoint(state, 0.0);
//   }

//   trajectory_processing::TimeOptimalTrajectoryGeneration totg;
//   const double v_scale = std::clamp(velocity_scaling, 0.01, 1.0);
//   const double a_scale = std::clamp(accel_scaling, 0.01, 1.0);
//   if (!totg.computeTimeStamps(rt, v_scale, a_scale))
//   {
//     RCLCPP_ERROR(get_logger(), "Time parameterization failed");
//     return false;
//   }

//   traj.joint_names = jmg->getVariableNames();
//   traj.points.clear();
//   traj.points.reserve(rt.getWayPointCount());
//   for (std::size_t i = 0; i < rt.getWayPointCount(); ++i)
//   {
//     const moveit::core::RobotState& st = rt.getWayPoint(i);
//     solve_executor::TrajectoryPoint p;
//     st.copyJointGroupPositions(jmg, p.positions);
//     st.copyJointGroupVelocities(jmg, p.velocities);
//     p.time_from_start = rt.getWayPointDurationFromStart(i);
//     traj.points.push_back(std::move(p));
//   }
//   return true;
// }

// void parameterize_time_from_start(Trajectory& traj, double velocity_scaling)
// {
//   if (traj.points.empty())
//   {
//     return;
//   }

//   constexpr double kNominalJointSpeedRadPerSec = 1.0;
//   constexpr double kMinDtSec = 0.01;
//   const double scale = std::clamp(velocity_scaling, 0.05, 1.0);

//   traj.points[0].time_from_start = 0.0;
//   traj.points[0].velocities.assign(traj.points[0].positions.size(), 0.0);

//   for (std::size_t i = 1; i < traj.points.size(); ++i)
//   {
//     const auto& prev = traj.points[i - 1];
//     auto& curr = traj.points[i];
//     const std::size_t dof = std::min(prev.positions.size(), curr.positions.size());

//     double max_delta = 0.0;
//     for (std::size_t j = 0; j < dof; ++j)
//     {
//       max_delta = std::max(max_delta, std::fabs(curr.positions[j] - prev.positions[j]));
//     }

//     const double dt = std::max(kMinDtSec, max_delta / (kNominalJointSpeedRadPerSec * scale));
//     curr.time_from_start = prev.time_from_start + dt;

//     curr.velocities.assign(curr.positions.size(), 0.0);
//     if (dt > 1e-9)
//     {
//       for (std::size_t j = 0; j < dof; ++j)
//       {
//         curr.velocities[j] = (curr.positions[j] - prev.positions[j]) / dt;
//       }
//     }
//   }
// }
// }  // namespace solve_executor

#include "executor/detail.hpp"
#include "executor/executor.hpp"

#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

#include "log_utils/log.hpp"

namespace solve_executor
{

bool SolveExecutor::timeParameterizeTrajectory(solve_executor::Trajectory& traj, std::string& err) const
{
  const std::string group_name = config_.group_name;
  const auto* jmg = robot_model_ ? robot_model_->getJointModelGroup(group_name) : nullptr;
  if (!jmg)
  {
    err = "JointModelGroup not found";
    LOGE("[solve_executor] {}", err);
    return false;
  }

  if (traj.points.empty())
  {
    err = "Cartesian trajectory is empty";
    LOGE("[solve_executor] {}", err);
    return false;
  }

  robot_trajectory::RobotTrajectory robot_traj(robot_model_, group_name);
  moveit::core::RobotState state(robot_model_);
  state.setToDefaultValues();
  state.update();

  for (const auto& point : traj.points)
  {
    if (point.positions.size() < jmg->getVariableCount())
    {
      err = "Trajectory point size mismatch";
      LOGE("[solve_executor] {}", err);
      return false;
    }
    state.setJointGroupPositions(jmg, point.positions);
    state.update();
    robot_traj.addSuffixWayPoint(state, 0.0);
  }

  trajectory_processing::TimeOptimalTrajectoryGeneration totg;
  constexpr double kVelocityScaling = 1.0;
  constexpr double kAccelScaling = 1.0;
  if (!totg.computeTimeStamps(robot_traj, kVelocityScaling, kAccelScaling))
  {
    err = "Time parameterization failed";
    LOGE("[solve_executor] {}", err);
    return false;
  }

  traj = trajectory_from_robot_trajectory(robot_model_, group_name, robot_traj);
  return true;
}

}  // namespace solve_executor

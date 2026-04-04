#pragma once

#include "executor/executor.hpp"

#include <Eigen/Geometry>

namespace solve_executor
{
// ---------------------------------------------------------------------------------------------
// 用于传入关节角
// ---------------------------------------------------------------------------------------------
bool fill_joint_state_require_all(const types::JointState& js, const moveit::core::JointModelGroup* jmg,
                                  moveit::core::RobotState& state, std::string& err);

// ---------------------------------------------------------------------------------------------
// 四元数转旋转矩阵
// ---------------------------------------------------------------------------------------------
Eigen::Isometry3d poseToIsometry(const Pose& pose);

// ---------------------------------------------------------------------------------------------
// 位姿类型转换
// ---------------------------------------------------------------------------------------------
geometry_msgs::msg::PoseStamped resolveTargetPose(const SolveExecutorConfig& config, const SolveRequest& req);

// ---------------------------------------------------------------------------------------------
// 路径类型转换
// ---------------------------------------------------------------------------------------------
Trajectory trajectory_from_robot_trajectory(const moveit::core::RobotModelConstPtr& robot_model,
                                            const std::string& group_name, const robot_trajectory::RobotTrajectory& rt);

Trajectory trajectory_from_plan_msg(const moveit::planning_interface::MoveGroupInterface::Plan& plan_msg);

}  // namespace solve_executor
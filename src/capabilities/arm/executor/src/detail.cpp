#include "executor/detail.hpp"
#include "executor/executor.hpp"

#include <unordered_map>
#include <vector>
#include <Eigen/Geometry>

namespace solve_executor
{
// ---------------------------------------------------------------------------------------------
// 用于传入关节角
// ---------------------------------------------------------------------------------------------
bool fill_joint_state_require_all(const types::JointState& js, const moveit::core::JointModelGroup* jmg,
                                  moveit::core::RobotState& state, std::string& err)
{
  if (!jmg)
  {
    err = "JointModelGroup null";
    return false;
  }
  const auto& group_joint_names = jmg->getVariableNames();
  if (js.names.empty() || js.positions.empty())
  {
    err = "Missing joint state";
    return false;
  }
  if (js.names.size() != js.positions.size())
  {
    err = "Joint names/positions size mismatch";
    return false;
  }

  std::unordered_map<std::string, double> pos_map;
  pos_map.reserve(js.names.size());
  for (std::size_t i = 0; i < js.names.size(); ++i)
  {
    pos_map[js.names[i]] = js.positions[i];
  }
  for (const auto& jn : group_joint_names)
  {
    auto it = pos_map.find(jn);
    if (it == pos_map.end())
    {
      err = "Missing joint state";
      return false;
    }
    state.setVariablePosition(it->first, it->second);
  }
  return true;
}

// ---------------------------------------------------------------------------------------------
// 用于传入关节角
// ---------------------------------------------------------------------------------------------
Eigen::Isometry3d poseToIsometry(const Pose& pose)
{
  Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();
  iso.translation() << pose.x, pose.y, pose.z;
  Eigen::Quaterniond q(pose.qw, pose.qx, pose.qy, pose.qz);
  q.normalize();
  iso.linear() = q.toRotationMatrix();
  return iso;
}

geometry_msgs::msg::PoseStamped resolveTargetPose(const SolveExecutorConfig& config, const SolveRequest& req)
{
  geometry_msgs::msg::PoseStamped target;
  target.header.frame_id = config.planning_frame_id;
  target.pose.position.x = req.target_pose.x;
  target.pose.position.y = req.target_pose.y;
  target.pose.position.z = req.target_pose.z;
  target.pose.orientation.x = req.target_pose.qx;
  target.pose.orientation.y = req.target_pose.qy;
  target.pose.orientation.z = req.target_pose.qz;
  target.pose.orientation.w = req.target_pose.qw;

  return target;
}

// 将Moveit自带的Trajectory转为自己定义的Trajectory
Trajectory trajectory_from_robot_trajectory(const moveit::core::RobotModelConstPtr& robot_model,
                                            const std::string& group_name, const robot_trajectory::RobotTrajectory& rt)
{
  Trajectory out;
  if (!robot_model)
  {
    return out;
  }
  const auto* jmg = robot_model->getJointModelGroup(group_name);
  if (!jmg)
  {
    return out;
  }
  out.joint_names = jmg->getVariableNames();
  const std::size_t count = rt.getWayPointCount();
  out.points.reserve(count);

  for (std::size_t i = 0; i < count; ++i)
  {
    const moveit::core::RobotState& st = rt.getWayPoint(i);
    TrajectoryPoint p;
    st.copyJointGroupPositions(jmg, p.positions);
    st.copyJointGroupVelocities(jmg, p.velocities);
    p.time_from_start = rt.getWayPointDurationFromStart(i);
    out.points.push_back(std::move(p));
  }
  return out;
}

Trajectory trajectory_from_plan_msg(const moveit::planning_interface::MoveGroupInterface::Plan& plan_msg)
{
  solve_executor::Trajectory traj;
  const auto& joint_traj = plan_msg.trajectory_.joint_trajectory;
  traj.joint_names = joint_traj.joint_names;
  traj.points.reserve(joint_traj.points.size());
  for (const auto& point_msg : joint_traj.points)
  {
    solve_executor::TrajectoryPoint point;
    point.positions = point_msg.positions;
    point.velocities = point_msg.velocities;
    point.time_from_start = static_cast<double>(point_msg.time_from_start.sec) +
                            static_cast<double>(point_msg.time_from_start.nanosec) * 1e-9;
    traj.points.push_back(std::move(point));
  }
  return traj;
}
}  // namespace solve_executor
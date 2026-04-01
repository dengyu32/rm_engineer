#include "executor/executor.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <unordered_map>

#include "log_utils/log.hpp"
#include "calculator/cost_func.hpp"
// #include "planner/limit_planner.hpp"
#include "planner/straight_planner.hpp"

#include <Eigen/Geometry>

#include <moveit/collision_detection/collision_common.h>
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

namespace solve_executor
{
// 规划总入口
std::optional<SolveResponse> SolveExecutor::plan(const SolveRequest& req, std::string& err)
{
  if (!adapter_)
  {
    LOGE("[solve_executor] MoveIt adapter not set");
    return std::nullopt;
  }

  switch (req.option)
  {
    case PlanOption::NORMAL:

      //加采样的话，这里要加一个if条件

      return plan_normal(req, err);
    case PlanOption::CARTESIAN:
      return plan_cartesian(req, err);
    case PlanOption::JOINTS:
      return plan_joints(req, err);
    default:
      LOGE("[solve_executor] Unknown planning option");
      return std::nullopt;
  }
}

// 使用ompl采样规划
std::optional<SolveResponse> SolveExecutor::plan_normal(const SolveRequest& req, std::string& err)
{
  const auto robot_model = adapter_->robot_model();
  if (!robot_model)
  {
    LOGE("[solve_executor] RobotModel is null");
    return std::nullopt;
  }

  const std::string group_name = !req.group_name.empty() ? req.group_name : adapter_->group_name();
  const auto* jmg = adapter_->joint_model_group(group_name);
  if (!jmg)
  {
    LOGE("[solve_executor] JointModelGroup not found");
    return std::nullopt;
  }

  auto solver = jmg->getSolverInstance();
  if (!solver)
  {
    LOGE("[solve_executor] IK solver missing");
    return std::nullopt;
  }

  moveit::core::RobotState start_state(robot_model);
  start_state.setToDefaultValues();
  fill_joint_state_allow_missing(req.current_joints, jmg, start_state);
  start_state.update();

  if (!start_state.satisfiesBounds(jmg))
  {
    start_state.enforceBounds(jmg);
    LOGE("[solve_executor] Start state out of bounds");
    return std::nullopt;
  }

  adapter_->set_start_state(start_state);

  std::string ee_link = !req.ee_link.empty() ? req.ee_link : adapter_->end_effector_link();
  if (ee_link.empty())
  {
    const auto& links = jmg->getLinkModelNames();
    if (!links.empty())
    {
      ee_link = links.back();
    }
  }

  Eigen::Isometry3d target_iso = pose_to_isometry(req.target_pose);

  SamplingConfigs sam_configs;
  sam_configs.sampling_mode = SamplingMode::ROLL_SAMPLE;
  // normal planner 的专属参数先由 planner 配置结构体默认值提供，不走 yaml。

  PlannerConfigs planner_cfg;
  planner_cfg.goal_position_tolerance = config_.goal_position_tolerance;
  planner_cfg.goal_orientation_tolerance = config_.goal_orientation_tolerance;
  planner_cfg.planning_time = config_.planning_time;
  planner_cfg.num_planning_attempts = config_.num_planning_attempts;
  planner_cfg.max_velocity_scaling = config_.max_velocity_scaling;
  planner_cfg.max_acc_scaling = config_.max_acc_scaling;

  std::shared_ptr<LimitPlanner> planner = std::make_shared<LimitPlanner>(adapter_);
  auto out_traj = planner->plan(jmg, ee_link, start_state, target_iso, sam_configs, err, planner_cfg);

  // std::optional<solve_executor::Trajectory>
  // plan_to_joint_target(const std::vector<std::string> &joint_names,
  //                      const std::vector<double> &joint_values,
  //                      const solve_executor::PlannerConfigs &configs) override {

  //   moveit::planning_interface::MoveGroupInterface::Plan plan_msg;
  //   const bool ok =
  //       (move_group_->plan(plan_msg) == moveit::core::MoveItErrorCode::SUCCESS);
  //   if (!ok) {
  //     RCLCPP_ERROR(logger_, "[arm_solve_server] Planning failed");
  //     return std::nullopt;
  //   }

  //   solve_executor::Trajectory traj;
  //   const auto &jt   = plan_msg.trajectory_.joint_trajectory;
  //   traj.joint_names = jt.joint_names;
  //   traj.points.reserve(jt.points.size());
  //   for (const auto &pt : jt.points) {
  //     solve_executor::TrajectoryPoint p;
  //     p.positions  = pt.positions;
  //     p.velocities = pt.velocities;
  //     p.time_from_start =
  //         static_cast<double>(pt.time_from_start.sec) +
  //         static_cast<double>(pt.time_from_start.nanosec) * 1e-9;
  //     traj.points.push_back(std::move(p));
  //   }
  //   return traj;
  // }

  auto resp = std::make_optional<SolveResponse>();
  if (out_traj)
  {
    resp->trajectory = std::move(*out_traj);
  }
  else
  {
    return std::nullopt;
  }
  return resp;
}

// 直线规划
std::optional<SolveResponse> SolveExecutor::plan_cartesian(const SolveRequest& req, std::string& err)
{
  auto robot_model = adapter_->robot_model();
  if (!robot_model)
  {
    err = "RobotModel is null";
    LOGE("[solve_executor] {}", err);
    return std::nullopt;
  }

  std::string group_name = req.group_name.empty() ? adapter_->group_name() : req.group_name;

  moveit::core::RobotState start_state(robot_model);
  if (!fill_joint_state_require_all(req.current_joints, adapter_->joint_model_group(group_name), start_state, err))
  {
    err = "Joint Values is absent";
    LOGE("[solve_executor] {}", err);
    return std::nullopt;
  }

  start_state.update();

  std::string ee_link = req.ee_link.empty() ? adapter_->end_effector_link() : req.ee_link;
  if (ee_link.empty())
  {
    err = "End effector link is empty";
    LOGE("[solve_executor] {}", err);
    return std::nullopt;
  }

  StraightPlanner planner(robot_model, group_name, ee_link);
  StraightPlannerConfigs strai_config;
  const Eigen::Isometry3d target_iso = pose_to_isometry(req.target_pose);
  const Eigen::Isometry3d start_iso = start_state.getGlobalLinkTransform(ee_link);
  if (!buildStraightPlannerConfigs(start_iso, target_iso, req.target_vector, req.target_length, strai_config, err))
  {
    LOGE("[solve_executor] {}", err);
    return std::nullopt;
  }

  // todo：暂时修的bug，后续可以改成直接在config里设置代价计算函数的参数
  CostOptions cost_opt;
  LOGI("Ready to Plan by Cartesian!");
  auto traj = planner.plan(start_state, target_iso, strai_config, cost_opt);

  if (!traj)
  {
    LOGE("[solve_executor] No Result from StraightPlanner");
    return std::nullopt;
  }

  // 直线规划结果进行自碰撞检测
  const auto* jmg = adapter_->joint_model_group(group_name);
  if (!jmg)
  {
    LOGE("[solve_executor] JointModelGroup not found");
    return std::nullopt;
  }

  moveit::core::RobotState rs(start_state);
  for (std::size_t i = 0; i < traj->points.size(); ++i)
  {
    const auto& pt = traj->points[i];
    if (pt.positions.size() < jmg->getVariableCount())
    {
      LOGE("[solve_executor] Trajectory point size mismatch for collision check");
      return std::nullopt;
    }
    rs.setJointGroupPositions(jmg, pt.positions);
    rs.update();
    std::string collision_err;
    if (!adapter_->check_self_collision(rs, group_name, collision_err))
    {
      if (collision_err.empty())
        collision_err = "Self collision detected";
      LOGE("[solve_executor] {}", collision_err);
      return std::nullopt;
    }
  }

  SolveResponse resp;
  resp.trajectory = std::move(*traj);
  std::string time_err;
  if (!adapter_->time_parameterize_trajectory(start_state, group_name, resp.trajectory, config_.max_velocity_scaling,
                                              config_.max_acc_scaling, time_err))
  {
    LOGE("[solve_executor] Time parameterization failed: {}", time_err);
    parameterize_time_from_start(resp.trajectory, config_.max_velocity_scaling);
  }
  LOGI("[solve_executor][plan_cartesian] Planning successful with StraightPlanner,trajiectory will be returned!");
  return resp;
}

// 关节空间规划
std::optional<SolveResponse> SolveExecutor::plan_joints(const SolveRequest& req, std::string& err)
{
  const auto robot_model = adapter_->robot_model();
  if (!robot_model)
  {
    LOGE("[solve_executor] RobotModel is null");
    return std::nullopt;
  }
  const std::string group_name = !req.group_name.empty() ? req.group_name : adapter_->group_name();
  const auto* jmg = adapter_->joint_model_group(group_name);
  if (!jmg)
  {
    LOGE("[solve_executor] JointModelGroup not found");
    return std::nullopt;
  }

  const auto& group_joint_names = jmg->getVariableNames();
  const std::size_t dof = group_joint_names.size();
  if (req.target_joints.size() < dof)
  {
    LOGE("[solve_executor] Target joints size mismatch");
    return std::nullopt;
  }

  moveit::core::RobotState start_state(robot_model);
  start_state.setToDefaultValues();
  if (!fill_joint_state_require_all(req.current_joints, jmg, start_state, err))
  {
    LOGE("[solve_executor] {}", err);
    return std::nullopt;
  }
  start_state.update();

  if (!start_state.satisfiesBounds(jmg))
  {
    start_state.enforceBounds(jmg);
  }

  std::unordered_map<std::string, double> start_joint_position;
  start_joint_position.reserve(dof);
  for (const auto& jn : group_joint_names)
  {
    const double* pos_ptr = start_state.getJointPositions(jn);
    const double pos_now = pos_ptr ? *pos_ptr : 0.0;
    start_joint_position.emplace(jn, pos_now);
  }

  JointsPlannerConfigs joints_config;
  joints_config.validate();
  const double max_step_rad = joints_config.max_step_rad;
  double max_delta = 0.0;
  for (std::size_t i = 0; i < dof; ++i)
  {
    const auto& jn = group_joint_names[i];
    auto it = start_joint_position.find(jn);
    if (it == start_joint_position.end())
    {
      LOGE("[solve_executor] Joint lookup failed");
      return std::nullopt;
    }
    max_delta = std::max(max_delta, std::fabs(req.target_joints[i] - it->second));
  }
  const int N = std::max(1, static_cast<int>(std::ceil(max_delta / std::max(1e-6, max_step_rad))));

  Trajectory traj;
  traj.joint_names = group_joint_names;
  traj.points.reserve(static_cast<std::size_t>(N + 1));

  moveit::core::RobotState rs = start_state;

  for (int k = 0; k <= N; ++k)
  {
    const double t_raw = static_cast<double>(k) / static_cast<double>(N);
    // 平滑插值（前缓中匀后缓），降低速度突变
    const double t = t_raw * t_raw * (3.0 - 2.0 * t_raw);
    std::vector<double> q(dof);
    for (std::size_t i = 0; i < dof; ++i)
    {
      const auto& jn = group_joint_names[i];
      auto it = start_joint_position.find(jn);
      if (it == start_joint_position.end())
      {
        LOGE("[solve_executor] Joint lookup failed");
        return std::nullopt;
      }
      q[i] = it->second + (req.target_joints[i] - it->second) * t;
    }

    rs.setJointGroupPositions(jmg, q);
    rs.update();

    std::string collision_err;
    if (!adapter_->check_self_collision(rs, group_name, collision_err))
    {
      if (collision_err.empty())
        collision_err = "Self collision detected";
      LOGE("[solve_executor] {}", collision_err);
      return std::nullopt;
    }

    TrajectoryPoint p;
    p.positions = q;
    traj.points.push_back(std::move(p));
  }

  SolveResponse resp;
  resp.trajectory = std::move(traj);
  parameterize_time_from_start(resp.trajectory, config_.max_velocity_scaling);
  return resp;
}

}  // namespace solve_executor
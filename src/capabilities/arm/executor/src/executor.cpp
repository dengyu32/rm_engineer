#include <algorithm>
#include <chrono>
#include <map>
#include <memory>
#include <thread>
#include <vector>
#include <Eigen/Geometry>

#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

#include "collision/self_collision_detector.hpp"
#include "executor/executor.hpp"
#include "log_tools/log.hpp"
#include "planner/straight_planner.hpp"
#include "calculator/cost_func.hpp"
#include "executor/detail.hpp"
#include "executor/test.hpp"

namespace solve_executor
{
namespace
{
const moveit::core::JointModelGroup* get_jmg(moveit::core::RobotModelConstPtr robot_model,const std::string& group_name)
{
  const auto jmg = robot_model->getJointModelGroup(group_name);
  if (!jmg)
  {
    return nullptr;
  }
  return jmg;
}

bool is_finite_pose(const Pose& pose)
{
  return std::isfinite(pose.x) && std::isfinite(pose.y) && std::isfinite(pose.z) && std::isfinite(pose.qx) &&
         std::isfinite(pose.qy) && std::isfinite(pose.qz) && std::isfinite(pose.qw);
}

}
SolveExecutor::SolveExecutor(rclcpp::Node& node)
  : node_(node)
  , clock_(node.get_clock())
  , config_(SolveExecutorConfig::Load(node))
  , last_plan_time_(clock_->now() - rclcpp::Duration(std::chrono::milliseconds(config_.plan_min_interval_ms)))
  , node_handle_(std::shared_ptr<rclcpp::Node>(&node_, [](rclcpp::Node*) {}))
{
  LOGI("\n{}", config_.summary());
}

bool SolveExecutor::isReady() const
{
  std::scoped_lock<std::mutex> lock(init_mutex_);
  return move_group_ && psm_ && robot_model_ && current_state_;
}

void SolveExecutor::stop()
{
  std::scoped_lock<std::mutex> lock(init_mutex_);
  if (move_group_)
  {
    move_group_->stop();
  }
}

bool SolveExecutor::ensureInitialized(std::string& err)
{
  std::scoped_lock<std::mutex> lock(init_mutex_);
  if (move_group_ && psm_ && robot_model_ && current_state_)
  {
    return true;
  }

  if (config_.late_init_delay_ms > 0)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(config_.late_init_delay_ms));
  }

  if (!move_group_)
  {
    move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(node_handle_, config_.group_name);
  }

  if (!psm_)
  {
    psm_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node_handle_, "robot_description");
    psm_->startSceneMonitor();
    psm_->startWorldGeometryMonitor();
    psm_->startStateMonitor();
  }

  robot_model_ = move_group_->getRobotModel();
  if (!robot_model_)
  {
    err = "Robot model is unavailable";
    LOGE("[solve_executor] {}", err);
    return false;
  }

  if (!current_state_)
  {
    current_state_ = std::make_unique<moveit::core::RobotState>(robot_model_);
    current_state_->setToDefaultValues();
    current_state_->update();
  }

  return true;
}

bool SolveExecutor::execute(const SolveRequest& req, solve_executor::Trajectory& out_traj, std::string& err)
{
  std::scoped_lock<std::mutex> lock(exec_mutex_);

  if (!ensureInitialized(err))
  {
    return false;
  }

  const auto now = clock_->now();
  const auto min_interval = rclcpp::Duration(std::chrono::milliseconds(config_.plan_min_interval_ms));
  if (config_.plan_min_interval_ms > 0 && now - last_plan_time_ < min_interval)
  {
    err = "Planning request throttled by plan_min_interval_ms";
    LOGW("[solve_executor] {}", err);
    return false;
  }

  if (!plan(req, out_traj, err))
  {
    return false;
  }

  last_plan_time_ = now;
  return true;
}

bool SolveExecutor::plan(const SolveRequest& req, solve_executor::Trajectory& out_traj, std::string& err)
{
  const auto planning_scene = psm_ ? psm_->getPlanningScene() : nullptr;
  if (!planning_scene)
  {
    err = "Planning scene is unavailable";
    LOGE("[solve_executor] {}", err);
    return false;
  }

  collision::SelfCollisionRequest collision_req;
  collision_req.max_contacts = config_.max_contacts;
  collision_req.max_contacts_per_pair = config_.max_contacts_per_pair;

  collision::SelfCollisionDetector self_collision_detector(config_.group_name, collision_req, robot_model_,
                                                           planning_scene);

  switch (req.option)
  {
    case PlanOption::NORMAL:
      return plan_normal(req, out_traj, err, &self_collision_detector);
    case PlanOption::CARTESIAN:
      return plan_cartesian(req, out_traj, err, &self_collision_detector);
    case PlanOption::JOINTS:
      return plan_joints(req, out_traj, err, &self_collision_detector);
    default:
      err = "Unknown planning option";
      LOGE("[solve_executor] {}", err);
      return false;
  }
}

bool SolveExecutor::plan_normal(const SolveRequest& req, solve_executor::Trajectory& out_traj, std::string& err,
                                const collision::SelfCollisionDetector* self_collision_detector)
{
  (void)self_collision_detector;
  if (!is_finite_pose(req.target_pose))
  {
    err = "target_pose contains non-finite values";
    LOGE("[solve_executor] {}", err);
    return false;
  }

  std::string group_name = config_.group_name;
  std::string ee_link_name = config_.ee_link_name;
  const auto jmg = get_jmg(robot_model_, group_name);
  moveit::core::RobotState start_state(robot_model_);
  if (!fill_joint_state_require_all(req.current_joints, jmg, start_state, err))
  {
    return false;
  }

  move_group_->setStartState(start_state);
  move_group_->clearPoseTargets();
  move_group_->setPoseReferenceFrame(config_.planning_frame_id);
  move_group_->setPoseTarget(resolveTargetPose(config_,req).pose, ee_link_name);

  moveit::planning_interface::MoveGroupInterface::Plan plan_msg;
  const bool ok = (move_group_->plan(plan_msg) == moveit::core::MoveItErrorCode::SUCCESS);
  if (!ok)
  {
    err = "MoveGroup pose planning failed";
    LOGE("[solve_executor] {}", err);
    return false;
  }

  out_traj = trajectory_from_plan_msg(plan_msg);
  return true;
}

bool SolveExecutor::plan_joints(const SolveRequest& req, solve_executor::Trajectory& out_traj, std::string& err,
                                const collision::SelfCollisionDetector* self_collision_detector)
{
  (void)self_collision_detector;
  for (double value : req.target_joints)
  {
    if (!std::isfinite(value))
    {
      err = "target_joints contains non-finite value";
      LOGE("[solve_executor] {}", err);
      return false;
    }
  }

  const auto group_name = config_.group_name;
  const auto jmg = get_jmg(robot_model_, group_name);
  moveit::core::RobotState start_state(robot_model_);
  if (!fill_joint_state_require_all(req.current_joints, jmg, start_state, err))
  {
    err = "target_joints contains non-finite value";
    LOGE("[solve_executor] {}", err);
    return false;
  }

  double max_step_rad = config_.joints_max_step_rad;
  double max_delta = 0.0;
  moveit::core::RobotState insert_state = start_state;
  for (std::size_t i = 0; i < config_.joint_names.size(); ++i)
  {
    // const auto& jn = config_.joint_names[i];
    // auto it = req.current_joints.find(jn);
    // if (it == req.current_joints.end())
    // {
    //   LOGE("[solve_core] Joint lookup failed");
    //   return std::nullopt;
    // }
    // const double target_near = ikc::wrapToNearby(req.target_joints[i], it->second);
    // max_delta = std::max(max_delta, std::fabs(req.target_joints[i] - it->second));
    max_delta = std::max(max_delta, std::fabs(req.target_joints[i] - req.current_joints.positions[i]));
  }

  const int N = std::max(1, static_cast<int>(std::ceil(max_delta / std::max(1e-6, max_step_rad))));

  Trajectory traj;
  traj.joint_names = config_.joint_names;
  traj.points.reserve(static_cast<std::size_t>(N + 1));

  for (int k = 0; k <= N; ++k)
  {
    const double t_raw = static_cast<double>(k) / static_cast<double>(N);
    // 平滑插值（前缓中匀后缓），降低速度突变
    const double t = t_raw * t_raw * (3.0 - 2.0 * t_raw);
    std::vector<double> q(config_.joint_count);
    for (std::size_t i = 0; i < config_.joint_names.size(); ++i)
    {
      // const auto& jn = config_.joint_names[i];
      // auto it = req.current_joints.find(jn);
      // if (it == req.current_joints.end())
      // {
      //   LOGE("[solve_core] Joint lookup failed");
      //   return false;
      // }
      // const double target_near = ikc::wrapToNearby(req.target_joints[i], it->second);
      q[i] = req.current_joints.positions[i] + (req.target_joints[i] - req.current_joints.positions[i]) * t;
    }

    insert_state.setJointGroupPositions(jmg, q);
    insert_state.update();

    collision::SelfCollisionCheckResult collision_result;
    if (!self_collision_detector->check(q, collision_result))
    {
      LOGE("[solve_executor][joint_planner] : Collision at joint space interpolation points k=%d", k);
      return false;
    }

    TrajectoryPoint p;
    p.positions = q;
    traj.points.push_back(std::move(p));
  }
  parameterize_time_from_start(traj, config_.max_velocity_scaling);

  out_traj = std::move(traj);
  return true;
}

bool SolveExecutor::plan_cartesian(const SolveRequest& req, solve_executor::Trajectory& out_traj, std::string& err,
                                   const collision::SelfCollisionDetector* self_collision_detector)
{
  constexpr double kEps = 1e-9;
  const auto group_name = config_.group_name;
  const auto ee_link = config_.ee_link_name;
  const auto jmg = get_jmg(robot_model_, group_name);
  const auto* ref_link = robot_model_->getLinkModel(config_.reference_link);
  const auto* ee_link_model = robot_model_->getLinkModel(ee_link);

  moveit::core::RobotState start_state(robot_model_);
  if (!fill_joint_state_require_all(req.current_joints, jmg, start_state, err))
  {
    return false;
  }
  start_state.update();

  if (!std::isfinite(req.target_length) || req.target_length <= 0.0F)
  {
    err = "target_length is invalid";
    LOGE("[solve_executor] {}", err);
    return false;
  }

  Eigen::Vector3d target_vector = req.target_vector;
  if (config_.use_vision_target_vector)
  {
    if (!target_vector.allFinite())
    {
      err = "target_vector contains non-finite values";
      LOGE("[solve_executor] {}", err);
      return false;
    }
    if (target_vector.norm() <= kEps)
    {
      err = "target_vector is a zero vector";
      LOGE("[solve_executor] {}", err);
      return false;
    }
  }
  else
  {
    if (!set_vector_with_current(start_state, ref_link, ee_link_model, target_vector, err))
    {
      LOGE("[solve_executor] {}", err);
      return false;
    }
  }

  planner::StraightPlannerSettings settings;
  settings.sample_step_m = config_.sample_step_m;
  settings.alignment_dot_threshold = config_.alignment_dot_threshold;
  settings.reference_link = config_.reference_link;
  planner::StraightPlanner planner(robot_model_, group_name, ee_link, settings, self_collision_detector);

  calculator::CostOptions cost_opt;
  auto raw_traj = planner.plan(start_state, target_vector, req.target_length, cost_opt);
  if (!raw_traj)
  {
    err = err.empty() ? "Straight planner failed" : err;
    LOGE("[solve_executor] {}", err);
    return false;
  }

  out_traj = *raw_traj;
  if (!timeParameterizeTrajectory(out_traj, err))
  {
    return false;
  }
  return true;
}

}  // namespace solve_executor

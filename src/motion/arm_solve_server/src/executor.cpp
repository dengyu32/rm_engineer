#include "executor.hpp"

#include <algorithm>
#include <chrono>
#include <map>
#include <memory>
#include <optional>

#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <rclcpp/node.hpp>

#include "arm_solve_server.hpp"

namespace arm_solve {
namespace solve_executor {
using namespace std::chrono_literals;

namespace {
solve_core::Pose toSolvePose(const geometry_msgs::msg::PoseStamped &pose_msg) {
  solve_core::Pose pose;
  pose.x = pose_msg.pose.position.x;
  pose.y = pose_msg.pose.position.y;
  pose.z = pose_msg.pose.position.z;
  pose.qx = pose_msg.pose.orientation.x;
  pose.qy = pose_msg.pose.orientation.y;
  pose.qz = pose_msg.pose.orientation.z;
  pose.qw = pose_msg.pose.orientation.w;
  return pose;
}
} // namespace

SolveExecutorConfig makeSolveExecutorConfig(const arm_solve::ArmSolveConfig &config) {
  SolveExecutorConfig exec_config;
  exec_config.group_name = config.group_name;
  exec_config.ee_link_name = config.ee_link_name;
  exec_config.joint_count = config.joint_count;
  exec_config.joint_names = config.joint_names;
  exec_config.joint_index = config.joint_index;
  exec_config.joint_states_topic = config.joint_states_topic;
  exec_config.joint_states_custom_topic = config.joint_states_custom_topic;
  exec_config.joint_states_verbose_topic = config.joint_states_verbose_topic;
  exec_config.joint_cmd_topic = config.joint_cmd_topic;
  exec_config.late_init_delay_ms = config.late_init_delay_ms;
  exec_config.plan_min_interval_ms = config.plan_min_interval_ms;
  exec_config.validate();
  return exec_config;
}

SolveExecutor::SolveExecutor(rclcpp::Node &host_node,
                             const SolveExecutorConfig &config,
                             const solve_core::SolveCoreConfig &solve_core_config)
    : logger_(host_node.get_logger()),
      clock_(host_node.get_clock()),
      config_(config),
      solve_core_config_(solve_core_config),
      last_plan_time_(
          clock_->now() - rclcpp::Duration(std::chrono::milliseconds(config_.plan_min_interval_ms))),
      moveit_adapter_(
          std::shared_ptr<solve_core::MoveItAdapter>(this, [](solve_core::MoveItAdapter *) {})) {
  config_.validate();
  solve_core_config_.validate();
  RCLCPP_INFO(logger_, "\n%s", config_.summary().c_str());
  RCLCPP_INFO(logger_, "\n%s", solve_core_config_.summary().c_str());
}

void SolveExecutor::lateInit(const std::shared_ptr<rclcpp::Node> &host_node) {
  if (!host_node) {
    RCLCPP_ERROR(logger_, "[solve_executor] lateInit failed: host node is null");
    return;
  }
  if (!move_group_) {
    move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
        host_node, config_.group_name);
  }
  if (!psm_) {
    psm_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
        host_node, "robot_description");
    psm_->startSceneMonitor();
    psm_->startWorldGeometryMonitor();
    psm_->startStateMonitor();
  }
  if (!solve_core_) {
    solve_core_ =
        std::make_unique<solve_core::SolveCore>(moveit_adapter_, solve_core_config_);
  }
}

bool SolveExecutor::isReady() const {
  return move_group_ && move_group_->getRobotModel() &&
         !move_group_->getPlanningFrame().empty() && solve_core_;
}

void SolveExecutor::stop() {
  if (move_group_) {
    move_group_->stop();
  }
}

std::string SolveExecutor::planning_frame_id() const {
  return planning_frame();
}

bool SolveExecutor::execute(const arm_solve::GoalContext &ctx,
                            solve_core::Trajectory &out_traj,
                            std::string &err) {
  std::scoped_lock<std::mutex> lock(exec_mutex_);
  if (!isReady()) {
    err = "SolveExecutor not ready";
    RCLCPP_ERROR(logger_, "[solve_executor] %s", err.c_str());
    return false;
  }
  return planTrajectory(ctx, out_traj, err);
}

bool SolveExecutor::planTrajectory(const arm_solve::GoalContext &ctx,
                                   solve_core::Trajectory &out_traj,
                                   std::string &err) {
  const auto now = clock_->now();
  const auto min_interval =
      rclcpp::Duration(std::chrono::milliseconds(config_.plan_min_interval_ms));
  if (config_.plan_min_interval_ms > 0 &&
      now - last_plan_time_ < min_interval) {
    err = "Planning request throttled by plan_min_interval_ms";
    RCLCPP_WARN(logger_, "[solve_executor] %s", err.c_str());
    return false;
  }

  solve_core::SolveRequest req;
  req.option = ctx.option;
  req.target_pose = toSolvePose(ctx.target_pose);
  req.target_vector = ctx.target_vector;
  req.target_length = ctx.target_length;
  req.target_joints.assign(ctx.target_joints.begin(), ctx.target_joints.end());
  req.current_joints = ctx.current_joints;
  req.group_name = config_.group_name;
  req.ee_link = config_.ee_link_name;

  auto resp = solve_core_->plan(req, err);
  last_plan_time_ = now;
  if (!resp) {
    if (err.empty()) {
      err = "SolveCore planning failed";
    }
    RCLCPP_ERROR(logger_, "[solve_executor] %s", err.c_str());
    return false;
  }

  out_traj = std::move(resp->trajectory);
  return true;
}

void SolveExecutor::applyPlannerConfigs(const solve_core::PlannerConfigs &configs) {
  if (!move_group_) {
    return;
  }
  move_group_->setPlanningTime(configs.planning_time);
  move_group_->setNumPlanningAttempts(configs.num_planning_attempts);
  move_group_->setMaxVelocityScalingFactor(configs.max_velocity_scaling);
  move_group_->setMaxAccelerationScalingFactor(configs.max_acc_scaling);
  move_group_->setGoalPositionTolerance(configs.goal_position_tolerance);
  move_group_->setGoalOrientationTolerance(configs.goal_orientation_tolerance);
}

std::shared_ptr<const moveit::core::RobotModel> SolveExecutor::robot_model() const {
  if (!move_group_) {
    return nullptr;
  }
  return move_group_->getRobotModel();
}

const moveit::core::JointModelGroup *
SolveExecutor::joint_model_group(const std::string &group_name) const {
  const auto model = robot_model();
  if (!model) {
    return nullptr;
  }
  return model->getJointModelGroup(group_name);
}

std::string SolveExecutor::group_name() const { return config_.group_name; }

std::string SolveExecutor::planning_frame() const {
  return move_group_ ? move_group_->getPlanningFrame() : std::string();
}

std::string SolveExecutor::end_effector_link() const {
  if (move_group_) {
    const auto ee = move_group_->getEndEffectorLink();
    if (!ee.empty()) {
      return ee;
    }
  }
  return config_.ee_link_name;
}

void SolveExecutor::set_start_state(const moveit::core::RobotState &state) {
  if (move_group_) {
    move_group_->setStartState(state);
  }
}

std::optional<solve_core::Trajectory> SolveExecutor::plan_to_joint_target(
    const std::vector<std::string> &joint_names,
    const std::vector<double> &joint_values,
    const solve_core::PlannerConfigs &options) {
  if (!move_group_) {
    RCLCPP_ERROR(logger_, "[solve_executor] MoveGroup not ready");
    return std::nullopt;
  }
  if (joint_names.size() != joint_values.size()) {
    RCLCPP_ERROR(logger_, "[solve_executor] joint_names/joint_values size mismatch");
    return std::nullopt;
  }

  applyPlannerConfigs(options);
  move_group_->clearPoseTargets();
  std::map<std::string, double> joint_targets;
  for (std::size_t i = 0; i < joint_names.size(); ++i) {
    joint_targets.emplace(joint_names[i], joint_values[i]);
  }
  if (!move_group_->setJointValueTarget(joint_targets)) {
    RCLCPP_ERROR(logger_, "[solve_executor] setJointValueTarget failed");
    return std::nullopt;
  }

  moveit::planning_interface::MoveGroupInterface::Plan plan_msg;
  const bool ok =
      (move_group_->plan(plan_msg) == moveit::core::MoveItErrorCode::SUCCESS);
  if (!ok) {
    RCLCPP_ERROR(logger_, "[solve_executor] MoveGroup plan failed");
    return std::nullopt;
  }

  solve_core::Trajectory traj;
  const auto &jt = plan_msg.trajectory_.joint_trajectory;
  traj.joint_names = jt.joint_names;
  traj.points.reserve(jt.points.size());
  for (const auto &pt : jt.points) {
    solve_core::TrajectoryPoint p;
    p.positions = pt.positions;
    p.velocities = pt.velocities;
    p.time_from_start = static_cast<double>(pt.time_from_start.sec) +
                        static_cast<double>(pt.time_from_start.nanosec) * 1e-9;
    traj.points.push_back(std::move(p));
  }
  return traj;
}

bool SolveExecutor::check_self_collision(const moveit::core::RobotState &state,
                                         const std::string &group_name,
                                         std::string &err) const {
  if (!psm_) {
    err = "Planning scene not ready";
    RCLCPP_ERROR(logger_, "[solve_executor] %s", err.c_str());
    return false;
  }

  planning_scene_monitor::LockedPlanningSceneRO scene(psm_);
  if (!scene) {
    err = "Planning scene unavailable";
    RCLCPP_ERROR(logger_, "[solve_executor] %s", err.c_str());
    return false;
  }

  collision_detection::CollisionRequest req;
  collision_detection::CollisionResult res;
  req.group_name = group_name;
  req.contacts = false;
  req.max_contacts = 0;
  scene->checkSelfCollision(req, res, state);
  if (res.collision) {
    err = "Self collision detected";
    RCLCPP_ERROR(logger_, "[solve_executor] %s", err.c_str());
    return false;
  }
  return true;
}

bool SolveExecutor::time_parameterize_trajectory(
    const moveit::core::RobotState &start_state, const std::string &group_name,
    solve_core::Trajectory &traj, double velocity_scaling, double accel_scaling,
    std::string &err) const {
  if (!move_group_) {
    err = "MoveGroup not ready";
    RCLCPP_ERROR(logger_, "[solve_executor] %s", err.c_str());
    return false;
  }

  const auto model = robot_model();
  if (!model) {
    err = "RobotModel not ready";
    RCLCPP_ERROR(logger_, "[solve_executor] %s", err.c_str());
    return false;
  }

  const auto *jmg = model->getJointModelGroup(group_name);
  if (!jmg) {
    err = "JointModelGroup not found";
    RCLCPP_ERROR(logger_, "[solve_executor] %s", err.c_str());
    return false;
  }

  if (traj.points.empty()) {
    return true;
  }

  robot_trajectory::RobotTrajectory rt(model, group_name);
  moveit::core::RobotState state(start_state);

  for (const auto &pt : traj.points) {
    if (pt.positions.size() < jmg->getVariableCount()) {
      err = "Trajectory point size mismatch";
      RCLCPP_ERROR(logger_, "[solve_executor] %s", err.c_str());
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
    err = "Time parameterization failed";
    RCLCPP_ERROR(logger_, "[solve_executor] %s", err.c_str());
    return false;
  }

  traj.joint_names = jmg->getVariableNames();
  traj.points.clear();
  traj.points.reserve(rt.getWayPointCount());
  for (std::size_t i = 0; i < rt.getWayPointCount(); ++i) {
    const moveit::core::RobotState &st = rt.getWayPoint(i);
    solve_core::TrajectoryPoint p;
    st.copyJointGroupPositions(jmg, p.positions);
    st.copyJointGroupVelocities(jmg, p.velocities);
    p.time_from_start = rt.getWayPointDurationFromStart(i);
    traj.points.push_back(std::move(p));
  }
  return true;
}

} // namespace solve_executor
} // namespace arm_solve

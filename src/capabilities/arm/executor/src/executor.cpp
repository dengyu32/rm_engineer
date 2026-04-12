#include <Eigen/Geometry>
#include <algorithm>
#include <chrono>
#include <map>
#include <memory>
#include <sstream>
#include <thread>
#include <vector>

#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

#include "calculator/cost_func.hpp"
#include "calculator/hybrid_ik.hpp"
#include "collision/self_collision_detector.hpp"
#include "executor/detail.hpp"
#include "executor/executor.hpp"
#include "executor/test.hpp"
#include "log_tools/log.hpp"
#include "planner/straight_planner.hpp"

namespace solve_executor {
namespace {
const moveit::core::JointModelGroup *
get_jmg(moveit::core::RobotModelConstPtr robot_model,
        const std::string &group_name) {
  const auto jmg = robot_model->getJointModelGroup(group_name);
  if (!jmg) {
    return nullptr;
  }
  return jmg;
}

bool is_finite_pose(const Pose &pose) {
  return std::isfinite(pose.x) && std::isfinite(pose.y) &&
         std::isfinite(pose.z) && std::isfinite(pose.qx) &&
         std::isfinite(pose.qy) && std::isfinite(pose.qz) &&
         std::isfinite(pose.qw);
}

} // namespace
SolveExecutor::SolveExecutor(rclcpp::Node &node)
    : node_(node), logger_(node.get_logger()), clock_(node.get_clock()),
      config_(SolveExecutorConfig::Load(node)),
      last_plan_time_(clock_->now() -
                      rclcpp::Duration(std::chrono::milliseconds(
                          config_.plan_min_interval_ms))),
      node_handle_(
          std::shared_ptr<rclcpp::Node>(&node_, [](rclcpp::Node *) {})) {
  RCLCPP_INFO(logger_, "\n%s", this->config_.summary().c_str());
}

bool SolveExecutor::isReady() const {
  std::scoped_lock<std::mutex> lock(init_mutex_);
  return move_group_ && psm_ && robot_model_ && current_state_;
}

void SolveExecutor::stop() {
  std::scoped_lock<std::mutex> lock(init_mutex_);
  if (move_group_) {
    move_group_->stop();
  }
}

void SolveExecutor::updateJointsMaxStepRad(double value) {
  std::scoped_lock<std::mutex> lock(exec_mutex_);
  config_.joints_max_step_rad = value;
  LOGI("[solve_executor] Updated joints_max_step_rad to {}", value);
}

void SolveExecutor::updateNominalJointSpeed(double value) {
  std::scoped_lock<std::mutex> lock(exec_mutex_);
  config_.nominal_joint_speed = value;
  LOGI("[solve_executor] Updated nominal_joint_speed to {}", value);
}

bool SolveExecutor::ensureInitialized(std::string &err) {
  std::scoped_lock<std::mutex> lock(init_mutex_);
  if (move_group_ && psm_ && robot_model_ && current_state_) {
    return true;
  }

  if (config_.late_init_delay_ms > 0) {
    std::this_thread::sleep_for(
        std::chrono::milliseconds(config_.late_init_delay_ms));
  }

  if (!move_group_) {
    move_group_ =
        std::make_unique<moveit::planning_interface::MoveGroupInterface>(
            node_handle_, config_.group_name);
  }

  if (!psm_) {
    psm_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
        node_handle_, "robot_description");
    psm_->startSceneMonitor();
    psm_->startWorldGeometryMonitor();
    psm_->startStateMonitor();
  }

  robot_model_ = move_group_->getRobotModel();
  if (!robot_model_) {
    err = "Robot model is unavailable";
    LOGE("[solve_executor] {}", err);
    return false;
  }

  if (!current_state_) {
    current_state_ = std::make_unique<moveit::core::RobotState>(robot_model_);
    current_state_->setToDefaultValues();
    current_state_->update();
  }

  return true;
}

bool SolveExecutor::execute(const SolveRequest &req,
                            solve_executor::Trajectory &out_traj,
                            std::string &err) {
  std::scoped_lock<std::mutex> lock(exec_mutex_);

  if (!ensureInitialized(err)) {
    return false;
  }

  const auto now = clock_->now();
  const auto min_interval =
      rclcpp::Duration(std::chrono::milliseconds(config_.plan_min_interval_ms));
  if (config_.plan_min_interval_ms > 0 &&
      now - last_plan_time_ < min_interval) {
    err = "Planning request throttled by plan_min_interval_ms";
    LOGW("[solve_executor] {}", err);
    return false;
  }

  if (!plan(req, out_traj, err)) {
    return false;
  }
<<<<<<< HEAD
<<<<<<< HEAD
  if (out_traj.points.empty()) {
    LOGW("[solve_executor] Planning succeeded but trajectory is empty");
  } else {
    const auto &last_point = out_traj.points.back();
    std::ostringstream oss;
    oss << "[solve_executor] Last trajectory point joint positions:";
    for (std::size_t i = 0; i < last_point.positions.size(); ++i) {
      oss << ' ';
      if (i < out_traj.joint_names.size()) {
        oss << out_traj.joint_names[i] << '=';
      } else {
        oss << "joint_" << i << '=';
      }
      oss << last_point.positions[i];
    }
<<<<<<< HEAD
=======

=======
>>>>>>> 13c3104 (rebase finish)
  if (out_traj.points.empty()) {
      LOGW("[solve_executor] Planning succeeded but trajectory is empty");
    } else {
      const auto &last_point = out_traj.points.back();
      std::ostringstream oss;
      oss << "[solve_executor] Last trajectory point joint positions:";
      for (std::size_t i = 0; i < last_point.positions.size(); ++i) {
        oss << ' ';
        if (i < out_traj.joint_names.size()) {
          oss << out_traj.joint_names[i] << '=';
        } else {
          oss << "joint_" << i << '=';
        }
        oss << last_point.positions[i];
      }
      LOGI("{}", oss.str());
    }
<<<<<<< HEAD
    LOGT("{}", oss.str());
  }

>>>>>>> c87041f (solve-1.10:加入五自由度限制直线规划,调整计算次数)
=======
>>>>>>> 13c3104 (rebase finish)
=======
    LOGI("{}", oss.str());
  }
>>>>>>> 8a38c46 (进度同步)
  last_plan_time_ = now;
  return true;
}

bool SolveExecutor::plan(const SolveRequest &req,
                         solve_executor::Trajectory &out_traj,
                         std::string &err) {
  const auto planning_scene = psm_ ? psm_->getPlanningScene() : nullptr;
  if (!planning_scene) {
    err = "Planning scene is unavailable";
    LOGE("[solve_executor] {}", err);
    return false;
  }

  collision::SelfCollisionRequest collision_req;
  collision_req.max_contacts = config_.max_contacts;
  collision_req.max_contacts_per_pair = config_.max_contacts_per_pair;

  collision::SelfCollisionDetector self_collision_detector(
      config_.group_name, collision_req, robot_model_, planning_scene);

  switch (req.option) {
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

bool SolveExecutor::plan_normal(
    const SolveRequest &req, solve_executor::Trajectory &out_traj,
    std::string &err,
    const collision::SelfCollisionDetector *self_collision_detector) {
  if (!is_finite_pose(req.target_pose)) {
    err = "target_pose contains non-finite values";
    LOGE("[solve_executor] {}", err);
    return false;
  }

  const auto group_name = config_.group_name;
  const auto ee_link_name = config_.ee_link_name;
  const auto jmg = get_jmg(robot_model_, group_name);
  if (!jmg) {
    err = "JointModelGroup not found";
    LOGE("[solve_executor] {}", err);
    return false;
  }

  moveit::core::RobotState start_state(robot_model_);
  if (!fill_joint_state_require_all(req.current_joints, jmg, start_state,
                                    err)) {
    return false;
  }
  start_state.update();

  calculator::HybridIK hybrid_ik(robot_model_, group_name, ee_link_name);
  calculator::IKOptions ik_options;
  std::vector<std::vector<double>> ik_solutions;
  if (!hybrid_ik.solveAll(start_state, poseToIsometry(req.target_pose),
                          ik_options, ik_solutions)) {
    err = "HybridIK failed for target_pose";
    LOGE("[solve_executor] {}", err);
    return false;
  }

  std::vector<double> target_pose = std::move(ik_solutions.front());
  if (target_pose.size() != jmg->getVariableCount()) {
    err = "IK joint result size mismatch";
    LOGE("[solve_executor] {}", err);
    return false;
  }

  SolveRequest joint_req = req;
  joint_req.target_joints = std::move(target_pose);
  return plan_joints(joint_req, out_traj, err, self_collision_detector);
}

bool SolveExecutor::plan_joints(
    const SolveRequest &req, solve_executor::Trajectory &out_traj,
    std::string &err,
    const collision::SelfCollisionDetector *self_collision_detector) {
  (void)self_collision_detector;
  for (double value : req.target_joints) {
    if (!std::isfinite(value)) {
      err = "target_joints contains non-finite value";
      LOGE("[solve_executor] {}", err);
      return false;
    }
  }

  const auto group_name = config_.group_name;
  const auto jmg = get_jmg(robot_model_, group_name);
  moveit::core::RobotState start_state(robot_model_);
  if (!fill_joint_state_require_all(req.current_joints, jmg, start_state,
                                    err)) {
    err = "target_joints contains non-finite value";
    LOGE("[solve_executor] {}", err);
    return false;
  }

  double max_step_rad = config_.joints_max_step_rad;
  double max_delta = 0.0;
  moveit::core::RobotState insert_state = start_state;
  for (std::size_t i = 0; i < config_.joint_names.size(); ++i) {
    // const auto& jn = config_.joint_names[i];
    // auto it = req.current_joints.find(jn);
    // if (it == req.current_joints.end())
    // {
    //   LOGE("[solve_core] Joint lookup failed");
    //   return std::nullopt;
    // }
    // const double target_near = ikc::wrapToNearby(req.target_joints[i],
    // it->second); max_delta = std::max(max_delta,
    // std::fabs(req.target_joints[i] - it->second));
    max_delta = std::max(max_delta, std::fabs(req.target_joints[i] -
                                              req.current_joints.positions[i]));
  }

  const int N = std::max(
      1, static_cast<int>(std::ceil(max_delta / std::max(1e-6, max_step_rad))));

  Trajectory traj;
  traj.joint_names = config_.joint_names;
  traj.points.reserve(static_cast<std::size_t>(N + 1));

  for (int k = 0; k <= N; ++k) {
    const double t_raw = static_cast<double>(k) / static_cast<double>(N);
    // 平滑插值（前缓中匀后缓），降低速度突变
    // const double t = t_raw * t_raw * (3.0 - 2.0 * t_raw);

    std::vector<double> q(config_.joint_count);
    for (std::size_t i = 0; i < config_.joint_names.size(); ++i) {
      // const auto& jn = config_.joint_names[i];
      // auto it = req.current_joints.find(jn);
      // if (it == req.current_joints.end())
      // {
      //   LOGE("[solve_core] Joint lookup failed");
      //   return false;
      // }
      // const double target_near = ikc::wrapToNearby(req.target_joints[i],
      // it->second);
      // q[i] = req.current_joints.positions[i] +
      //        (req.target_joints[i] - req.current_joints.positions[i]) * t;
      q[i] = req.current_joints.positions[i] +
             (req.target_joints[i] - req.current_joints.positions[i]) *
                 (10 * std::pow(t_raw, 3) - 15 * std::pow(t_raw, 4) +
                  6 * std::pow(t_raw, 5));
    }

    insert_state.setJointGroupPositions(jmg, q);
    insert_state.update();

    collision::SelfCollisionCheckResult collision_result;
    if (!self_collision_detector->check(q, collision_result)) {
      LOGE("[solve_executor][joint_planner] : Collision at joint space "
           "interpolation points k={}",
           k);
      return false;
    }

    TrajectoryPoint p;
    p.positions = q;
    traj.points.push_back(std::move(p));
  }
  parameterize_time_from_start(traj, config_.max_velocity_scaling);

  const double total_duration =
      traj.points.empty() ? 0.0 : traj.points.back().time_from_start;
  for (int k = 0; k <= N; ++k) {
    auto &point = traj.points[static_cast<std::size_t>(k)];
    point.velocities.assign(point.positions.size(), 0.0);
    if (total_duration <= 1e-9) {
      continue;
    }

    const double u = static_cast<double>(k) / static_cast<double>(N);
    const double blend_derivative =
        30.0 * std::pow(u, 2) - 60.0 * std::pow(u, 3) + 30.0 * std::pow(u, 4);
    for (std::size_t i = 0; i < config_.joint_names.size(); ++i) {
      point.velocities[i] =
          (req.target_joints[i] - req.current_joints.positions[i]) *
          blend_derivative / total_duration;
    }
  }

  out_traj = std::move(traj);
  return true;
}

bool SolveExecutor::plan_cartesian(
    const SolveRequest &req, solve_executor::Trajectory &out_traj,
    std::string &err,
    const collision::SelfCollisionDetector *self_collision_detector) {
  constexpr double kEps = 1e-9;
  const auto group_name = config_.group_name;
  const auto jmg = get_jmg(robot_model_, group_name);
  const auto *ref_link = robot_model_->getLinkModel(config_.reference_link);
  const auto *ee_link = robot_model_->getLinkModel(config_.ee_link_name);

  planner::StraightPlannerSettings settings;
  settings.sample_step_m = config_.sample_step_m;
  settings.alignment_dot_threshold = config_.alignment_dot_threshold;
  settings.max_joint_jump_rad = config_.max_joint_jump_rad;
  settings.reference_link = config_.reference_link;

  moveit::core::RobotState start_state(robot_model_);
  if (!fill_joint_state_require_all(req.current_joints, jmg, start_state,
                                    err)) {
    return false;
  }
  start_state.update();

  if (!std::isfinite(req.target_length) || req.target_length <= 0.0) {
    err = "target_length is invalid";
    LOGE("[solve_executor] {}", err);
    return false;
  }

  Eigen::Vector3d target_vector = req.target_vector;
  if (!target_vector.allFinite()) {
    err = "target_vector contains non-finite values";
    LOGE("[solve_executor] {}", err);
    return false;
  }
  if (config_.use_vision_target_vector) {
    if (target_vector.norm() <= kEps) {
      err = "target_vector is a zero vector";
      LOGE("[solve_executor] {}", err);
      return false;
    }
    const Eigen::Vector3d normalized_target = target_vector.normalized();
    const Eigen::Isometry3d T_ref =
        start_state.getGlobalLinkTransform(ref_link);
    Eigen::Isometry3d T0 =
        start_state.getGlobalLinkTransform(ee_link); // 起始末端位姿
    const Eigen::Matrix3d R_ref_ee = T_ref.linear().transpose() * T0.linear();
    const Eigen::Vector3d ee_x_in_ref = R_ref_ee.col(0).normalized();
    const double direction_dot = ee_x_in_ref.dot(normalized_target);
    const double abs_direction_dot = std::abs(direction_dot);
    if (!std::isfinite(abs_direction_dot) ||
        abs_direction_dot < settings.alignment_dot_threshold) {
      LOGE("[solve_executor][straight_planner] Alignment check failed: "
           "abs(dot)={}, threshold={}",
           abs_direction_dot, settings.alignment_dot_threshold);
      return false;
    }

    const Eigen::Vector3d straight_dir =
        (direction_dot >= 0.0 ? 1.0 : -1.0) * ee_x_in_ref;
    target_vector = straight_dir;
  }

  // planner::StraightPlannerSettings settings;
  // settings.sample_step_m = config_.sample_step_m;
  // settings.alignment_dot_threshold = config_.alignment_dot_threshold;
  // settings.reference_link = config_.reference_link;
  planner::StraightPlanner planner(robot_model_, group_name,
                                   config_.ee_link_name, settings,
                                   self_collision_detector);

  calculator::CostOptions cost_opt;
  cost_opt.summary();
  // auto raw_traj =
  //     planner.plan(start_state, target_vector, req.target_length, cost_opt);
  auto raw_traj = planner.plan_with_5dof_constrain(start_state, target_vector,
                                                   req.target_length, cost_opt);
  if (!raw_traj) {
    err = err.empty() ? "Straight planner failed" : err;
    LOGE("[solve_executor] {}", err);
    return false;
  }

  out_traj = *raw_traj;
  if (!timeParameterizeTrajectory(out_traj, err)) {
    return false;
  }
  return true;
}

} // namespace solve_executor

#include "solve_core/solve_core.hpp"

#include <algorithm>
#include <cmath>
#include <unordered_map>

#include "log_utils/log.hpp"
#include "error_code_utils/app_error.hpp"

#include <Eigen/Geometry>

#include <moveit/collision_detection/collision_common.h>
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

namespace solve_core {
namespace {

Eigen::Isometry3d pose_to_isometry(const Pose &pose) {
  Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();
  iso.translation() << pose.x, pose.y, pose.z;
  Eigen::Quaterniond q(pose.qw, pose.qx, pose.qy, pose.qz);
  q.normalize();
  iso.linear() = q.toRotationMatrix();
  return iso;
}

bool fill_joint_state_allow_missing(const JointState &js,
                                    const moveit::core::JointModelGroup *jmg,
                                    moveit::core::RobotState &state) {
  if (!jmg) {
    return false;
  }
  if (js.names.empty() || js.positions.empty()) {
    return true;
  }
  const auto &group_joint_names = jmg->getVariableNames();
  for (const auto &jn : group_joint_names) {
    auto it = std::find(js.names.begin(), js.names.end(), jn);
    if (it == js.names.end()) {
      continue;
    }
    const auto idx = static_cast<std::size_t>(std::distance(js.names.begin(), it));
    if (idx >= js.positions.size()) {
      continue;
    }
    const double pos = js.positions[idx];
    state.setJointPositions(jn, &pos);
  }
  return true;
}

bool fill_joint_state_require_all(const JointState &js,
                                  const moveit::core::JointModelGroup *jmg,
                                  moveit::core::RobotState &state,
                                  std::string &err) {
  if (!jmg) {
    err = "JointModelGroup null";
    return false;
  }
  const auto &group_joint_names = jmg->getVariableNames();
  if (js.names.empty() || js.positions.empty()) {
    err = "Missing joint state";
    return false;
  }
  if (js.names.size() != js.positions.size()) {
    err = "Joint names/positions size mismatch";
    return false;
  }
  std::unordered_map<std::string, double> pos_map;
  pos_map.reserve(js.names.size());
  for (std::size_t i = 0; i < js.names.size(); ++i) {
    pos_map[js.names[i]] = js.positions[i];
  }
  for (const auto &jn : group_joint_names) {
    auto it = pos_map.find(jn);
    if (it == pos_map.end()) {
      err = "Missing joint state";
      return false;
    }
    state.setVariablePosition(it->first, it->second);
  }
  return true;
}

Trajectory trajectory_from_robot_trajectory(const moveit::core::RobotModelConstPtr &robot_model,
                                            const std::string &group_name,
                                            const robot_trajectory::RobotTrajectory &rt) {
  Trajectory out;
  if (!robot_model) {
    return out;
  }
  const auto *jmg = robot_model->getJointModelGroup(group_name);
  if (!jmg) {
    return out;
  }
  out.joint_names = jmg->getVariableNames();
  const std::size_t count = rt.getWayPointCount();
  out.points.reserve(count);

  for (std::size_t i = 0; i < count; ++i) {
    const moveit::core::RobotState &st = rt.getWayPoint(i);
    TrajectoryPoint p;
    st.copyJointGroupPositions(jmg, p.positions);
    st.copyJointGroupVelocities(jmg, p.velocities);
    p.time_from_start = rt.getWayPointDurationFromStart(i);
    out.points.push_back(std::move(p));
  }
  return out;
}

std::optional<Trajectory> time_parameterize_path(
    const moveit::core::RobotModelConstPtr &robot_model,
    const std::string &group_name,
    const std::vector<std::vector<double>> &q_path,
    const moveit::core::RobotState &start_state,
    double path_tolerance = 1.0) {
  if (!robot_model) {
    LOGE("[solve_core] RobotModel is null");
    return std::nullopt;
  }
  const auto *jmg = robot_model->getJointModelGroup(group_name);
  if (!jmg) {
    LOGE("[solve_core] JointModelGroup not found");
    return std::nullopt;
  }
  if (q_path.empty()) {
    LOGE("[solve_core] q_path is empty");
    return std::nullopt;
  }

  robot_trajectory::RobotTrajectory rt(robot_model, group_name);
  moveit::core::RobotState st = start_state;
  st.update();

  for (std::size_t i = 0; i < q_path.size(); ++i) {
    if (q_path[i].size() != jmg->getVariableCount()) {
      LOGE("[solve_core] q_path size mismatch");
      return std::nullopt;
    }
    st.setJointGroupPositions(jmg, q_path[i]);
    st.update();
    const double dt = (i == 0) ? 0.0 : 0.0;
    rt.addSuffixWayPoint(st, dt);
  }

  trajectory_processing::TimeOptimalTrajectoryGeneration totg(path_tolerance);
  if (!totg.computeTimeStamps(rt)) {
    LOGE("[solve_core] Time parameterization failed");
    return std::nullopt;
  }

  return trajectory_from_robot_trajectory(robot_model, group_name, rt);
}

} // namespace

SolveCore::SolveCore(std::shared_ptr<MoveItAdapter> adapter)
    : adapter_(std::move(adapter)) {
  log_utils::init_console_logger("solve_core");
  LOGI("[solve_core] logger init");
}

void SolveCore::set_error_bus(const std::shared_ptr<error_code_utils::ErrorBus> &bus) {
  error_bus_ = bus;
}

void SolveCore::publish_error(const error_code_utils::Error &err) const {
  if (!error_bus_) {
    return;
  }
  error_bus_->publish(err);
}

std::optional<SolveResponse> SolveCore::plan(const SolveRequest &req) {
  if (!adapter_) {
    LOGE("[solve_core] MoveIt adapter not set");
    publish_error(error_code_utils::app::make_app_error(error_code_utils::ErrorDomain::SOLVE, error_code_utils::app::SolveCode::AdapterMissing, "MoveIt adapter not set"));
    return std::nullopt;
  }

  switch (req.option) {
  case PlanOption::NORMAL:
    return plan_normal(req);
  case PlanOption::CARTESIAN:
    return plan_cartesian(req);
  case PlanOption::JOINTS:
    return plan_joints(req);
  default:
    LOGE("[solve_core] Unknown planning option");
    publish_error(error_code_utils::app::make_app_error(error_code_utils::ErrorDomain::SOLVE, error_code_utils::app::SolveCode::UnknownOption, "Unknown planning option"));
    return std::nullopt;
  }
}

std::optional<SolveResponse> SolveCore::plan_normal(const SolveRequest &req) {
  const auto robot_model = adapter_->robot_model();
  if (!robot_model) {
    LOGE("[solve_core] RobotModel is null");
    publish_error(error_code_utils::app::make_app_error(error_code_utils::ErrorDomain::SOLVE, error_code_utils::app::SolveCode::RobotModelMissing, "RobotModel is null"));
    return std::nullopt;
  }

  const std::string group_name = !req.group_name.empty()
                                     ? req.group_name
                                     : adapter_->group_name();
  const auto *jmg = adapter_->joint_model_group(group_name);
  if (!jmg) {
    LOGE("[solve_core] JointModelGroup not found");
    publish_error(error_code_utils::app::make_app_error(error_code_utils::ErrorDomain::SOLVE, error_code_utils::app::SolveCode::JointModelGroupMissing, "JointModelGroup not found",
                             {{"group_name", group_name}}));
    return std::nullopt;
  }

  auto solver = jmg->getSolverInstance();
  if (!solver) {
    LOGE("[solve_core] IK solver missing");
    publish_error(error_code_utils::app::make_app_error(error_code_utils::ErrorDomain::SOLVE, error_code_utils::app::SolveCode::IkSolverMissing, "IK solver missing",
                             {{"group_name", group_name}}));
    return std::nullopt;
  }

  moveit::core::RobotState start_state(robot_model);
  start_state.setToDefaultValues();
  fill_joint_state_allow_missing(req.current_joints, jmg, start_state);
  start_state.update();

  if (!start_state.satisfiesBounds(jmg)) {
    start_state.enforceBounds(jmg);
    LOGE("[solve_core] Start state out of bounds");
    publish_error(error_code_utils::app::make_app_error(error_code_utils::ErrorDomain::SOLVE, error_code_utils::app::SolveCode::StartStateInvalid, "Start state out of bounds",
                             {{"group_name", group_name}}));
    return std::nullopt;
  }

  adapter_->set_start_state(start_state);

  std::string ee_link = !req.ee_link.empty() ? req.ee_link : adapter_->end_effector_link();
  if (ee_link.empty()) {
    const auto &links = jmg->getLinkModelNames();
    if (!links.empty()) {
      ee_link = links.back();
    }
  }

  Eigen::Isometry3d target_iso = pose_to_isometry(req.target_pose);
  moveit::core::RobotState ik_state(start_state);
  bool ik_ok = ik_state.setFromIK(jmg, target_iso, ee_link, 2.0);
  if (!ik_ok) {
    const Eigen::Isometry3d current_fk = start_state.getGlobalLinkTransform(ee_link);
    ik_ok = ik_state.setFromIK(jmg, current_fk, ee_link, 2.0);
    if (!ik_ok) {
      LOGE("[solve_core] IK failed");
      publish_error(error_code_utils::app::make_app_error(error_code_utils::ErrorDomain::SOLVE, error_code_utils::app::SolveCode::IkFailed, "IK failed",
                               {{"group_name", group_name}, {"ee_link", ee_link}}));
      return std::nullopt;
    }
  }

  std::vector<double> q_target;
  ik_state.copyJointGroupPositions(jmg, q_target);
  if (q_target.size() != jmg->getVariableCount()) {
    LOGE("[solve_core] IK variables size mismatch");
    publish_error(error_code_utils::app::make_app_error(error_code_utils::ErrorDomain::SOLVE, error_code_utils::app::SolveCode::TargetSizeMismatch, "IK variables size mismatch",
                             {{"group_name", group_name}}));
    return std::nullopt;
  }

  const auto joint_names = jmg->getVariableNames();
  auto plan_res = adapter_->plan_to_joint_target(joint_names, q_target, req.planner);
  if (!plan_res) {
    LOGE("[solve_core] plan_to_joint_target failed");
    publish_error(error_code_utils::app::make_app_error(error_code_utils::ErrorDomain::SOLVE, error_code_utils::app::SolveCode::PlanFailed, "plan_to_joint_target failed",
                             {{"group_name", group_name}}));
    return std::nullopt;
  }

  SolveResponse resp;
  resp.trajectory = std::move(*plan_res);
  return resp;
}

std::optional<SolveResponse> SolveCore::plan_cartesian(const SolveRequest &req) {
  const auto robot_model = adapter_->robot_model();
  if (!robot_model) {
    LOGE("[solve_core] RobotModel is null");
    publish_error(error_code_utils::app::make_app_error(error_code_utils::ErrorDomain::SOLVE, error_code_utils::app::SolveCode::RobotModelMissing, "RobotModel is null"));
    return std::nullopt;
  }
  const std::string group_name = !req.group_name.empty()
                                     ? req.group_name
                                     : adapter_->group_name();
  const auto *jmg = adapter_->joint_model_group(group_name);
  if (!jmg) {
    LOGE("[solve_core] JointModelGroup not found");
    publish_error(error_code_utils::app::make_app_error(error_code_utils::ErrorDomain::SOLVE, error_code_utils::app::SolveCode::JointModelGroupMissing, "JointModelGroup not found",
                             {{"group_name", group_name}}));
    return std::nullopt;
  }

  moveit::core::RobotState kinematic_state(robot_model);
  std::string err;
  if (!fill_joint_state_require_all(req.current_joints, jmg, kinematic_state, err)) {
    LOGE("[solve_core] {}", err);
    publish_error(error_code_utils::app::make_app_error(error_code_utils::ErrorDomain::SOLVE, error_code_utils::app::SolveCode::JointStateMissing, err,
                             {{"group_name", group_name}}));
    return std::nullopt;
  }
  kinematic_state.update();

  std::string ee_link = !req.ee_link.empty() ? req.ee_link : adapter_->end_effector_link();
  if (ee_link.empty()) {
    ee_link = "link6";
  }

  const auto *ee_link_model = robot_model->getLinkModel(ee_link);
  if (!ee_link_model) {
    LOGE("[solve_core] End-effector link missing");
    publish_error(error_code_utils::app::make_app_error(error_code_utils::ErrorDomain::SOLVE, error_code_utils::app::SolveCode::InvalidRequest, "End-effector link missing",
                             {{"ee_link", ee_link}}));
    return std::nullopt;
  }
  const Eigen::Isometry3d start_pose = kinematic_state.getGlobalLinkTransform(ee_link_model);
  const Eigen::Isometry3d target_pose = pose_to_isometry(req.target_pose);

  const int num_points = 50;
  std::vector<Eigen::Isometry3d> waypoints;
  waypoints.reserve(static_cast<std::size_t>(num_points));

  Eigen::Quaterniond q_start(start_pose.rotation());
  Eigen::Quaterniond q_end(target_pose.rotation());
  q_start.normalize();
  q_end.normalize();

  for (int i = 1; i <= num_points; ++i) {
    const double ratio = static_cast<double>(i) / num_points;
    Eigen::Isometry3d interp = Eigen::Isometry3d::Identity();
    interp.translation() = start_pose.translation() +
                           (target_pose.translation() - start_pose.translation()) * ratio;
    Eigen::Quaterniond q_interp = q_start.slerp(ratio, q_end);
    interp.linear() = q_interp.toRotationMatrix();
    waypoints.push_back(interp);
  }

  std::vector<double> q_prev;
  kinematic_state.copyJointGroupPositions(jmg, q_prev);

  std::vector<std::vector<double>> q_path;
  q_path.reserve(waypoints.size());

  moveit::core::RobotState st(robot_model);
  st.setJointGroupPositions(jmg, q_prev);
  st.update();

  for (std::size_t i = 0; i < waypoints.size(); ++i) {
    if (!st.setFromIK(jmg, waypoints[i], ee_link, 0.02)) {
      LOGE("[solve_core] IK failed on waypoint");
      publish_error(error_code_utils::app::make_app_error(error_code_utils::ErrorDomain::SOLVE, error_code_utils::app::SolveCode::IkFailed, "IK failed on waypoint",
                               {{"group_name", group_name}, {"ee_link", ee_link}}));
      return std::nullopt;
    }
    std::vector<double> q_i;
    st.copyJointGroupPositions(jmg, q_i);
    q_path.push_back(q_i);
    q_prev = q_i;
  }

  auto traj_res = time_parameterize_path(robot_model, group_name, q_path, kinematic_state);
  if (!traj_res) {
    publish_error(error_code_utils::app::make_app_error(error_code_utils::ErrorDomain::SOLVE, error_code_utils::app::SolveCode::TimeParameterizationFailed,
                             "Time parameterization failed",
                             {{"group_name", group_name}}));
    return std::nullopt;
  }

  SolveResponse resp;
  resp.trajectory = std::move(*traj_res);
  resp.joint_path = std::move(q_path);
  return resp;
}

std::optional<SolveResponse> SolveCore::plan_joints(const SolveRequest &req) {
  const auto robot_model = adapter_->robot_model();
  if (!robot_model) {
    LOGE("[solve_core] RobotModel is null");
    publish_error(error_code_utils::app::make_app_error(error_code_utils::ErrorDomain::SOLVE, error_code_utils::app::SolveCode::RobotModelMissing, "RobotModel is null"));
    return std::nullopt;
  }
  const std::string group_name = !req.group_name.empty()
                                     ? req.group_name
                                     : adapter_->group_name();
  const auto *jmg = adapter_->joint_model_group(group_name);
  if (!jmg) {
    LOGE("[solve_core] JointModelGroup not found");
    publish_error(error_code_utils::app::make_app_error(error_code_utils::ErrorDomain::SOLVE, error_code_utils::app::SolveCode::JointModelGroupMissing, "JointModelGroup not found",
                             {{"group_name", group_name}}));
    return std::nullopt;
  }

  const auto &group_joint_names = jmg->getVariableNames();
  const std::size_t dof = group_joint_names.size();
  if (req.target_joints.size() < dof) {
    LOGE("[solve_core] Target joints size mismatch");
    publish_error(error_code_utils::app::make_app_error(error_code_utils::ErrorDomain::SOLVE, error_code_utils::app::SolveCode::TargetSizeMismatch, "Target joints size mismatch",
                             {{"group_name", group_name}}));
    return std::nullopt;
  }

  moveit::core::RobotState start_state(robot_model);
  start_state.setToDefaultValues();
  std::string err;
  if (!fill_joint_state_require_all(req.current_joints, jmg, start_state, err)) {
    LOGE("[solve_core] {}", err);
    publish_error(error_code_utils::app::make_app_error(error_code_utils::ErrorDomain::SOLVE, error_code_utils::app::SolveCode::JointStateMissing, err,
                             {{"group_name", group_name}}));
    return std::nullopt;
  }
  start_state.update();

  if (!start_state.satisfiesBounds(jmg)) {
    start_state.enforceBounds(jmg);
  }

  std::unordered_map<std::string, double> start_joint_position;
  start_joint_position.reserve(dof);
  for (const auto &jn : group_joint_names) {
    const double *pos_ptr = start_state.getJointPositions(jn);
    const double pos_now = pos_ptr ? *pos_ptr : 0.0;
    start_joint_position.emplace(jn, pos_now);
  }

  double max_step_rad = 0.05;
  double max_delta = 0.0;
  for (std::size_t i = 0; i < dof; ++i) {
    const auto &jn = group_joint_names[i];
    auto it = start_joint_position.find(jn);
    if (it == start_joint_position.end()) {
      LOGE("[solve_core] Joint lookup failed");
      publish_error(error_code_utils::app::make_app_error(error_code_utils::ErrorDomain::SOLVE, error_code_utils::app::SolveCode::JointLookupFailed, "Joint lookup failed",
                               {{"group_name", group_name}}));
      return std::nullopt;
    }
    max_delta = std::max(max_delta, std::fabs(req.target_joints[i] - it->second));
  }
  const int N = std::max(1, static_cast<int>(std::ceil(max_delta / std::max(1e-6, max_step_rad))));

  Trajectory traj;
  traj.joint_names = group_joint_names;
  traj.points.reserve(static_cast<std::size_t>(N + 1));

  moveit::core::RobotState rs = start_state;

  for (int k = 0; k <= N; ++k) {
    const double t = static_cast<double>(k) / static_cast<double>(N);
    std::vector<double> q(dof);
    for (std::size_t i = 0; i < dof; ++i) {
      const auto &jn = group_joint_names[i];
      auto it = start_joint_position.find(jn);
      if (it == start_joint_position.end()) {
        LOGE("[solve_core] Joint lookup failed");
        publish_error(error_code_utils::app::make_app_error(error_code_utils::ErrorDomain::SOLVE, error_code_utils::app::SolveCode::JointLookupFailed, "Joint lookup failed",
                                 {{"group_name", group_name}}));
        return std::nullopt;
      }
      q[i] = it->second + (req.target_joints[i] - it->second) * t;
    }

    rs.setJointGroupPositions(jmg, q);
    rs.update();

    std::string collision_err;
    if (!adapter_->check_self_collision(rs, group_name, collision_err)) {
      if (collision_err.empty())
        collision_err = "Self collision detected";
      LOGE("[solve_core] {}", collision_err);
      publish_error(error_code_utils::app::make_app_error(error_code_utils::ErrorDomain::SOLVE, error_code_utils::app::SolveCode::CollisionDetected, collision_err,
                               {{"group_name", group_name}}));
      return std::nullopt;
    }

    TrajectoryPoint p;
    p.positions = q;
    p.time_from_start = 0.05 * k;
    traj.points.push_back(std::move(p));
  }

  SolveResponse resp;
  resp.trajectory = std::move(traj);
  return resp;
}

} // namespace solve_core

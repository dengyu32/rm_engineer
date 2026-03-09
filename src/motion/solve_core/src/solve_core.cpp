#include "solve_core/solve_core.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <sstream>
#include <unordered_map>

#include "log_utils/log.hpp"
#include "solve_core/calculate_tools/wrap.hpp"
#include "solve_core/planner/limit_planner.hpp"
#include "solve_core/planner/straight_planner.hpp"

#include <Eigen/Geometry>

#include <moveit/collision_detection/collision_common.h>
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

namespace solve_core {
namespace {

const char *to_string(SolveCore::SolveCode code) {
  switch (code) {
  case SolveCore::SolveCode::AdapterMissing:
    return "AdapterMissing";
  case SolveCore::SolveCode::RobotModelMissing:
    return "RobotModelMissing";
  case SolveCore::SolveCode::JointModelGroupMissing:
    return "JointModelGroupMissing";
  case SolveCore::SolveCode::IkSolverMissing:
    return "IkSolverMissing";
  case SolveCore::SolveCode::StartStateInvalid:
    return "StartStateInvalid";
  case SolveCore::SolveCode::InvalidRequest:
    return "InvalidRequest";
  case SolveCore::SolveCode::TargetSizeMismatch:
    return "TargetSizeMismatch";
  case SolveCore::SolveCode::JointStateMissing:
    return "JointStateMissing";
  case SolveCore::SolveCode::CollisionDetected:
    return "CollisionDetected";
  case SolveCore::SolveCode::JointLookupFailed:
    return "JointLookupFailed";
  case SolveCore::SolveCode::UnknownOption:
    return "UnknownOption";
  default:
    return "Unknown";
  }
}

std::string format_context(const std::map<std::string, std::string> &ctx) {
  if (ctx.empty()) {
    return "";
  }
  std::ostringstream oss;
  bool first = true;
  for (const auto &kv : ctx) {
    if (!first) {
      oss << ", ";
    }
    first = false;
    oss << kv.first << "=" << kv.second;
  }
  return oss.str();
}

// _四元数转变换矩阵
Eigen::Isometry3d pose_to_isometry(const Pose &pose) {
  Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();
  iso.translation() << pose.x, pose.y, pose.z;
  Eigen::Quaterniond q(pose.qw, pose.qx, pose.qy, pose.qz);
  q.normalize();
  iso.linear() = q.toRotationMatrix();
  return iso;
}

// _坐标轴向量归一化
bool parse_direction_vector(const std::array<double, 3> &direction, Eigen::Vector3d &dir,
                            std::string &err) {
  constexpr double kEps = 1e-9;   //eps：epsilon，一个非常小的数，用于数值计算中避免除以零或判断数值是否接近零的情况
  dir = Eigen::Vector3d(direction[0], direction[1], direction[2]);
  const double norm = dir.norm();   //norm：向量的模长
  if (!std::isfinite(norm) || norm <= kEps) {
    err = "Cartesian request direction vector is invalid";
    return false;
  }
  dir /= norm;
  return true;
}

// 将关节角写进robotstate，可以缺失
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

// 将关节角写进robotstate，必须齐全
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

// 将Moveit自带的Trajectory转为自己定义的Trajectory
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

void parameterize_time_from_start(Trajectory &traj, double velocity_scaling) {
  if (traj.points.empty()) {
    return;
  }

  constexpr double kNominalJointSpeedRadPerSec = 1.0;
  constexpr double kMinDtSec = 0.02;
  const double scale = std::clamp(velocity_scaling, 0.05, 1.0);

  traj.points[0].time_from_start = 0.0;
  traj.points[0].velocities.assign(traj.points[0].positions.size(), 0.0);

  for (std::size_t i = 1; i < traj.points.size(); ++i) {
    const auto &prev = traj.points[i - 1];
    auto &curr = traj.points[i];
    const std::size_t dof = std::min(prev.positions.size(), curr.positions.size());

    double max_delta = 0.0;
    for (std::size_t j = 0; j < dof; ++j) {
      max_delta = std::max(max_delta, std::fabs(curr.positions[j] - prev.positions[j]));
    }

    const double dt = std::max(kMinDtSec, max_delta / (kNominalJointSpeedRadPerSec * scale));
    curr.time_from_start = prev.time_from_start + dt;

    curr.velocities.assign(curr.positions.size(), 0.0);
    if (dt > 1e-9) {
      for (std::size_t j = 0; j < dof; ++j) {
        curr.velocities[j] = (curr.positions[j] - prev.positions[j]) / dt;
      }
    }
  }
}

} // namespace

// 构造函数，初始化日志
SolveCore::SolveCore(std::shared_ptr<MoveItAdapter> adapter,
                     const SolveCoreConfig &config)
    : adapter_(std::move(adapter)), config_(config) {
  log_utils::init_console_logger("solve_core");
  LOGI("[solve_core] logger init");
}

void SolveCore::publish_error(
    SolveCode code, const std::string &message,
    const std::map<std::string, std::string> &context) const {
  const auto ctx = format_context(context);
  if (ctx.empty()) {
    LOGE("[solve_core][error][code={}][name={}] {}", static_cast<int>(code),
         to_string(code), message);
    return;
  }
  LOGE("[solve_core][error][code={}][name={}] {} [{}]", static_cast<int>(code),
       to_string(code), message, ctx);
}

// 规划总入口
std::optional<SolveResponse> SolveCore::plan(const SolveRequest &req, std::string &err) {
  if (!adapter_) {
    LOGE("[solve_core] MoveIt adapter not set");
    publish_error(SolveCode::AdapterMissing, "MoveIt adapter not set");
    return std::nullopt;
  }

  switch (req.option) {
  case PlanOption::NORMAL:
    
    //加采样的话，这里要加一个if条件
    
    return plan_normal(req,err);
  case PlanOption::CARTESIAN:
    return plan_cartesian(req,err);
  case PlanOption::JOINTS:
    return plan_joints(req,err);
  default:
    LOGE("[solve_core] Unknown planning option");
    publish_error(SolveCode::UnknownOption, "Unknown planning option");
    return std::nullopt;
  }
}

// 使用ompl采样规划
std::optional<SolveResponse> SolveCore::plan_normal(const SolveRequest &req, std::string &err) {
  const auto robot_model = adapter_->robot_model();
  if (!robot_model) {
    LOGE("[solve_core] RobotModel is null");
    publish_error(SolveCode::RobotModelMissing, "RobotModel is null");
    return std::nullopt;
  }

  const std::string group_name = !req.group_name.empty()
                                     ? req.group_name
                                     : adapter_->group_name();
  const auto *jmg = adapter_->joint_model_group(group_name);
  if (!jmg) {
    LOGE("[solve_core] JointModelGroup not found");
    publish_error(SolveCode::JointModelGroupMissing, "JointModelGroup not found",
                  {{"group_name", group_name}});
    return std::nullopt;
  }

  auto solver = jmg->getSolverInstance();
  if (!solver) {
    LOGE("[solve_core] IK solver missing");
    publish_error(SolveCode::IkSolverMissing, "IK solver missing",
                  {{"group_name", group_name}});
    return std::nullopt;
  }

  moveit::core::RobotState start_state(robot_model);
  start_state.setToDefaultValues();
  fill_joint_state_allow_missing(req.current_joints, jmg, start_state);
  start_state.update();

  if (!start_state.satisfiesBounds(jmg)) {
    start_state.enforceBounds(jmg);
    LOGE("[solve_core] Start state out of bounds");
    publish_error(SolveCode::StartStateInvalid, "Start state out of bounds",
                  {{"group_name", group_name}});
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

  LimitPlannerOptions limit_opt;
  limit_opt.sampling_mode = config_.limit_enable_target_pose_sampling
                                ? SamplingMode::ROLL_SAMPLE
                                : SamplingMode::NORMAL;
  limit_opt.enable_target_pose_sampling = config_.limit_enable_target_pose_sampling;
  limit_opt.roll_samples = config_.limit_roll_samples;
  limit_opt.roll_range_rad = config_.limit_roll_range_rad;
  limit_opt.top_k_after_ik = config_.limit_top_k_after_ik;
  limit_opt.orientation_weight = config_.limit_orientation_weight;
  limit_opt.ik_distance_weight = config_.limit_ik_distance_weight;
  limit_opt.joint_motion_weight = config_.limit_joint_motion_weight;

  PlannerConfigs planner_cfg;
  planner_cfg.goal_position_tolerance = config_.goal_position_tolerance;
  planner_cfg.goal_orientation_tolerance = config_.goal_orientation_tolerance;
  planner_cfg.planning_time = config_.planning_time;
  planner_cfg.num_planning_attempts = config_.num_planning_attempts;
  planner_cfg.max_velocity_scaling = config_.max_velocity_scaling;
  planner_cfg.max_acc_scaling = config_.max_acc_scaling;

  std::shared_ptr<LimitPlanner> planner = std::make_shared<LimitPlanner>(adapter_);
  auto out_traj = planner->plan(jmg, ee_link, start_state, target_iso, limit_opt, err, planner_cfg);

  auto resp = std::make_optional<SolveResponse>();
  if (out_traj) {
    resp->trajectory = std::move(*out_traj);
  } else {
    
    return std::nullopt;
  }
  return resp;
}

// 直线规划
std::optional<SolveResponse>
SolveCore::plan_cartesian(const SolveRequest &req, std::string &err) {

  auto robot_model = adapter_->robot_model();
  std::string group_name =
      req.group_name.empty() ?
      adapter_->group_name() :
      req.group_name;

  moveit::core::RobotState start_state(robot_model);
  if (!fill_joint_state_require_all(
          req.current_joints,
          adapter_->joint_model_group(group_name),
          start_state, err))
    return std::nullopt;

  start_state.update();

  std::string ee_link =
      req.ee_link.empty() ?
      adapter_->end_effector_link() :
      req.ee_link;

  Eigen::Vector3d direction;
  if (!parse_direction_vector(req.target_direction, direction, err)) {
    LOGE("[solve_core] {}", err);
    publish_error(SolveCode::InvalidRequest, err);
    return std::nullopt;
  }

  StraightPlanner planner(robot_model, group_name, ee_link);
  StraightPlannerOptions opt;
  opt.num_waypoints = config_.cartesian_num_waypoints;
  opt.use_directional_sampling = true;
  opt.sample_step_m = config_.cartesian_sample_step_m;
  opt.direction_x = direction.x();
  opt.direction_y = direction.y();
  opt.direction_z = direction.z();

  auto traj = planner.plan(
      start_state,
      pose_to_isometry(req.target_pose),
      opt,
      config_.straight_cost_options);

  if (!traj)
    return std::nullopt;

  SolveResponse resp;
  resp.trajectory = std::move(*traj);
  parameterize_time_from_start(resp.trajectory, config_.max_velocity_scaling);
  return resp;
}


// 关节空间规划
std::optional<SolveResponse> SolveCore::plan_joints(const SolveRequest &req, std::string &err) {
  const auto robot_model = adapter_->robot_model();
  if (!robot_model) {
    LOGE("[solve_core] RobotModel is null");
    publish_error(SolveCode::RobotModelMissing, "RobotModel is null");
    return std::nullopt;
  }
  const std::string group_name = !req.group_name.empty()
                                     ? req.group_name
                                     : adapter_->group_name();
  const auto *jmg = adapter_->joint_model_group(group_name);
  if (!jmg) {
    LOGE("[solve_core] JointModelGroup not found");
    publish_error(SolveCode::JointModelGroupMissing, "JointModelGroup not found",
                  {{"group_name", group_name}});
    return std::nullopt;
  }

  const auto &group_joint_names = jmg->getVariableNames();
  const std::size_t dof = group_joint_names.size();
  if (req.target_joints.size() < dof) {
    LOGE("[solve_core] Target joints size mismatch");
    publish_error(SolveCode::TargetSizeMismatch, "Target joints size mismatch",
                  {{"group_name", group_name}});
    return std::nullopt;
  }

  moveit::core::RobotState start_state(robot_model);
  start_state.setToDefaultValues();
  if (!fill_joint_state_require_all(req.current_joints, jmg, start_state, err)) {
    LOGE("[solve_core] {}", err);
    publish_error(SolveCode::JointStateMissing, err, {{"group_name", group_name}});
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

  double max_step_rad = config_.joints_max_step_rad;
  double max_delta = 0.0;
  for (std::size_t i = 0; i < dof; ++i) {
    const auto &jn = group_joint_names[i];
    auto it = start_joint_position.find(jn);
    if (it == start_joint_position.end()) {
      LOGE("[solve_core] Joint lookup failed");
      publish_error(SolveCode::JointLookupFailed, "Joint lookup failed",
                    {{"group_name", group_name}});
      return std::nullopt;
    }
    const double target_near = ikc::wrapToNearby(req.target_joints[i], it->second);
    max_delta = std::max(max_delta, std::fabs(target_near - it->second));
  }
  const int N = std::max(1, static_cast<int>(std::ceil(max_delta / std::max(1e-6, max_step_rad))));

  Trajectory traj;
  traj.joint_names = group_joint_names;
  traj.points.reserve(static_cast<std::size_t>(N + 1));

  moveit::core::RobotState rs = start_state;

  for (int k = 0; k <= N; ++k) {
    const double t_raw = static_cast<double>(k) / static_cast<double>(N);
    // 平滑插值（前缓中匀后缓），降低速度突变
    const double t = t_raw * t_raw * (3.0 - 2.0 * t_raw);
    std::vector<double> q(dof);
    for (std::size_t i = 0; i < dof; ++i) {
      const auto &jn = group_joint_names[i];
      auto it = start_joint_position.find(jn);
      if (it == start_joint_position.end()) {
        LOGE("[solve_core] Joint lookup failed");
        publish_error(SolveCode::JointLookupFailed, "Joint lookup failed",
                      {{"group_name", group_name}});
        return std::nullopt;
      }
      const double target_near = ikc::wrapToNearby(req.target_joints[i], it->second);
      q[i] = it->second + (target_near - it->second) * t;
    }

    rs.setJointGroupPositions(jmg, q);
    rs.update();

    std::string collision_err;
    if (!adapter_->check_self_collision(rs, group_name, collision_err)) {
      if (collision_err.empty())
        collision_err = "Self collision detected";
      LOGE("[solve_core] {}", collision_err);
      publish_error(SolveCode::CollisionDetected, collision_err,
                    {{"group_name", group_name}});
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

} // namespace solve_core

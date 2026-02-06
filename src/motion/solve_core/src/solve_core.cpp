#include "solve_core/solve_core.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <unordered_map>

#include "log_utils/log.hpp"
#include "solve_core/error_code.hpp"
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

// 四元数转变换矩阵
Eigen::Isometry3d pose_to_isometry(const Pose &pose) {
  Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();
  iso.translation() << pose.x, pose.y, pose.z;
  Eigen::Quaterniond q(pose.qw, pose.qx, pose.qy, pose.qz);
  q.normalize();
  iso.linear() = q.toRotationMatrix();
  return iso;
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

} // namespace

// 构造函数，初始化日志
SolveCore::SolveCore(std::shared_ptr<MoveItAdapter> adapter)
    : adapter_(std::move(adapter)) {
  log_utils::init_console_logger("solve_core");
  LOGI("[solve_core] logger init");
}

// 错误发布通道
void SolveCore::set_error_bus(const std::shared_ptr<error_code_utils::ErrorBus> &bus) {
  error_bus_ = bus;
}

// 发布错误
void SolveCore::publish_error(const error_code_utils::Error &err) const {
  if (!error_bus_) {
    return;
  }
  error_bus_->publish(err);
}

// 规划总入口
std::optional<SolveResponse> SolveCore::plan(const SolveRequest &req, std::string &err) {
  if (!adapter_) {
    LOGE("[solve_core] MoveIt adapter not set");
    publish_error(error_code_utils::app::make_app_error(error_code_utils::ErrorDomain::SOLVE, error_code_utils::app::SolveCode::AdapterMissing, "MoveIt adapter not set"));
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
    publish_error(error_code_utils::app::make_app_error(error_code_utils::ErrorDomain::SOLVE, error_code_utils::app::SolveCode::UnknownOption, "Unknown planning option"));
    return std::nullopt;
  }
}

// 使用ompl采样规划
std::optional<SolveResponse> SolveCore::plan_normal(const SolveRequest &req, std::string &err) {
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

  std::shared_ptr<LimitPlanner> planner = std::make_shared<LimitPlanner>(adapter_);
  auto out_traj = planner->plan(jmg, ee_link, start_state, target_iso, LimitPlannerOptions(), err, PlannerConfigs());

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

  StraightPlanner planner(robot_model, group_name, ee_link);
  StraightPlannerOptions opt;

  auto traj = planner.plan(
      start_state,
      pose_to_isometry(req.target_pose),
      opt);

  if (!traj)
    return std::nullopt;

  SolveResponse resp;
  resp.trajectory = std::move(*traj);
  return resp;
}


// 关节空间规划
std::optional<SolveResponse> SolveCore::plan_joints(const SolveRequest &req, std::string &err) {
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

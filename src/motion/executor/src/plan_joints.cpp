#include "solve_core/solve_core.hpp"

#include <algorithm>
#include <cmath>
#include <unordered_map>

#include "solve_core/error_code.hpp"

#include <Eigen/Geometry>

#include <moveit/collision_detection/collision_common.h>
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

// namespace solve_core {
// namespace {

// Eigen::Isometry3d pose_to_isometry(const Pose &pose) {
//   Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();
//   iso.translation() << pose.x, pose.y, pose.z;
//   Eigen::Quaterniond q(pose.qw, pose.qx, pose.qy, pose.qz);
//   q.normalize();
//   iso.linear() = q.toRotationMatrix();
//   return iso;
// }

// bool fill_joint_state_allow_missing(const JointState &js,
//                                     const moveit::core::JointModelGroup *jmg,
//                                     moveit::core::RobotState &state) {
//   if (!jmg) {
//     return false;
//   }
//   if (js.names.empty() || js.positions.empty()) {
//     return true;
//   }
//   const auto &group_joint_names = jmg->getVariableNames();
//   for (const auto &jn : group_joint_names) {
//     auto it = std::find(js.names.begin(), js.names.end(), jn);
//     if (it == js.names.end()) {
//       continue;
//     }
//     const auto idx = static_cast<std::size_t>(std::distance(js.names.begin(), it));
//     if (idx >= js.positions.size()) {
//       continue;
//     }
//     const double pos = js.positions[idx];
//     state.setJointPositions(jn, &pos);
//   }
//   return true;
// }

// bool fill_joint_state_require_all(const JointState &js,
//                                   const moveit::core::JointModelGroup *jmg,
//                                   moveit::core::RobotState &state,
//                                   std::string &err) {
//   if (!jmg) {
//     err = "JointModelGroup null";
//     return false;
//   }
//   const auto &group_joint_names = jmg->getVariableNames();
//   if (js.names.empty() || js.positions.empty()) {
//     err = "Missing joint state";
//     return false;
//   }
//   if (js.names.size() != js.positions.size()) {
//     err = "Joint names/positions size mismatch";
//     return false;
//   }
//   std::unordered_map<std::string, double> pos_map;
//   pos_map.reserve(js.names.size());
//   for (std::size_t i = 0; i < js.names.size(); ++i) {
//     pos_map[js.names[i]] = js.positions[i];
//   }
//   for (const auto &jn : group_joint_names) {
//     auto it = pos_map.find(jn);
//     if (it == pos_map.end()) {
//       err = "Missing joint state";
//       return false;
//     }
//     state.setVariablePosition(it->first, it->second);
//   }
//   return true;
// }

// Trajectory trajectory_from_robot_trajectory(const moveit::core::RobotModelConstPtr &robot_model,
//                                             const std::string &group_name,
//                                             const robot_trajectory::RobotTrajectory &rt) {
//   Trajectory out;
//   if (!robot_model) {
//     return out;
//   }
//   const auto *jmg = robot_model->getJointModelGroup(group_name);
//   if (!jmg) {
//     return out;
//   }
//   out.joint_names = jmg->getVariableNames();
//   const std::size_t count = rt.getWayPointCount();
//   out.points.reserve(count);

//   for (std::size_t i = 0; i < count; ++i) {
//     const moveit::core::RobotState &st = rt.getWayPoint(i);
//     TrajectoryPoint p;
//     st.copyJointGroupPositions(jmg, p.positions);
//     st.copyJointGroupVelocities(jmg, p.velocities);
//     p.time_from_start = rt.getWayPointDurationFromStart(i);
//     out.points.push_back(std::move(p));
//   }
//   return out;
// }

// std::optional<Trajectory> time_parameterize_path(
//     const moveit::core::RobotModelConstPtr &robot_model,
//     const std::string &group_name,
//     const std::vector<std::vector<double>> &q_path,
//     const moveit::core::RobotState &start_state,
//     const rclcpp::Logger &logger,
//     double path_tolerance = 1.0) {
//   if (!robot_model) {
//     RCLCPP_ERROR(logger, "[solve_core] RobotModel is null");
//     return std::nullopt;
//   }
//   const auto *jmg = robot_model->getJointModelGroup(group_name);
//   if (!jmg) {
//     RCLCPP_ERROR(logger, "[solve_core] JointModelGroup not found");
//     return std::nullopt;
//   }
//   if (q_path.empty()) {
//     RCLCPP_ERROR(logger, "[solve_core] q_path is empty");
//     return std::nullopt;
//   }

//   robot_trajectory::RobotTrajectory rt(robot_model, group_name);
//   moveit::core::RobotState st = start_state;
//   st.update();

//   for (std::size_t i = 0; i < q_path.size(); ++i) {
//     if (q_path[i].size() != jmg->getVariableCount()) {
//       RCLCPP_ERROR(logger, "[solve_core] q_path size mismatch");
//       return std::nullopt;
//     }
//     st.setJointGroupPositions(jmg, q_path[i]);
//     st.update();
//     const double dt = (i == 0) ? 0.0 : 0.0;
//     rt.addSuffixWayPoint(st, dt);
//   }

//   trajectory_processing::TimeOptimalTrajectoryGeneration totg(path_tolerance);
//   if (!totg.computeTimeStamps(rt)) {
//     RCLCPP_ERROR(logger, "[solve_core] Time parameterization failed");
//     return std::nullopt;
//   }

//   return trajectory_from_robot_trajectory(robot_model, group_name, rt);
// }

// } // namespace



// std::optional<SolveResponse> SolveCore::plan_joints(const SolveRequest &req) {
//   const auto robot_model = adapter_->robot_model();
//   if (!robot_model) {
//     RCLCPP_ERROR(logger_, "[solve_core] RobotModel is null");
//     publish_error(make_error(SolveErrc::RobotModelMissing, "RobotModel is null"));
//     return std::nullopt;
//   }
//   const std::string group_name = !req.group_name.empty()
//                                      ? req.group_name
//                                      : adapter_->group_name();
//   const auto *jmg = adapter_->joint_model_group(group_name);
//   if (!jmg) {
//     RCLCPP_ERROR(logger_, "[solve_core] JointModelGroup not found");
//     publish_error(make_error(SolveErrc::JointModelGroupMissing, "JointModelGroup not found",
//                              {{"group_name", group_name}}));
//     return std::nullopt;
//   }

//   const auto &group_joint_names = jmg->getVariableNames();
//   const std::size_t dof = group_joint_names.size();
//   if (req.target_joints.size() < dof) {
//     RCLCPP_ERROR(logger_, "[solve_core] Target joints size mismatch");
//     publish_error(make_error(SolveErrc::TargetSizeMismatch, "Target joints size mismatch",
//                              {{"group_name", group_name}}));
//     return std::nullopt;
//   }

//   moveit::core::RobotState start_state(robot_model);
//   start_state.setToDefaultValues();
//   std::string err;
//   if (!fill_joint_state_require_all(req.current_joints, jmg, start_state, err)) {
//     RCLCPP_ERROR(logger_, "[solve_core] %s", err.c_str());
//     publish_error(make_error(SolveErrc::JointStateMissing, err,
//                              {{"group_name", group_name}}));
//     return std::nullopt;
//   }
//   start_state.update();

//   if (!start_state.satisfiesBounds(jmg)) {
//     start_state.enforceBounds(jmg);
//   }

//   std::unordered_map<std::string, double> start_joint_position;
//   start_joint_position.reserve(dof);
//   for (const auto &jn : group_joint_names) {
//     const double *pos_ptr = start_state.getJointPositions(jn);
//     const double pos_now = pos_ptr ? *pos_ptr : 0.0;
//     start_joint_position.emplace(jn, pos_now);
//   }

//   double max_step_rad = 0.05;
//   double max_delta = 0.0;
//   for (std::size_t i = 0; i < dof; ++i) {
//     const auto &jn = group_joint_names[i];
//     auto it = start_joint_position.find(jn);
//     if (it == start_joint_position.end()) {
//       RCLCPP_ERROR(logger_, "[solve_core] Joint lookup failed");
//       publish_error(make_error(SolveErrc::JointLookupFailed, "Joint lookup failed",
//                                {{"group_name", group_name}}));
//       return std::nullopt;
//     }
//     max_delta = std::max(max_delta, std::fabs(req.target_joints[i] - it->second));
//   }
//   const int N = std::max(1, static_cast<int>(std::ceil(max_delta / std::max(1e-6, max_step_rad))));

//   Trajectory traj;
//   traj.joint_names = group_joint_names;
//   traj.points.reserve(static_cast<std::size_t>(N + 1));

//   moveit::core::RobotState rs = start_state;

//   for (int k = 0; k <= N; ++k) {
//     const double t = static_cast<double>(k) / static_cast<double>(N);
//     std::vector<double> q(dof);
//     for (std::size_t i = 0; i < dof; ++i) {
//       const auto &jn = group_joint_names[i];
//       auto it = start_joint_position.find(jn);
//       if (it == start_joint_position.end()) {
//         RCLCPP_ERROR(logger_, "[solve_core] Joint lookup failed");
//         publish_error(make_error(SolveErrc::JointLookupFailed, "Joint lookup failed",
//                                  {{"group_name", group_name}}));
//         return std::nullopt;
//       }
//       q[i] = it->second + (req.target_joints[i] - it->second) * t;
//     }

//     rs.setJointGroupPositions(jmg, q);
//     rs.update();

//     std::string collision_err;
//     if (!adapter_->check_self_collision(rs, group_name, collision_err)) {
//       if (collision_err.empty())
//         collision_err = "Self collision detected";
//       RCLCPP_ERROR(logger_, "[solve_core] %s", collision_err.c_str());
//       publish_error(make_error(SolveErrc::CollisionDetected, collision_err,
//                                {{"group_name", group_name}}));
//       return std::nullopt;
//     }

//     TrajectoryPoint p;
//     p.positions = q;
//     p.time_from_start = 0.05 * k;
//     traj.points.push_back(std::move(p));
//   }

//   SolveResponse resp;
//   resp.trajectory = std::move(traj);
//   return resp;
// }

// }  // namespace solve_core

bool ArmSolveServer::planTrajectoryByJoints(const std::shared_ptr<GoalContext>& ctx,
                                            moveit_msgs::msg::RobotTrajectory& out_traj_msg) {
  // 取 ctx->target_joints
  // 检查机器人模型是否完整
  const moveit::core::RobotModelConstPtr robot_model = move_group_->getRobotModel(); // 获取模型
  if (!robot_model) {
    RCLCPP_ERROR(get_logger(), "RobotModel 为空");
    return false;
  }
  // 检查规划组与urdf是否匹配
  const std::string group_name             = move_group_->getName();                      // 获取当前规划组的名称
  const moveit::core::JointModelGroup* jmg = robot_model->getJointModelGroup(group_name); // 根据名称获取当前的规划组
  if (!jmg) {
    RCLCPP_ERROR(get_logger(), "找不到 JointModelGroup '%s'", group_name.c_str());
    return false;
  }

  bool planning_ready_ = true;
  if (!planning_ready_ || !psm_ || !robot_model || !jmg) {
    RCLCPP_ERROR(get_logger(), "psm的lateInit失败");
    return false;
  }
  // 获取关节状态快照
  engineer_interfaces::msg::Joints current_joints_copy;
  {
    std::lock_guard<std::mutex> lock(current_joints_mutex_);
    current_joints_copy = current_joints_;
  }

  // 按关节名称映射获取关节角
  const auto& group_joint_names = jmg->getVariableNames();
  const size_t dof              = group_joint_names.size();
  moveit::core::RobotState start_joint_state(robot_model);
  start_joint_state.setToDefaultValues(); // 先用默认

  // 1. 先把 current_joints_copy 转成 map（无序）
  std::unordered_map<std::string, double> position_map;
  position_map.reserve(current_joints_copy.joints.size());

  for (const auto& j : current_joints_copy.joints) {
    // 结构化绑定
    auto [it, inserted] = position_map.emplace(j.name, j.position);
    if (!inserted) {
      RCLCPP_WARN(get_logger(), "关节[%s]在消息中重复，使用最后一次的值覆盖", j.name.c_str());
      it->second = j.position;
    }
  }

  // 2. 再按 group_joint_names 提取并写RobotState（有序）
  std::unordered_map<std::string, double> start_joint_position;
  start_joint_position.reserve(dof);

  for (const auto& jn : group_joint_names) {
    auto it = position_map.find(jn);
    if (it == position_map.end()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 5000 /*ms*/, "当前状态缺少关节[%s]", jn.c_str());
      return false;
    }
    start_joint_position.emplace(it->first, it->second);
    // 因为前面已经做过对关节数据重复性的检测，而且关节个数是固定且相同的，所以这里不再进行防御性编程
    // 将关节角按名称写入state
    start_joint_state.setVariablePosition(it->first, it->second);
  }

  start_joint_state.update();

  // 检查起始状态是否在关节限制内
  bool start_ok = start_joint_state.satisfiesBounds(jmg);
  if (!start_ok) {
    RCLCPP_ERROR(get_logger(), "[DEBUG] joint_state 已经越界");
    for (const auto& jn : group_joint_names) {
      const moveit::core::JointModel* jm = robot_model->getJointModel(jn);
      if (!jm)
        continue;

      const double* pos_ptr = start_joint_state.getJointPositions(jn);
      const double pos_now  = pos_ptr ? *pos_ptr : 0.0;

      // 取这个关节变量的上下限
      const auto& b = jm->getVariableBounds(jn);

      RCLCPP_ERROR(get_logger(),
                   "[DEBUG] 关节[%s] = %.3f rad (limit [%.3f, %.3f])",
                   jn.c_str(),
                   pos_now,
                   b.min_position_,
                   b.max_position_);
    }
    // 强行把越界的值夹回范围内，避免规划器直接拒绝
    start_joint_state.enforceBounds(jmg);
    RCLCPP_WARN(get_logger(),
                "将起始姿态夹回关节限制范围。\n"
                "需要注意如果真实机械臂当前姿态本来就越界，规划出"
                "的第一步会是一个跳变！");
  } else {
    RCLCPP_INFO(get_logger(), "[DEBUG] joint_state 在关节限制范围内");
  }

  // 进行关节空间线性插值
  double max_step_rad = 0.05; // 每关节最大步进弧度
  double max_delta    = 0.0;  // 关节目标状态与当前状态关节角差值的最大值

  out_traj_msg = moveit_msgs::msg::RobotTrajectory(); // 清空

  int i = 0;
  for (auto joint_name : group_joint_names) {
    auto it = start_joint_position.find(joint_name);
    if (it == start_joint_position.end()) {
      // 没找到
    } else {
      // 所有关节中目标关节角与当前关节角的差值的最大值
      max_delta = std::max(max_delta, std::fabs(ctx->target_joints[i] - it->second));
      ++i;
    }
  }
  // 总变化量除以单步最大变化量，向上取整，至少 1 步
  const int N = std::max(1, (int)std::ceil(max_delta / std::max(1e-6, max_step_rad)));

  // 4) 准备碰撞检测对象（只做 self-collision）
  collision_detection::CollisionRequest req;
  collision_detection::CollisionResult res;
  req.group_name   = move_group_->getName();
  req.contacts     = false;
  req.max_contacts = 0;

  RCLCPP_INFO(get_logger(), "即将阻塞线程以获取scene");
  planning_scene_monitor::LockedPlanningSceneRO scene(psm_);
  RCLCPP_INFO(get_logger(), "获得scene后取消阻塞");

  if (!scene) {
    RCLCPP_ERROR(get_logger(), "[ERROR]自碰撞检测前获得的场景信息为空");
    return false;
  }

  // 6) 逐点插值 + 自碰撞检查
  trajectory_msgs::msg::JointTrajectory jt;
  jt.joint_names = group_joint_names;
  jt.points.reserve(N + 1);

  moveit::core::RobotState rs = start_joint_state; // 拷贝一份 state 用于设置关节角

  for (int k = 0; k <= N; ++k) {
    const double t = (double)k / (double)N;

    // std::vector<double> q; // 一个路点的六个关节角
    // for (auto joint_name : group_joint_names) {
    //   auto it         = start_joint_position.find(joint_name);
    //   double q_interp = it->second + (ctx->target_joints[i] - it->second) * t;
    //   q.push_back(q_interp);
    // }

    std::vector<double> q(dof);
    for (size_t i = 0; i < dof; ++i) {
      const auto& jn = group_joint_names[i];
      auto it        = start_joint_position.find(jn);
      if (it == start_joint_position.end())
        return false;
      q[i] = it->second + (ctx->target_joints[i] - it->second) * t;
    }

    if (q.size() != dof) {
      RCLCPP_ERROR(get_logger(), "q中传入的关节个数有误，PlanByJoints failed");
      return false;
    }
    // 设置 robot state（并更新）
    rs.setJointGroupPositions(jmg, q);
    rs.update();

    res.clear();
    // 只查自碰撞：checkSelfCollision
    scene->checkSelfCollision(req, res, rs);
    if (res.collision) {
      RCLCPP_ERROR(get_logger(), "机械臂自身发生碰撞，planByJoints failed");
      return false; // 一旦碰撞，直接失败
    }

    trajectory_msgs::msg::JointTrajectoryPoint p;
    rs.copyJointGroupPositions(jmg, p.positions);
    // time_from_start 不强制,但为了上层更好处理，给一个均匀时间

    const double dt           = 0.05;
    const double time         = k * dt;
    p.time_from_start.sec     = (int)time;
    p.time_from_start.nanosec = (uint32_t)((time - (int)time) * 1e9);

    jt.points.push_back(std::move(p));
    p.positions.clear();
  }

  // 7) 写回 RobotTrajectory
  out_traj_msg.joint_trajectory = std::move(jt);
  return true;
}
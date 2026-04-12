#include <Eigen/Geometry>
#include <algorithm>
#include <vector>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include "collision/self_collision_detector.hpp"
#include "executor/detail.hpp"
#include "executor/executor.hpp"
#include "log_tools/log.hpp"

namespace solve_executor {
bool SolveExecutor::plan_joints(
    const SolveRequest &req, solve_executor::Trajectory &out_traj,
    std::string &err,
    const collision::SelfCollisionDetector *self_collision_detector) {

  // 1. 基础合法性检查
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
    return false;
  }

  // 2. 计算最大关节位移
  double max_delta = 0.0;
  for (std::size_t i = 0; i < config_.joint_names.size(); ++i) {
    max_delta = std::max(max_delta, std::fabs(req.target_joints[i] -
                                              req.current_joints.positions[i]));
  }

  double v_limit = config_.nominal_joint_speed * config_.max_velocity_scaling;
double total_duration = (max_delta * 1.875) / std::max(1e-6, v_limit);

  // 3. 计算插值点数 N
  // 引入 1.875 补偿系数：确保速度最快处的步长也不超过 max_step_rad
  double max_step_rad = std::max(1e-6, config_.joints_max_step_rad);
  const int N = std::max(
      1, static_cast<int>(std::ceil((max_delta * 1.875) / max_step_rad)));

  Trajectory traj;
  traj.joint_names = config_.joint_names;
  traj.points.reserve(static_cast<std::size_t>(N + 1));

  moveit::core::RobotState insert_state = start_state;

  // 4. 生成位置轨迹 (归一化时间插值)
  for (int k = 0; k <= N; ++k) {
    const double t = static_cast<double>(k) / static_cast<double>(N);
    const double t2 = t * t;
    const double t3 = t2 * t;
    // 五次多项式位置插值系数: 10t^3 - 15t^4 + 6t^5
    const double blend = t3 * (10.0 - 15.0 * t + 6.0 * t2);

    std::vector<double> q(config_.joint_count);
    for (std::size_t i = 0; i < config_.joint_names.size(); ++i) {
      q[i] = req.current_joints.positions[i] +
             (req.target_joints[i] - req.current_joints.positions[i]) * blend;
    }

    // 碰撞检测
    collision::SelfCollisionCheckResult collision_result;
    if (self_collision_detector &&
        !self_collision_detector->check(q, collision_result)) {
      LOGE("[solve_executor] Collision detected at step k={}/{}", k, N);
      err = "Collision at interpolation point";
      return false;
    }

    TrajectoryPoint p;
    p.positions = q;
    traj.points.push_back(std::move(p));
  }

  // 5. 时间参数化 (根据速度/加速度限幅计算每点的时间戳)
  parameterize_time_from_start(traj, config_.max_velocity_scaling);

  // 6. 计算速度和加速度 (基于时间戳进行物理量映射)
  const double total_duration =
      traj.points.empty() ? 0.0 : traj.points.back().time_from_start;

  for (int k = 0; k <= N; ++k) {
    auto &point = traj.points[static_cast<std::size_t>(k)];
    point.velocities.resize(config_.joint_names.size());
    point.accelerations.resize(config_.joint_names.size());

    if (total_duration <= 1e-9) {
      std::fill(point.velocities.begin(), point.velocities.end(), 0.0);
      std::fill(point.accelerations.begin(), point.accelerations.end(), 0.0);
      continue;
    }

    const double u = static_cast<double>(k) / static_cast<double>(N);
    const double u2 = u * u;
    const double u3 = u2 * u;

    // 速度导数: s'(u) = 30u^2 - 60u^3 + 30u^4
    const double v_blend = 30.0 * u2 - 60.0 * u3 + 30.0 * u2 * u2;
    // 加速度导数: s''(u) = 60u - 180u^2 + 120u^3
    const double a_blend = 60.0 * u - 180.0 * u2 + 120.0 * u3;

    for (std::size_t i = 0; i < config_.joint_names.size(); ++i) {
      double delta_q = req.target_joints[i] - req.current_joints.positions[i];

      // 物理速度 = delta_q * s'(u) / T
      point.velocities[i] = delta_q * v_blend / total_duration;
      // 物理加速度 = delta_q * s''(u) / T^2
      point.accelerations[i] =
          delta_q * a_blend / (total_duration * total_duration);
    }
  }

  out_traj = std::move(traj);
  return true;
}
} // namespace solve_executor
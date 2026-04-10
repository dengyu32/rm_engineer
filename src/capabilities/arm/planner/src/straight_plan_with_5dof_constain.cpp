#include "planner/straight_planner.hpp"

#include "calculator/cost_func.hpp"
#include "calculator/hybrid_ik.hpp"
#include "log_tools/log.hpp"

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_set>

namespace planner {
using types::Trajectory;

namespace {
// 辅助函数：将一个 double vector
// 中的元素扩大大倍数后得到离散值，将原vector组装为一个字符串
// key（用于关节空间解去重，基于四舍五入）
static std::string vec_key_rounded(const std::vector<double> &v,
                                   double eps = 1e-4) {
  std::string k;
  k.reserve(v.size() * 8);
  for (double x : v) {
    long r = static_cast<long>(std::round(x / eps));
    k += std::to_string(r);
    k.push_back(',');
  }
  return k;
}

// 辅助函数：计算两个关节向量的 L2 距离（用于 DP 连续性评分）
static double distance_l2(const std::vector<double> &q,
                          const std::vector<double> &q_ref) {
  const size_t n = std::min(q.size(), q_ref.size());
  if (n == 0) {
    return std::numeric_limits<double>::infinity();
  }
  double sum = 0.0;
  for (size_t i = 0; i < n; ++i) {
    const double d = q[i] - q_ref[i];
    sum += d * d;
  }
  return std::sqrt(sum);
}

// 辅助函数：计算两个关节向量中单关节的最大角度变化量（用于连续性评分打破并列）
static double max_joint_delta(const std::vector<double> &q,
                              const std::vector<double> &q_ref) {
  const size_t n = std::min(q.size(), q_ref.size());
  double max_delta = 0.0;
  for (size_t i = 0; i < n; ++i) {
    max_delta = std::max(max_delta, std::abs(q[i] - q_ref[i]));
  }
  return max_delta;
}

// 辅助函数：根据最大 pitch 变化幅度和采样数生成 pitch 偏移列表
static std::vector<double> build_axis_rotation_offsets(double max_rotation_rad,
                                                       int rotation_samples) {
  constexpr double kEps = 1e-9;
  const double rotation_limit = std::abs(max_rotation_rad);
  const int sample_count = std::max(1, rotation_samples);
  if (rotation_limit <= kEps || sample_count == 1) {
    return {0.0}; // rotation无偏移
  }

  std::vector<double> offsets;
  offsets.reserve(static_cast<std::size_t>(sample_count));
  const double step =
      (2.0 * rotation_limit) / static_cast<double>(sample_count - 1);
  for (int idx = 0; idx < sample_count; ++idx) {
    offsets.push_back(-rotation_limit +
                      static_cast<double>(idx) *
                          step); // 从 -limit 到 +limit 均匀分布
  }
  return offsets;
}

} // namespace

std::optional<Trajectory> StraightPlanner::plan_with_5dof_constrain(
    moveit::core::RobotState &start_state, Eigen::Vector3d &target_vector,
    double target_length, const calculator::CostOptions &cost_opt) {
  LOGI("Start Plan Cartesian With 5dof Constrain!");

  const auto *jmg = robot_model_->getJointModelGroup(group_name_);
  const auto *ee_link = robot_model_->getLinkModel(ee_link_);
  const auto *ref_link = robot_model_->getLinkModel(settings_.reference_link);
  if (!jmg || !ee_link || !ref_link) {
    LOGE("[solve_executor][straight_planner_5dof] Invalid model handles: "
         "jmg={}, ee_link={}, ref_link={}",
         jmg != nullptr, ee_link != nullptr, ref_link != nullptr);
    return std::nullopt;
  }

  calculator::HybridIK ik(robot_model_, group_name_, ee_link_);
  calculator::IKOptions ik_opt;

  // 获取所有y轴的旋转偏移
  const Eigen::Isometry3d T0 = start_state.getGlobalLinkTransform(ee_link);
  const Eigen::Matrix3d base_rotation = T0.linear();
  const std::vector<double> rotation_offsets = build_axis_rotation_offsets(
      settings_.max_rotation_rad, settings_.rotation_samples);

  // 计算每个路点位置和路点个数
  const Eigen::Vector3d line_delta = target_vector * target_length;
  const int num_waypoints = std::max(
      1, static_cast<int>(std::ceil(target_length / settings_.sample_step_m)));

  const int cap_sols_per_waypoint = 6;
  std::vector<std::vector<std::vector<double>>>
      sols_all_waypoint; // 三维数组：所有路点的所有关节空间解列表
  sols_all_waypoint.reserve(num_waypoints);

  // DP的前一层解列表，初始为起始状态的单一解（保证至少有一个解可以扩展）
  std::vector<std::vector<double>> prev_solutions;
  {
    std::vector<double> q0;
    start_state.copyJointGroupPositions(jmg, q0);
    prev_solutions.push_back(q0);
  }

  for (int i = 1; i <= num_waypoints; ++i) { // loop0：遍历每个路点
    LOGE("[solve_executor][straight_planner_5dof] Getting waypoint{} from "
         "different rotation",
         i);

    // 设置当前路点的目标位置（沿直线插值）
    const double rate =
        static_cast<double>(i) / static_cast<double>(num_waypoints);
    const Eigen::Vector3d target_position =
        T0.translation() + line_delta * rate;

    std::vector<std::vector<double>> sols_per_waypoint;
    std::unordered_set<std::string>
        seen_keys; // 用于去重的哈希集合，在最内层循环中使用

    // 第一次遍历：得到当前点的所有自检测有效的关节空间解
    for (const auto &prev_q : prev_solutions) // loop1.0：遍历前一个点的所有解
    {
      LOGI("[solve_executor][straight_planner_5dof] Solving IK for waypoint {}",i);
      moveit::core::RobotState seed_state(start_state);
      seed_state.setJointGroupPositions(jmg, prev_q);
      seed_state.enforceBounds(jmg);
      seed_state.update();

      for (double rotation_offset :
           rotation_offsets) // loop1.1：遍历每个路点的所有旋转偏移
      {
        Eigen::Isometry3d Ti = Eigen::Isometry3d::Identity();
        Ti.translation() = target_position;
        const Eigen::Matrix3d delta_rotation =
            Eigen::AngleAxisd(rotation_offset, Eigen::Vector3d::UnitY())
                .toRotationMatrix();
        Ti.linear() =
            base_rotation *
            delta_rotation; // 绕link6自身y轴旋转，注意左乘与右乘的区别

        std::vector<std::vector<double>> sols;
        if (!ik.solveAll(seed_state, Ti, ik_opt, sols)) {
          LOGW("[solve_executor][ik] Failed to solve IK for waypoint {} with "
               "rotation offset {}",
               i, rotation_offset);
          continue;
        }

        for (
            auto &per_sol :
            sols) { // loop1.2：遍历每个路点的每个旋转偏移的所有关节空间解，进行自碰撞过滤和去重
          if (self_collision_detector_) {
            collision::SelfCollisionCheckResult collision_result;
            if (!self_collision_detector_->check(per_sol, collision_result)) {
              LOGE("[solve_executor][straight_planner_5dof] Filter IK solution "
                   "by self collision: {}",
                   collision_result.message);
              continue;
            }
          }

          std::string key = vec_key_rounded(per_sol, 1e-4);
          if (seen_keys.insert(key).second) {
            sols_per_waypoint.push_back(std::move(per_sol));
          } else {
            LOGW(
                "[solve_executor][straight_planner_5dof] Duplicate IK solution "
                "filtered by key: {}",
                key); // 重复的关节空间解已经被过滤
          }
        }
      }
    }

    // 当前点没有任何关节空间解
    if (sols_per_waypoint.empty()) {
      LOGE("[solve_executor][straight_planner_5dof] No IK solutions at "
           "waypoint {} within rotation limit {} rad",
           i, settings_.max_rotation_rad);
      return std::nullopt;
    }
    else {
      LOGI("[solve_executor][straight_planner_5dof] Found {} available IK solutions at waypoint {}",
           sols_per_waypoint.size(), i);
    }

    // 第二次遍历：对当前路点的解进行连续性评分和排序，优先保留与前一层解连续性更好的解（距离更近，且最大关节跳变更小）
    struct ScoredSolution {
      double continuity;
      double max_delta;
      std::vector<double> q;
    };

    std::vector<ScoredSolution> scored;
    scored.reserve(sols_per_waypoint.size());
    constexpr double kScoreEps = 1e-12;
    for (auto &q :
         sols_per_waypoint) { // loop2.0：遍历当前点的所有解，计算连续性评分
      double best = std::numeric_limits<double>::infinity();
      double best_max_delta = std::numeric_limits<double>::infinity();
      for (
          const auto &prev_q :
          prev_solutions) { // loop2.1：遍历前一点的所有解，找到与当前解连续性最好（距离最近）的一个，并记录其距离和最大关节跳变
        const double dist = distance_l2(q, prev_q);
        const double delta = max_joint_delta(q, prev_q);
        if (dist + kScoreEps < best) {
          best = dist;
          best_max_delta = delta;
        } else if (std::abs(dist - best) <= kScoreEps &&
                   delta < best_max_delta) {
          best_max_delta = delta;
        }
      }
      scored.push_back({best, best_max_delta, std::move(q)});
    }

    // 第三次遍历：对当前点的解根据连续性评分进行排序，优先保留连续性更好的解（距离更近，且最大关节跳变更小）
    std::stable_sort(
        scored.begin(), scored.end(), [](const auto &a, const auto &b) {
          constexpr double kLocalScoreEps = 1e-12;
          if (std::abs(a.continuity - b.continuity) > kLocalScoreEps) {
            return a.continuity < b.continuity;
          }
          return a.max_delta < b.max_delta;
        });

    sols_per_waypoint.clear();
    const size_t keep =
        std::min(scored.size(),
                 static_cast<size_t>(
                     cap_sols_per_waypoint)); // 只保留连续性评分最好的前几个解
    sols_per_waypoint.reserve(keep);
    for (size_t idx = 0; idx < keep; ++idx) {
      sols_per_waypoint.push_back(std::move(scored[idx].q));
    }

    sols_all_waypoint.push_back(sols_per_waypoint);
    prev_solutions = sols_per_waypoint;
  }
  // 到这里，sols_all_waypoint
  // 中已经包含了每个路点的所有关节空间解列表，并且每个列表中的解都经过了自碰撞过滤和局部连续性优先排序，只保留了前几个解

  // 下面计算连续性和避奇异点全局代价
  const size_t N = sols_all_waypoint.size();
  std::vector<std::vector<double>> dp_costs(N); // 每个路点每个解的代价
  std::vector<std::vector<int>> prev_idx(
      N); // DP回溯索引，记录每个路点每个解在前一层中最优连接的解的索引

  calculator::CostFunc cost_func(start_state, jmg, ee_link, cost_opt);

  // assign
  // 把容器中的每个元素都设置为指定值，这里初始化第一层的代价为无穷大，回溯索引为-1（表示没有前驱）
  dp_costs[0].assign(sols_all_waypoint[0].size(),
                     std::numeric_limits<double>::infinity());
  prev_idx[0].assign(sols_all_waypoint[0].size(), -1);

  std::vector<double> q0;
  start_state.copyJointGroupPositions(jmg, q0);

  for (size_t j = 0; j < sols_all_waypoint[0].size();
       ++j) { // loop3.0：初始化第一层的DP代价，计算起始状态到第一层每个解的代价
    dp_costs[0][j] = cost_func.compute(q0, sols_all_waypoint[0][j]);
  }

  for (
      size_t i = 1; i < N;
      ++i) { // loop3.1：DP主循环，遍历每个路点，从第二层开始，计算每个解的最优前驱和代价
    const auto &prev_layer = sols_all_waypoint[i - 1];
    const auto &cur_layer = sols_all_waypoint[i];

    dp_costs[i].assign(cur_layer.size(),
                       std::numeric_limits<double>::infinity());
    prev_idx[i].assign(cur_layer.size(), -1);
    bool all_edges_rejected_by_jump_limit = true;

    for (size_t j = 0; j < cur_layer.size(); ++j) {
      const auto &qj = cur_layer[j];
      for (size_t k = 0; k < prev_layer.size(); ++k) {
        if (!std::isfinite(dp_costs[i - 1][k])) {
          continue;
        }
        const auto &qk = prev_layer[k];
        const double delta = max_joint_delta(qk, qj);
        if (!std::isfinite(delta) || delta > settings_.max_joint_jump_rad) {
          continue;
        }
        all_edges_rejected_by_jump_limit = false;

        const double total = dp_costs[i - 1][k] + cost_func.compute(qk, qj);
        if (total < dp_costs[i][j]) {
          dp_costs[i][j] = total;
          prev_idx[i][j] = static_cast<int>(k);
        }
      }
    }

    bool any_ok = false;
    for (double v : dp_costs[i]) {
      if (std::isfinite(v)) {
        any_ok = true;
        break;
      }
    }
    if (!any_ok) {
      if (all_edges_rejected_by_jump_limit) {
        LOGE("[solve_executor][straight_planner_5dof] All transitions rejected "
             "by max_joint_jump_rad at layer {}: "
             "threshold={}, prev_candidates={}, cur_candidates={}",
             i, settings_.max_joint_jump_rad, prev_layer.size(),
             cur_layer.size());
      }
      LOGE(
          "[solve_executor][straight_planner_5dof] DP disconnected at layer {}",
          i);
      return std::nullopt;
    }
  }

  std::vector<std::vector<double>> best_path_rev;
  best_path_rev.reserve(N);

  size_t last_idx = 0;
  double best_last_cost = std::numeric_limits<double>::infinity();
  for (size_t j = 0; j < dp_costs[N - 1].size(); ++j) {
    if (dp_costs[N - 1][j] < best_last_cost) {
      best_last_cost = dp_costs[N - 1][j];
      last_idx = j;
    }
  }

  int cur_idx = static_cast<int>(last_idx);
  for (int i = static_cast<int>(N) - 1; i >= 0; --i) {
    best_path_rev.push_back(sols_all_waypoint[i][cur_idx]);
    cur_idx = prev_idx[i][cur_idx];
    if (i > 0 && cur_idx < 0) {
      LOGE("[solve_executor][straight_planner_5dof] DP traceback failed at "
           "layer {}",
           i);
      return std::nullopt;
    }
  }

  std::vector<std::vector<double>> q_path;
  q_path.reserve(N);
  for (auto it = best_path_rev.rbegin(); it != best_path_rev.rend(); ++it) {
    q_path.push_back(*it);
  }

  std::vector<double> q_start;
  start_state.copyJointGroupPositions(jmg, q_start);

  std::vector<std::vector<double>> q_path_interp;
  q_path_interp.reserve(q_path.size() + 8);
  q_path_interp.push_back(q_start);
  if (!q_path.empty()) {
    const auto &q_first = q_path.front();
    const double max_delta_to_first = max_joint_delta(q_start, q_first);
    if (std::isfinite(max_delta_to_first) &&
        settings_.max_joint_jump_rad > 1e-9) {
      const int start_segments =
          std::max(1, static_cast<int>(std::ceil(
                          max_delta_to_first / settings_.max_joint_jump_rad)));
      for (int seg = 1; seg < start_segments; ++seg) {
        const double t =
            static_cast<double>(seg) / static_cast<double>(start_segments);
        std::vector<double> q_mid(q_start.size(), 0.0);
        for (std::size_t idx = 0; idx < q_mid.size(); ++idx) {
          q_mid[idx] = q_start[idx] + (q_first[idx] - q_start[idx]) * t;
        }
        q_path_interp.push_back(std::move(q_mid));
      }
    }
  }
  for (auto &q : q_path) {
    q_path_interp.push_back(std::move(q));
  }

  Trajectory traj;
  traj.joint_names = jmg->getVariableNames();
  traj.points.reserve(q_path_interp.size());
  for (const auto &q : q_path_interp) {
    types::TrajectoryPoint pt;
    pt.positions = q;
    traj.points.push_back(std::move(pt));
  }

  return traj;
}

} // namespace planner


// #include <Eigen/Geometry>
// #include <algorithm>
// #include <chrono>
// #include <map>
// #include <memory>
// #include <sstream>
// #include <thread>
// #include <vector>

// #include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

// #include "calculator/hybrid_ik.hpp"
// #include "calculator/cost_func.hpp"
// #include "collision/self_collision_detector.hpp"
// #include "executor/detail.hpp"
// #include "executor/executor.hpp"
// #include "executor/test.hpp"
// #include "log_tools/log.hpp"
// #include "planner/straight_planner.hpp"

// namespace solve_executor {
// namespace {
// const moveit::core::JointModelGroup *
// get_jmg(moveit::core::RobotModelConstPtr robot_model,
//         const std::string &group_name) {
//   const auto jmg = robot_model->getJointModelGroup(group_name);
//   if (!jmg) {
//     return nullptr;
//   }
//   return jmg;
// }

// bool is_finite_pose(const Pose &pose) {
//   return std::isfinite(pose.x) && std::isfinite(pose.y) &&
//          std::isfinite(pose.z) && std::isfinite(pose.qx) &&
//          std::isfinite(pose.qy) && std::isfinite(pose.qz) &&
//          std::isfinite(pose.qw);
// }

// } // namespace

// SolveExecutor::SolveExecutor(rclcpp::Node &node)
//     : node_(node), logger_(node.get_logger()), clock_(node.get_clock()),
//       config_(SolveExecutorConfig::Load(node)),
//       last_plan_time_(clock_->now() -
//                       rclcpp::Duration(std::chrono::milliseconds(
//                           config_.plan_min_interval_ms))),
//       node_handle_(
//           std::shared_ptr<rclcpp::Node>(&node_, [](rclcpp::Node *) {})) {
//   RCLCPP_INFO(logger_, "\n%s", this->config_.summary().c_str());
// }

// bool SolveExecutor::isReady() const {
//   std::scoped_lock<std::mutex> lock(init_mutex_);
//   return move_group_ && psm_ && robot_model_ && current_state_;
// }

// void SolveExecutor::stop() {
//   std::scoped_lock<std::mutex> lock(init_mutex_);
//   if (move_group_) {
//     move_group_->stop();
//   }
// }

// bool SolveExecutor::ensureInitialized(std::string &err) {
//   std::scoped_lock<std::mutex> lock(init_mutex_);
//   if (move_group_ && psm_ && robot_model_ && current_state_) {
//     return true;
//   }

//   if (config_.late_init_delay_ms > 0) {
//     std::this_thread::sleep_for(
//         std::chrono::milliseconds(config_.late_init_delay_ms));
//   }

//   if (!move_group_) {
//     move_group_ =
//         std::make_unique<moveit::planning_interface::MoveGroupInterface>(
//             node_handle_, config_.group_name);
//   }

//   if (!psm_) {
//     psm_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
//         node_handle_, "robot_description");
//     psm_->startSceneMonitor();
//     psm_->startWorldGeometryMonitor();
//     psm_->startStateMonitor();
//   }

//   robot_model_ = move_group_->getRobotModel();
//   if (!robot_model_) {
//     err = "Robot model is unavailable";
//     LOGE("[solve_executor] {}", err);
//     return false;
//   }

//   if (!current_state_) {
//     current_state_ = std::make_unique<moveit::core::RobotState>(robot_model_);
//     current_state_->setToDefaultValues();
//     current_state_->update();
//   }

//   return true;
// }

// bool SolveExecutor::execute(const SolveRequest &req,
//                             solve_executor::Trajectory &out_traj,
//                             std::string &err) {
//   std::scoped_lock<std::mutex> lock(exec_mutex_);

//   if (!ensureInitialized(err)) {
//     return false;
//   }

//   const auto now = clock_->now();
//   const auto min_interval =
//       rclcpp::Duration(std::chrono::milliseconds(config_.plan_min_interval_ms));
//   if (config_.plan_min_interval_ms > 0 &&
//       now - last_plan_time_ < min_interval) {
//     err = "Planning request throttled by plan_min_interval_ms";
//     LOGW("[solve_executor] {}", err);
//     return false;
//   }

//   if (!plan(req, out_traj, err)) {
//     return false;
//   }

//   if (out_traj.points.empty()) {
//     LOGW("[solve_executor] Planning succeeded but trajectory is empty");
//   } else {
//     const auto &last_point = out_traj.points.back();
//     std::ostringstream oss;
//     oss << "[solve_executor] Last trajectory point joint positions:";
//     for (std::size_t i = 0; i < last_point.positions.size(); ++i) {
//       oss << ' ';
//       if (i < out_traj.joint_names.size()) {
//         oss << out_traj.joint_names[i] << '=';
//       } else {
//         oss << "joint_" << i << '=';
//       }
//       oss << last_point.positions[i];
//     }
//     LOGT("{}", oss.str());
//   }

//   last_plan_time_ = now;
//   return true;
// }

// bool SolveExecutor::plan(const SolveRequest &req,
//                          solve_executor::Trajectory &out_traj,
//                          std::string &err) {
//   const auto planning_scene = psm_ ? psm_->getPlanningScene() : nullptr;
//   if (!planning_scene) {
//     err = "Planning scene is unavailable";
//     LOGE("[solve_executor] {}", err);
//     return false;
//   }

//   collision::SelfCollisionRequest collision_req;
//   collision_req.max_contacts = config_.max_contacts;
//   collision_req.max_contacts_per_pair = config_.max_contacts_per_pair;

//   collision::SelfCollisionDetector self_collision_detector(
//       config_.group_name, collision_req, robot_model_, planning_scene);

//   switch (req.option) {
//   case PlanOption::NORMAL:
//     return plan_normal(req, out_traj, err, &self_collision_detector);
//   case PlanOption::CARTESIAN:
//     return plan_cartesian(req, out_traj, err, &self_collision_detector);
//   case PlanOption::JOINTS:
//     return plan_joints(req, out_traj, err, &self_collision_detector);
//   default:
//     err = "Unknown planning option";
//     LOGE("[solve_executor] {}", err);
//     return false;
//   }
// }

// bool SolveExecutor::plan_normal(
//     const SolveRequest &req, solve_executor::Trajectory &out_traj,
//     std::string &err,
//     const collision::SelfCollisionDetector *self_collision_detector) {
//   if (!is_finite_pose(req.target_pose)) {
//     err = "target_pose contains non-finite values";
//     LOGE("[solve_executor] {}", err);
//     return false;
//   }

//   const auto group_name = config_.group_name;
//   const auto ee_link_name = config_.ee_link_name;
//   const auto jmg = get_jmg(robot_model_, group_name);
//   if (!jmg) {
//     err = "JointModelGroup not found";
//     LOGE("[solve_executor] {}", err);
//     return false;
//   }

//   moveit::core::RobotState start_state(robot_model_);
//   if (!fill_joint_state_require_all(req.current_joints, jmg, start_state, err)) {
//     return false;
//   }
//   start_state.update();

//   calculator::HybridIK hybrid_ik(robot_model_, group_name, ee_link_name);
//   calculator::IKOptions ik_options;
//   std::vector<std::vector<double>> ik_solutions;
  
//   if (!hybrid_ik.solveAll(start_state, poseToIsometry(req.target_pose),
//                           ik_options, ik_solutions)) {
//     err = "HybridIK failed for target_pose";
//     LOGE("[solve_executor] {}", err);
//     return false;
//   }

//   // 因为 HybridIK 已经按照连续性排序，front() 拿到的就是最连续的解
//   std::vector<double> target_pose = std::move(ik_solutions.front());
//   if (target_pose.size() != jmg->getVariableCount()) {
//     err = "IK joint result size mismatch";
//     LOGE("[solve_executor] {}", err);
//     return false;
//   }

//   SolveRequest joint_req = req;
//   joint_req.target_joints = std::move(target_pose);
//   return plan_joints(joint_req, out_traj, err, self_collision_detector);
// }

// bool SolveExecutor::plan_joints(
//     const SolveRequest &req, solve_executor::Trajectory &out_traj,
//     std::string &err,
//     const collision::SelfCollisionDetector *self_collision_detector) {
//   (void)self_collision_detector;
//   for (double value : req.target_joints) {
//     if (!std::isfinite(value)) {
//       err = "target_joints contains non-finite value";
//       LOGE("[solve_executor] {}", err);
//       return false;
//     }
//   }

//   const auto group_name = config_.group_name;
//   const auto jmg = get_jmg(robot_model_, group_name);
//   moveit::core::RobotState start_state(robot_model_);
//   if (!fill_joint_state_require_all(req.current_joints, jmg, start_state, err)) {
//     err = "target_joints contains non-finite value";
//     LOGE("[solve_executor] {}", err);
//     return false;
//   }

//   double max_step_rad = config_.joints_max_step_rad;
//   double max_delta = 0.0;
//   moveit::core::RobotState insert_state = start_state;
//   for (std::size_t i = 0; i < config_.joint_names.size(); ++i) {
//     max_delta = std::max(max_delta, std::fabs(req.target_joints[i] -
//                                               req.current_joints.positions[i]));
//   }

//   const int N = std::max(
//       1, static_cast<int>(std::ceil(max_delta / std::max(1e-6, max_step_rad))));

//   Trajectory traj;
//   traj.joint_names = config_.joint_names;
//   traj.points.reserve(static_cast<std::size_t>(N + 1));

//   for (int k = 0; k <= N; ++k) {
//     const double t_raw = static_cast<double>(k) / static_cast<double>(N);
//     // 平滑插值（前缓中匀后缓），降低速度突变
//     const double t = t_raw * t_raw * (3.0 - 2.0 * t_raw);
//     std::vector<double> q(config_.joint_count);
//     for (std::size_t i = 0; i < config_.joint_names.size(); ++i) {
//       q[i] = req.current_joints.positions[i] +
//              (req.target_joints[i] - req.current_joints.positions[i]) * t;
//     }

//     insert_state.setJointGroupPositions(jmg, q);
//     insert_state.update();

//     collision::SelfCollisionCheckResult collision_result;
//     if (!self_collision_detector->check(q, collision_result)) {
//       LOGE("[solve_executor][joint_planner] : Collision at joint space interpolation points k={}", k);
//       return false;
//     }

//     TrajectoryPoint p;
//     p.positions = q;
//     traj.points.push_back(std::move(p));
//   }
//   parameterize_time_from_start(traj, config_.max_velocity_scaling);

//   out_traj = std::move(traj);
//   return true;
// }

// bool SolveExecutor::plan_cartesian(
//     const SolveRequest &req, solve_executor::Trajectory &out_traj,
//     std::string &err,
//     const collision::SelfCollisionDetector *self_collision_detector) {
//   constexpr double kEps = 1e-9;
//   const auto group_name = config_.group_name;
//   const auto jmg = get_jmg(robot_model_, group_name);
//   const auto *ref_link = robot_model_->getLinkModel(config_.reference_link);
//   const auto *ee_link = robot_model_->getLinkModel(config_.ee_link_name);

//   planner::StraightPlannerSettings settings;
//   settings.sample_step_m = config_.sample_step_m;
//   settings.alignment_dot_threshold = config_.alignment_dot_threshold;
//   settings.max_joint_jump_rad = config_.max_joint_jump_rad;
//   settings.reference_link = config_.reference_link;

//   moveit::core::RobotState start_state(robot_model_);
//   if (!fill_joint_state_require_all(req.current_joints, jmg, start_state, err)) {
//     return false;
//   }
//   start_state.update();

//   if (!std::isfinite(req.target_length) || req.target_length <= 0.0) {
//     err = "target_length is invalid";
//     LOGE("[solve_executor] {}", err);
//     return false;
//   }

//   Eigen::Vector3d target_vector = req.target_vector;
//   if (!target_vector.allFinite()) {
//     err = "target_vector contains non-finite values";
//     LOGE("[solve_executor] {}", err);
//     return false;
//   }
  
//   if (config_.use_vision_target_vector) {
//     if (target_vector.norm() <= kEps) {
//       err = "target_vector is a zero vector";
//       LOGE("[solve_executor] {}", err);
//       return false;
//     }
//     const Eigen::Vector3d normalized_target = target_vector.normalized();
//     const Eigen::Isometry3d T_ref = start_state.getGlobalLinkTransform(ref_link);
//     Eigen::Isometry3d T0 = start_state.getGlobalLinkTransform(ee_link);
//     const Eigen::Matrix3d R_ref_ee = T_ref.linear().transpose() * T0.linear();
//     const Eigen::Vector3d ee_x_in_ref = R_ref_ee.col(0).normalized();
//     const double direction_dot = ee_x_in_ref.dot(normalized_target);
//     const double abs_direction_dot = std::abs(direction_dot);
//     if (!std::isfinite(abs_direction_dot) ||
//         abs_direction_dot < settings.alignment_dot_threshold) {
//       LOGE("[solve_executor][straight_planner] Alignment check failed: abs(dot)={}, threshold={}",
//            abs_direction_dot, settings.alignment_dot_threshold);
//       return false;
//     }

//     const Eigen::Vector3d straight_dir =
//         (direction_dot >= 0.0 ? 1.0 : -1.0) * ee_x_in_ref;
//     target_vector = straight_dir;
//   }

//   planner::StraightPlanner planner(robot_model_, group_name,
//                                    config_.ee_link_name, settings,
//                                    self_collision_detector);

//   calculator::CostOptions cost_opt;
//   cost_opt.summary();
//   auto raw_traj = planner.plan(start_state, target_vector, req.target_length, cost_opt);
  
//   if (!raw_traj) {
//     err = err.empty() ? "Straight planner failed" : err;
//     LOGE("[solve_executor] {}", err);
//     return false;
//   }

//   out_traj = *raw_traj;
//   if (!timeParameterizeTrajectory(out_traj, err)) {
//     return false;
//   }
//   return true;
// }

// } // namespace solve_executor
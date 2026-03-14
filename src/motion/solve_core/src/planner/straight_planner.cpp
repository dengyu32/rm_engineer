#include "solve_core/planner/straight_planner.hpp"
#include "solve_core/calculate_tools/cost_func.hpp"
#include "solve_core/calculate_tools/hybrid_ik.hpp"
#include "solve_core/calculate_tools/wrap.hpp"
#include "solve_core/solve_core.hpp"
#include "log_utils/log.hpp"
#include <limits>
#include <cmath>
#include <algorithm>
#include <unordered_set>

namespace solve_core {

namespace {

// === 辅助：向量哈希/比较用于去重（简单做法，缩放并四舍五入） ===
static std::string vec_key_rounded(const std::vector<double>& v, double eps = 1e-4) {
  std::string k;
  k.reserve(v.size() * 8);
  for (double x : v) {
    long r = static_cast<long>(std::round(x / eps));
    k += std::to_string(r);
    k.push_back(',');
  }
  return k;
}

// === 辅助：计算 q 相对某参考向量的 wrap 距离（L2） ===
static double wrap_distance_l2(const std::vector<double>& q,
                               const std::vector<double>& q_ref) {
  const size_t n = std::min(q.size(), q_ref.size());
  if (n == 0) return std::numeric_limits<double>::infinity();
  double sum = 0.0;
  for (size_t i = 0; i < n; ++i) {
    const double qi = ikc::wrapToNearby(q[i], q_ref[i]);
    const double d = qi - q_ref[i];
    sum += d * d;
  }
  return std::sqrt(sum);
}

} // namespace

StraightPlanner::StraightPlanner(
    const moveit::core::RobotModelConstPtr& model,
    const std::string& group_name,
    const std::string& ee_link)
  : robot_model_(model),
    group_name_(group_name),
    ee_link_(ee_link) {}

// === DP 版本的 plan() ===
std::optional<Trajectory>
StraightPlanner::plan(moveit::core::RobotState& start_state,
                      const Eigen::Isometry3d& target_pose,
                      const StraightPlannerOptions& opt,
                      const CostOptions& cost_opt,
                      std::vector<std::vector<double>>* joint_path_out) {
  if (!robot_model_) {
    LOGE("[solve_core][straight_planner] Robot model is null");
    return std::nullopt;
  }
  if (group_name_.empty()) {
    LOGE("[solve_core][straight_planner] Planning group is empty");
    return std::nullopt;
  }
  if (ee_link_.empty()) {
    LOGE("[solve_core][straight_planner] End-effector link is empty");
    return std::nullopt;
  }
  if (opt.num_waypoints <= 0) {
    LOGE("[solve_core][straight_planner] Invalid num_waypoints: {}", opt.num_waypoints);
    return std::nullopt;
  }

  const auto* jmg = robot_model_->getJointModelGroup(group_name_);
  const auto* ee_link = robot_model_->getLinkModel(ee_link_);
  if (!jmg || !ee_link) {
    LOGE("[solve_core][straight_planner] Invalid model handles: jmg={}, link={}",
         jmg != nullptr, ee_link != nullptr);
    return std::nullopt;
  }

  HybridIK ik(robot_model_, group_name_, ee_link_);
  IKOptions ik_opt;
  ik_opt.log();

  // 起始末端位姿（用于生成直线离散路点）
  Eigen::Isometry3d T0 = start_state.getGlobalLinkTransform(ee_link);
  Eigen::Vector3d line_delta = Eigen::Vector3d::Zero();
  if (opt.use_directional_sampling) {
    Eigen::Vector3d vector(opt.direction_x, opt.direction_y, opt.direction_z);
    const double norm = vector.norm();  // 计算方向向量的模长
    if (opt.sample_step_m > 0.0 && norm > 1e-9) {
      vector /= norm;
      line_delta = vector * (opt.sample_step_m * static_cast<double>(opt.num_waypoints)); // 计算总的位移增量

      // Debug: log T0, direction, and first waypoint pose (T1)
      const Eigen::Quaterniond q0(T0.linear());
      const Eigen::Vector3d t0 = T0.translation();
      const double r1 = 1.0 / static_cast<double>(opt.num_waypoints);
      Eigen::Isometry3d T1 = Eigen::Isometry3d::Identity();
      T1.translation() = t0 + line_delta * r1;
      T1.linear() = T0.linear();
      const Eigen::Quaterniond q1(T1.linear());
      LOGI("[solve_core][straight_planner] T0: pos=({:.6f}, {:.6f}, {:.6f}) quat=({:.6f}, {:.6f}, {:.6f}, {:.6f})",
           t0.x(), t0.y(), t0.z(), q0.x(), q0.y(), q0.z(), q0.w());
      LOGI("[solve_core][straight_planner] dir(unit)=({:.6f}, {:.6f}, {:.6f}) sample_step={:.6f} num_waypoints={} line_delta=({:.6f}, {:.6f}, {:.6f})",
           vector.x(), vector.y(), vector.z(),
           opt.sample_step_m, opt.num_waypoints,
           line_delta.x(), line_delta.y(), line_delta.z());
      LOGI("[solve_core][straight_planner] T1: pos=({:.6f}, {:.6f}, {:.6f}) quat=({:.6f}, {:.6f}, {:.6f}, {:.6f})",
           T1.translation().x(), T1.translation().y(), T1.translation().z(),
           q1.x(), q1.y(), q1.z(), q1.w());
    } else {
      LOGE("[solve_core][straight_planner] Invalid directional sampling params");
      return std::nullopt;
    }
  } else {
    LOGE("[solve_core][straight_planner] Directional sampling must be enabled");
    return std::nullopt;
  }

  // 每个路点的候选解集合（sols_per_waypoint[i] = vector of q vectors）
  std::vector<std::vector<std::vector<double>>> sols_per_waypoint;
  sols_per_waypoint.reserve(opt.num_waypoints);

  // 用于生成候选解的随机种子：对第一个路点我们用 start_state 作为 seed，
  // 对后续路点我们用上层候选解作为 seed（见下面循环）。
  // 注意：为了控制复杂度，我们限制每层候选解数量（cap_candidates）
  const int cap_candidates = 10; // 每路点候选解上限（可调）

  // 1) 逐路点生成候选解集（使用“从前一层候选解作为 seed”策略）
  // prev_seeds 初始为只有 start_state
  std::vector<std::vector<double>> prev_solutions;    //prev = previous 上一个的
  {
    // prev_solutions 初始化为 start_state 的关节值（作为一种 seed 占位）
    std::vector<double> q0;
    start_state.copyJointGroupPositions(jmg, q0);
    prev_solutions.push_back(q0);
  }

  for (int i = 1; i <= opt.num_waypoints; ++i) {
    double r = double(i) / opt.num_waypoints;
    Eigen::Isometry3d Ti = Eigen::Isometry3d::Identity();   //用单位矩阵初始化路点 i = insert
    Ti.translation() =
        T0.translation() +
        line_delta * r;
    Ti.linear() = T0.linear();    // linear 旋转矩阵

    // 收集该路点所有候选解（从每个 prev_solution 作为 seed 去调用 ik.solveAll）
    std::vector<std::vector<double>> candidates;

    // 去重哈希表：存已见过的解的 key
    std::unordered_set<std::string> seen_keys;

    // 对每个 prev seed（prev_solutions），用其生成该路点的候选解
    for (const auto& prev_q : prev_solutions) {
      // 构造一个 RobotState 作为 seed
      moveit::core::RobotState seed_state(start_state);
      seed_state.setJointGroupPositions(jmg, prev_q);
      seed_state.enforceBounds(jmg);
      seed_state.update();

      std::vector<std::vector<double>> sols;
      if (!ik.solveAll(seed_state, Ti, ik_opt, sols)) {
        // 该 seed 无解则跳过（并继续尝试其它 prev seed）
        continue;
      }

      // 将 sols 收集到 candidates，先去重，稍后排序再截断
      for (auto &q : sols) {
        // 简单去重：基于四舍五入后的字符串 key
        std::string key = vec_key_rounded(q, 1e-4);
        if (seen_keys.insert(key).second) {
          candidates.push_back(std::move(q));
        }
      }
    }

    // 如果该路点没有候选解则整个规划失败
    // option2: 回退
    if (candidates.empty()) {
      LOGE("[solve_core][straight_planner] No IK candidates at waypoint {}", i);

      return std::nullopt;
    }

    // 按与上一层解的 wrap 距离排序，再截断到 cap_candidates
    std::vector<std::pair<double, std::vector<double>>> scored;
    scored.reserve(candidates.size());
    for (auto &q : candidates) {
      double best = std::numeric_limits<double>::infinity();
      for (const auto &prev_q : prev_solutions) {
        best = std::min(best, wrap_distance_l2(q, prev_q));
      }
      scored.emplace_back(best, std::move(q));
    }
    std::sort(scored.begin(), scored.end(),
              [](const auto &a, const auto &b) { return a.first < b.first; });
    candidates.clear();
    const size_t keep = std::min(scored.size(), static_cast<size_t>(cap_candidates));
    candidates.reserve(keep);
    for (size_t idx = 0; idx < keep; ++idx) {
      candidates.push_back(std::move(scored[idx].second));
    }

    // 保存该路点候选解，并作为下一层 prev_solutions 的来源
    sols_per_waypoint.push_back(candidates);
    prev_solutions = candidates; // 下一轮用当前候选作为 seed 来源
  } // end for waypoints

  // 2) 在候选解集合上做动态规划找到总代价最小路径
  // DP 数据结构：dp_costs[i][j] = 最小代价到第 i 层第 j 个候选
  // prev_idx[i][j] = 使 dp_costs[i][j] 最小的上一层索引
  const size_t N = sols_per_waypoint.size();
  std::vector<std::vector<double>> dp_costs(N);
  std::vector<std::vector<int>> prev_idx(N);

  CostFunc cost_func(start_state, jmg, ee_link, cost_opt);

  // 初始化第一层 dp（从 start_state 到第一层每个候选的代价）
  dp_costs[0].assign(sols_per_waypoint[0].size(), std::numeric_limits<double>::infinity());
  prev_idx[0].assign(sols_per_waypoint[0].size(), -1);

  for (size_t j = 0; j < sols_per_waypoint[0].size(); ++j) {
    const auto& qj = sols_per_waypoint[0][j];

    // 代价 = continuity from start_state + condition penalty
    std::vector<double> q0;
    start_state.copyJointGroupPositions(jmg, q0);

    dp_costs[0][j] = cost_func.compute(q0, qj);
  }

  // 递推：对每层 i>=1
  for (size_t i = 1; i < N; ++i) {
    const auto& prev_layer = sols_per_waypoint[i-1];
    const auto& cur_layer = sols_per_waypoint[i];

    dp_costs[i].assign(cur_layer.size(), std::numeric_limits<double>::infinity());
    prev_idx[i].assign(cur_layer.size(), -1);

    for (size_t j = 0; j < cur_layer.size(); ++j) {
      const auto& qj = cur_layer[j];

      for (size_t k = 0; k < prev_layer.size(); ++k) {
        if (!std::isfinite(dp_costs[i-1][k])) continue;
        const auto& qk = prev_layer[k];

        double total = dp_costs[i-1][k] + cost_func.compute(qk, qj);

        if (total < dp_costs[i][j]) {
          dp_costs[i][j] = total;
          prev_idx[i][j] = static_cast<int>(k);
        }
      }
    }

    // 如果本层所有 dp_costs 都是 inf（没有可连通的候选），直接失败
    bool any_ok = false;
    for (double v : dp_costs[i]) {
      if (std::isfinite(v)) { any_ok = true; break; }
    }
    if (!any_ok) {
      LOGE("[solve_core][straight_planner] DP disconnected at layer {}", i);
      return std::nullopt;
    }
  }

  // 3) 回溯得到最优路径（最后一层选择最小 dp）
  std::vector<std::vector<double>> best_path_rev;
  best_path_rev.reserve(N);

  // 找到最后一层最小代价索引
  size_t last_idx = 0;
  double best_last_cost = std::numeric_limits<double>::infinity();
  for (size_t j = 0; j < dp_costs[N-1].size(); ++j) {
    if (dp_costs[N-1][j] < best_last_cost) {
      best_last_cost = dp_costs[N-1][j];
      last_idx = j;
    }
  }

  // 回溯
  int cur_idx = static_cast<int>(last_idx);
  for (int i = static_cast<int>(N) - 1; i >= 0; --i) {
    best_path_rev.push_back(sols_per_waypoint[i][cur_idx]);
    cur_idx = prev_idx[i][cur_idx];
    if (i > 0 && cur_idx < 0) {
      // 不应该发生 —— 表示回溯失败
      LOGE("[solve_core][straight_planner] DP traceback failed at layer {}", i);
      return std::nullopt;
    }
  }

  // 反转得到顺序
  std::vector<std::vector<double>> q_path;
  q_path.reserve(N);
  for (auto it = best_path_rev.rbegin(); it != best_path_rev.rend(); ++it)
    q_path.push_back(*it);

  // 关节角度线性插值：已移除，直接使用路径点
  std::vector<std::vector<double>> q_path_interp = q_path;

  // Wrap the final joint sequence for continuity (in-place).
  ikc::unwrapTrajectory(q_path_interp);

  // 可选输出
  if (joint_path_out)
    *joint_path_out = q_path_interp;

  Trajectory traj;
  traj.joint_names = jmg->getVariableNames();
  traj.points.reserve(q_path_interp.size());
  for (const auto& q : q_path_interp) {
    TrajectoryPoint pt;
    pt.positions = q;
    traj.points.push_back(std::move(pt));
  }

  return traj;
;
}

} // namespace solve_core

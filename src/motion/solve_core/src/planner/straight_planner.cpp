#include "solve_core/planner/straight_planner.hpp"
#include "solve_core/calculate_tools/cost_func.hpp"
#include "solve_core/calculate_tools/hybrid_ik.hpp"
#include "solve_core/solve_core.hpp"
#include <limits>
#include <cmath>
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
                      std::vector<std::vector<double>>* joint_path_out) {

  const auto* jmg = robot_model_->getJointModelGroup(group_name_);
  const auto* link = robot_model_->getLinkModel(ee_link_);
  if (!jmg || !link) return std::nullopt;

  HybridIK ik(robot_model_, group_name_, ee_link_);
  IKOptions ik_opt;

  // 起始末端位姿（只用于生成插值位姿）
  Eigen::Isometry3d T0 = start_state.getGlobalLinkTransform(link);

  // 每个路点的候选解集合（sols_per_waypoint[i] = vector of q vectors）
  std::vector<std::vector<std::vector<double>>> sols_per_waypoint;
  sols_per_waypoint.reserve(opt.num_waypoints);

  // 用于生成候选解的随机种子：对第一个路点我们用 start_state 作为 seed，
  // 对后续路点我们用上层候选解作为 seed（见下面循环）。
  // 注意：为了控制复杂度，我们限制每层候选解数量（cap_candidates）
  const int cap_candidates = 12; // 每路点候选解上限（可调）

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
        (target_pose.translation() - T0.translation()) * r;
    Ti.linear() = T0.linear();    // linear 旋转矩阵

    // 收集该路点所有候选解（从每个 prev_solution 作为 seed 去调用 ik.solveAll）
    std::vector<std::vector<double>> candidates;
    candidates.reserve(cap_candidates);

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

      // 将 sols 收集到 candidates，去重并且限制总数 cap_candidates
      for (auto &q : sols) {
        if ((int)candidates.size() >= cap_candidates) break;

        // 简单去重：基于四舍五入后的字符串 key
        std::string key = vec_key_rounded(q, 1e-4);
        if (seen_keys.insert(key).second) {
          candidates.push_back(std::move(q));
        }
      }

      if ((int)candidates.size() >= cap_candidates) break;
    }

    // 如果该路点没有候选解则整个规划失败
    if (candidates.empty()) return std::nullopt;

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

  CostFunc cost_func(start_state, jmg, link);

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
    if (!any_ok) return std::nullopt;
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
      return std::nullopt;
    }
  }

  // 反转得到顺序
  std::vector<std::vector<double>> q_path;
  q_path.reserve(N);
  for (auto it = best_path_rev.rbegin(); it != best_path_rev.rend(); ++it)
    q_path.push_back(*it);

  // 可选输出
  if (joint_path_out)
    *joint_path_out = q_path;

  Trajectory traj;
  traj.joint_names = jmg->getVariableNames();
  traj.points.reserve(q_path.size());
  for (const auto& q : q_path) {
    TrajectoryPoint pt;
    pt.positions = q;
    traj.points.push_back(std::move(pt));
  }

  return traj;
;
}

} // namespace solve_core

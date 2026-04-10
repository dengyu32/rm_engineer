#include "planner/straight_planner.hpp"
#include <Eigen/src/Core/Matrix.h>
#include "calculator/cost_func.hpp"
#include "types/types.hpp"
#include "calculator/hybrid_ik.hpp"
#include "log_tools/log.hpp"
#include <limits>
#include <cmath>
#include <algorithm>
#include <unordered_set>

namespace planner
{
using types::Pose;
using types::Trajectory;

namespace
{
//
// 辅助函数：请求有效性检验，向量单位化处理
// ------------------------------------------------------------------------------------------------------
// bool is_valid()
// {
//   return true;
// }
// ------------------------------------------------------------------------------------------------------
// 辅助函数：将一个 double vector 中的元素扩大大倍数后得到离散值，将原vector组装为一个字符串 key（用于去重，基于四舍五入）
// ------------------------------------------------------------------------------------------------------
static std::string vec_key_rounded(const std::vector<double>& v, double eps = 1e-4)
{
  std::string k;
  k.reserve(v.size() * 8);
  for (double x : v)
  {
    long r = static_cast<long>(std::round(x / eps));
    k += std::to_string(r);
    k.push_back(',');
  }
  return k;
}

// ------------------------------------------------------------------------------------------------------
// 辅助函数：计算两个关节向量的 L2 距离（用于 DP 连续性评分）
// ------------------------------------------------------------------------------------------------------
static double distance_l2(const std::vector<double>& q, const std::vector<double>& q_ref)
{
  const size_t n = std::min(q.size(), q_ref.size());
  if (n == 0)
    return std::numeric_limits<double>::infinity();    //代价无穷大
  double sum = 0.0;
  for (size_t i = 0; i < n; ++i)
  {
    const double d = q[i] - q_ref[i];
    sum += d * d;
  }
  return std::sqrt(sum);
}

// ------------------------------------------------------------------------------------------------------
// 辅助函数：计算两个关节向量中单关节的最大角度变化量（用于连续性评分打破并列）
// ------------------------------------------------------------------------------------------------------
static double max_joint_delta(const std::vector<double>& q, const std::vector<double>& q_ref)
{
  const size_t n = std::min(q.size(), q_ref.size());
  double max_delta = 0.0;
  for (size_t i = 0; i < n; ++i)
  {
    max_delta = std::max(max_delta, std::abs(q[i] - q_ref[i]));
  }
  return max_delta;
}

}    // namespace

StraightPlanner::StraightPlanner(const moveit::core::RobotModelConstPtr& model, const std::string& group_name,
                                 const std::string& ee_link, const StraightPlannerSettings& settings,
                                 const collision::SelfCollisionDetector* self_collision_detector)
  : robot_model_(model)
  , group_name_(group_name)
  , ee_link_(ee_link)
  , settings_(settings)
  , self_collision_detector_(self_collision_detector)
{
}

std::optional<Trajectory> StraightPlanner::plan(moveit::core::RobotState& start_state, Eigen::Vector3d& target_vector,
                                                double target_length, const calculator::CostOptions& cost_opt)
{
  LOGI("Start Plan Cartesian!");

  const auto* jmg = robot_model_->getJointModelGroup(group_name_);
  const auto* ee_link = robot_model_->getLinkModel(ee_link_);
  const auto* ref_link = robot_model_->getLinkModel(settings_.reference_link);
  if (!jmg || !ee_link || !ref_link)
  {
    LOGE("[solve_executor][straight_planner] Invalid model handles: jmg={}, ee_link={}, ref_link={}", jmg != nullptr,
         ee_link != nullptr, ref_link != nullptr);
    return std::nullopt;
  }

  calculator::HybridIK ik(robot_model_, group_name_, ee_link_);
  calculator::IKOptions ik_opt;

  // {
  // 如果视觉不传方向直接使用link6的x轴方向的话，采用这部分代码
  // const Eigen::Vector3d normalized_target = target_vector.normalized();
  // const Eigen::Isometry3d T_ref = start_state.getGlobalLinkTransform(ref_link);
  Eigen::Isometry3d T0 = start_state.getGlobalLinkTransform(ee_link);    // 起始末端位姿
  // const Eigen::Matrix3d R_ref_ee = T_ref.linear().transpose() * T0.linear();
  // const Eigen::Vector3d ee_x_in_ref = R_ref_ee.col(0).normalized();
  // const double direction_dot = ee_x_in_ref.dot(normalized_target);
  // const double abs_direction_dot = std::abs(direction_dot);
  // if (!std::isfinite(abs_direction_dot) || abs_direction_dot < settings_.alignment_dot_threshold)
  // {
  //   LOGE("[solve_executor][straight_planner] Alignment check failed: abs(dot)={}, threshold={}", abs_direction_dot,
  //        settings_.alignment_dot_threshold);
  //   return std::nullopt;
  // }

  // const Eigen::Vector3d straight_dir = (direction_dot >= 0.0 ? 1.0 : -1.0) * ee_x_in_ref;
  // target_vector = straight_dir;
  // }
  const Eigen::Vector3d line_delta = target_vector * target_length;
  const int num_waypoints = std::max(1, static_cast<int>(std::ceil(target_length / settings_.sample_step_m)));

  const int cap_sols_per_waypoint = 10;                               // 每路点候选解上限
  std::vector<std::vector<std::vector<double>>> sols_all_waypoint;    // 所有路点的候选解集合
  sols_all_waypoint.reserve(num_waypoints);

  std::vector<std::vector<double>> prev_solutions;    // prev = previous 上一个的
  {
    std::vector<double> q0;
    start_state.copyJointGroupPositions(jmg, q0);
    prev_solutions.push_back(q0);
  }

  for (int i = 1; i <= num_waypoints; ++i)
  {
    double r = double(i) / num_waypoints;
    Eigen::Isometry3d Ti = Eigen::Isometry3d::Identity();    //用单位矩阵初始化路点
    Ti.translation() = T0.translation() + line_delta * r;
    Ti.linear() = T0.linear();    // linear 旋转矩阵

    std::vector<std::vector<double>> sols_per_waypoint;    // 收集该路点所有候选解
    std::unordered_set<std::string> seen_keys;             // 去重哈希表

    for (const auto& prev_q : prev_solutions)
    {
      moveit::core::RobotState seed_state(start_state);
      seed_state.setJointGroupPositions(jmg, prev_q);
      seed_state.enforceBounds(jmg);
      seed_state.update();

      std::vector<std::vector<double>> sols;
      if (!ik.solveAll(seed_state, Ti, ik_opt, sols))
      {
        LOGE("[solve_executor][straight_planner] IK solveAll failed for waypoint with current seed,changing to next "
             "seed "
             "...");
        continue;
      }

      for (auto& q : sols)
      {
        if (self_collision_detector_)
        {
          collision::SelfCollisionCheckResult collision_result;
          if (!self_collision_detector_->check(q, collision_result))
          {
            LOGE("[solve_executor][straight_planner] Filter IK solution by self collision: {}",
                 collision_result.message);
            continue;
          }
        }

        std::string key = vec_key_rounded(q, 1e-4);
        if (seen_keys.insert(key).second)
        {
          sols_per_waypoint.push_back(std::move(q));
        }
      }
    }
    if (sols_per_waypoint.empty())
    {
      LOGE("[solve_executor][straight_planner] No IK sols_per_waypoint at waypoint {}", i);

      return std::nullopt;
    }

    // 按与上一层解的连续性距离排序
    struct ScoredSolution
    {
      double continuity;
      double max_delta;
      std::vector<double> q;
    };
    std::vector<ScoredSolution> scored;    //解及其连续性评分
    scored.reserve(sols_per_waypoint.size());
    constexpr double kScoreEps = 1e-12;
    for (auto& q : sols_per_waypoint)
    {
      double best = std::numeric_limits<double>::infinity();
      double best_max_delta = std::numeric_limits<double>::infinity();
      for (const auto& prev_q : prev_solutions)
      {
        const double dist = distance_l2(q, prev_q);
        const double delta = max_joint_delta(q, prev_q);
        if (dist + kScoreEps < best)
        {
          best = dist;
          best_max_delta = delta;
        }
        else if (std::abs(dist - best) <= kScoreEps && delta < best_max_delta)
        {
          best_max_delta = delta;
        }
      }
      scored.push_back({ best, best_max_delta, std::move(q) });
    }
    std::stable_sort(scored.begin(), scored.end(), [](const auto& a, const auto& b) {
      if (std::abs(a.continuity - b.continuity) > kScoreEps)
      {
        return a.continuity < b.continuity;
      }
      return a.max_delta < b.max_delta;
    });
    sols_per_waypoint.clear();
    const size_t keep = std::min(scored.size(), static_cast<size_t>(cap_sols_per_waypoint));
    sols_per_waypoint.reserve(keep);
    for (size_t idx = 0; idx < keep; ++idx)
    {
      sols_per_waypoint.push_back(std::move(scored[idx].q));
    }

    sols_all_waypoint.push_back(sols_per_waypoint);
    prev_solutions = sols_per_waypoint;
  }

  const size_t N = sols_all_waypoint.size();
  std::vector<std::vector<double>> dp_costs(N);    //代价
  std::vector<std::vector<int>> prev_idx(N);       //

  calculator::CostFunc cost_func(start_state, jmg, ee_link, cost_opt);

  // 初始化第一层 dp（从 start_state 到第一层每个候选的代价）
  dp_costs[0].assign(sols_all_waypoint[0].size(), std::numeric_limits<double>::infinity());
  prev_idx[0].assign(sols_all_waypoint[0].size(), -1);

  std::vector<double> q0;
  start_state.copyJointGroupPositions(jmg, q0);

  for (size_t j = 0; j < sols_all_waypoint[0].size(); ++j)
  {
    const auto& qj = sols_all_waypoint[0][j];
    // 代价 = continuity from start_state + condition penalty
    // 首层起点剪枝暂时关闭，只保留代价初始化；如需恢复可重新启用 q0->qj 的 jump limit 检查。
    dp_costs[0][j] = cost_func.compute(q0, qj);
  }

  // 递推：对每层 i>=1
  for (size_t i = 1; i < N; ++i)
  {
    const auto& prev_layer = sols_all_waypoint[i - 1];
    const auto& cur_layer = sols_all_waypoint[i];

    dp_costs[i].assign(cur_layer.size(), std::numeric_limits<double>::infinity());
    prev_idx[i].assign(cur_layer.size(), -1);
    bool all_edges_rejected_by_jump_limit = true;

    for (size_t j = 0; j < cur_layer.size(); ++j)
    {    //遍历当前路点的所有解
      const auto& qj = cur_layer[j];

      for (size_t k = 0; k < prev_layer.size(); ++k)
      {    //遍历前一点的所有解
        if (!std::isfinite(dp_costs[i - 1][k]))
          continue;
        const auto& qk = prev_layer[k];
        const double delta = max_joint_delta(qk, qj);
        if (!std::isfinite(delta) || delta > settings_.max_joint_jump_rad)
        {
          continue;
        }
        all_edges_rejected_by_jump_limit = false;

        double total =
            dp_costs[i - 1][k] +
            cost_func.compute(qk, qj);    //前一个路点第k个解的代价加上当前点与前一个点第k个解的代价为此路径的总代价

        if (total < dp_costs[i][j])
        {
          dp_costs[i][j] = total;
          prev_idx[i][j] = static_cast<int>(k);
        }
      }
    }

    bool any_ok = false;    // 如果本层所有 dp_costs 都是 inf（没有可连通的候选），直接失败
    for (double v : dp_costs[i])
    {
      if (std::isfinite(v))
      {
        any_ok = true;
        break;
      }
    }
    if (!any_ok)
    {
      if (all_edges_rejected_by_jump_limit)
      {
        LOGE("[solve_executor][straight_planner] All transitions rejected by max_joint_jump_rad at layer {}: "
             "threshold={}, prev_candidates={}, cur_candidates={}",
             i, settings_.max_joint_jump_rad, prev_layer.size(), cur_layer.size());
      }
      LOGE("[solve_executor][straight_planner] DP disconnected at layer {}", i);
      return std::nullopt;
    }
  }

  // 回溯得到最优路径
  std::vector<std::vector<double>> best_path_rev;
  best_path_rev.reserve(N);

  // 找到最后一层最小代价索引
  size_t last_idx = 0;
  double best_last_cost = std::numeric_limits<double>::infinity();
  for (size_t j = 0; j < dp_costs[N - 1].size(); ++j)
  {
    if (dp_costs[N - 1][j] < best_last_cost)
    {
      best_last_cost = dp_costs[N - 1][j];
      last_idx = j;
    }
  }

  // 回溯
  int cur_idx = static_cast<int>(last_idx);
  for (int i = static_cast<int>(N) - 1; i >= 0; --i)
  {
    best_path_rev.push_back(sols_all_waypoint[i][cur_idx]);
    cur_idx = prev_idx[i][cur_idx];
    if (i > 0 && cur_idx < 0)
    {
      // 不应该发生 —— 表示回溯失败
      LOGE("[solve_executor][straight_planner] DP traceback failed at layer {}", i);
      return std::nullopt;
    }
  }

  // 反转得到顺序
  std::vector<std::vector<double>> q_path;
  q_path.reserve(N);
  for (auto it = best_path_rev.rbegin(); it != best_path_rev.rend(); ++it)
    q_path.push_back(*it);

  // 将真实起始关节作为轨迹首点纳入后续 TOTG，避免首段未时间参数化导致突变。
  std::vector<double> q_start;
  start_state.copyJointGroupPositions(jmg, q_start);

  std::vector<std::vector<double>> q_path_interp;
  q_path_interp.reserve(q_path.size() + 8);
  q_path_interp.push_back(q_start);
  if (!q_path.empty())
  {
    const auto& q_first = q_path.front();
    const double max_delta_to_first = max_joint_delta(q_start, q_first);
    if (std::isfinite(max_delta_to_first) && settings_.max_joint_jump_rad > 1e-9)
    {
      const int start_segments =
          std::max(1, static_cast<int>(std::ceil(max_delta_to_first / settings_.max_joint_jump_rad)));
      for (int seg = 1; seg < start_segments; ++seg)
      {
        const double t = static_cast<double>(seg) / static_cast<double>(start_segments);
        std::vector<double> q_mid(q_start.size(), 0.0);
        for (std::size_t idx = 0; idx < q_mid.size(); ++idx)
        {
          q_mid[idx] = q_start[idx] + (q_first[idx] - q_start[idx]) * t;
        }
        q_path_interp.push_back(std::move(q_mid));
      }
    }
  }
  for (auto& q : q_path)
  {
    q_path_interp.push_back(std::move(q));
  }

  Trajectory traj;
  traj.joint_names = jmg->getVariableNames();
  traj.points.reserve(q_path_interp.size());
  for (const auto& q : q_path_interp)
  {
    types::TrajectoryPoint pt;
    pt.positions = q;
    traj.points.push_back(std::move(pt));
  }

  return traj;
}

}    // namespace planner


// #include "planner/straight_planner.hpp"
// #include <Eigen/src/Core/Matrix.h>
// #include "calculator/cost_func.hpp"
// #include "types/types.hpp"
// #include "calculator/hybrid_ik.hpp"
// #include "log_tools/log.hpp"
// #include <limits>
// #include <cmath>
// #include <algorithm>
// #include <unordered_set>

// namespace planner
// {
// using types::Pose;
// using types::Trajectory;

// namespace
// {
// // 辅助函数：将一个 double vector 中的元素扩大大倍数后得到离散值，组装为一个字符串 key（用于去重）
// static std::string vec_key_rounded(const std::vector<double>& v, double eps = 1e-4)
// {
//   std::string k;
//   k.reserve(v.size() * 8);
//   for (double x : v)
//   {
//     long r = static_cast<long>(std::round(x / eps));
//     k += std::to_string(r);
//     k.push_back(',');
//   }
//   return k;
// }

// // 辅助函数：计算两个关节向量中单关节的最大角度变化量（用于 DP 判断 jump limit）
// static double max_joint_delta(const std::vector<double>& q, const std::vector<double>& q_ref)
// {
//   const size_t n = std::min(q.size(), q_ref.size());
//   double max_delta = 0.0;
//   for (size_t i = 0; i < n; ++i)
//   {
//     max_delta = std::max(max_delta, std::abs(q[i] - q_ref[i]));
//   }
//   return max_delta;
// }

// }    // namespace

// StraightPlanner::StraightPlanner(const moveit::core::RobotModelConstPtr& model, const std::string& group_name,
//                                  const std::string& ee_link, const StraightPlannerSettings& settings,
//                                  const collision::SelfCollisionDetector* self_collision_detector)
//   : robot_model_(model)
//   , group_name_(group_name)
//   , ee_link_(ee_link)
//   , settings_(settings)
//   , self_collision_detector_(self_collision_detector)
// {
// }

// std::optional<Trajectory> StraightPlanner::plan(moveit::core::RobotState& start_state, Eigen::Vector3d& target_vector,
//                                                 double target_length, const calculator::CostOptions& cost_opt)
// {
//   LOGI("Start Plan Cartesian!");

//   const auto* jmg = robot_model_->getJointModelGroup(group_name_);
//   const auto* ee_link = robot_model_->getLinkModel(ee_link_);
//   const auto* ref_link = robot_model_->getLinkModel(settings_.reference_link);
//   if (!jmg || !ee_link || !ref_link)
//   {
//     LOGE("[solve_executor][straight_planner] Invalid model handles");
//     return std::nullopt;
//   }

//   calculator::HybridIK ik(robot_model_, group_name_, ee_link_);
//   calculator::IKOptions ik_opt;

//   Eigen::Isometry3d T0 = start_state.getGlobalLinkTransform(ee_link);    // 起始末端位姿
//   const Eigen::Vector3d line_delta = target_vector * target_length;
//   const int num_waypoints = std::max(1, static_cast<int>(std::ceil(target_length / settings_.sample_step_m)));

//   const int cap_sols_per_waypoint = 10;                               // 每路点候选解上限
//   std::vector<std::vector<std::vector<double>>> sols_all_waypoint;    // 所有路点的候选解集合
//   sols_all_waypoint.reserve(num_waypoints);

//   std::vector<std::vector<double>> prev_solutions;
//   {
//     std::vector<double> q0;
//     start_state.copyJointGroupPositions(jmg, q0);
//     prev_solutions.push_back(q0);
//   }

//   for (int i = 1; i <= num_waypoints; ++i)
//   {
//     double r = double(i) / num_waypoints;
//     Eigen::Isometry3d Ti = Eigen::Isometry3d::Identity();
//     Ti.translation() = T0.translation() + line_delta * r;
//     Ti.linear() = T0.linear();

//     std::vector<std::vector<double>> sols_per_waypoint;
//     std::unordered_set<std::string> seen_keys;

//     for (const auto& prev_q : prev_solutions)
//     {
//       moveit::core::RobotState seed_state(start_state);
//       seed_state.setJointGroupPositions(jmg, prev_q);
//       seed_state.enforceBounds(jmg);
//       seed_state.update();

//       std::vector<std::vector<double>> sols;
//       if (!ik.solveAll(seed_state, Ti, ik_opt, sols))
//       {
//         continue;
//       }

//       // HybridIK 内部已经按照最连续（距离 seed_state 最近）对解进行了排序
//       // 我们在此只需从前往后取即可，同时施加碰撞和去重过滤
//       int added_for_this_seed = 0;
//       for (auto& q : sols)
//       {
//         if (self_collision_detector_)
//         {
//           collision::SelfCollisionCheckResult collision_result;
//           if (!self_collision_detector_->check(q, collision_result))
//           {
//             continue;
//           }
//         }

//         std::string key = vec_key_rounded(q, 1e-4);
//         if (seen_keys.insert(key).second)
//         {
//           sols_per_waypoint.push_back(std::move(q));
//           added_for_this_seed++;
          
//           // 每个 seed 最多提取一定数量的优质解以避免状态空间爆炸
//           if (added_for_this_seed >= cap_sols_per_waypoint) {
//               break;
//           }
//         }
//       }
//     }

//     if (sols_per_waypoint.empty())
//     {
//       LOGE("[solve_executor][straight_planner] No IK sols_per_waypoint at waypoint {}", i);
//       return std::nullopt;
//     }

//     // 截断该路点总的解数量，维持 DP 计算量在一个稳定水平
//     if (sols_per_waypoint.size() > static_cast<size_t>(cap_sols_per_waypoint)) {
//         sols_per_waypoint.resize(cap_sols_per_waypoint);
//     }

//     sols_all_waypoint.push_back(sols_per_waypoint);
//     prev_solutions = sols_per_waypoint;
//   }

//   const size_t N = sols_all_waypoint.size();
//   std::vector<std::vector<double>> dp_costs(N);    //代价
//   std::vector<std::vector<int>> prev_idx(N);       //前缀索引

//   calculator::CostFunc cost_func(start_state, jmg, ee_link, cost_opt);

//   // 初始化第一层 dp（从 start_state 到第一层每个候选的代价）
//   dp_costs[0].assign(sols_all_waypoint[0].size(), std::numeric_limits<double>::infinity());
//   prev_idx[0].assign(sols_all_waypoint[0].size(), -1);

//   std::vector<double> q0;
//   start_state.copyJointGroupPositions(jmg, q0);

//   for (size_t j = 0; j < sols_all_waypoint[0].size(); ++j)
//   {
//     const auto& qj = sols_all_waypoint[0][j];
//     dp_costs[0][j] = cost_func.compute(q0, qj);
//   }

//   // DP 递推：对每层 i>=1
//   for (size_t i = 1; i < N; ++i)
//   {
//     const auto& prev_layer = sols_all_waypoint[i - 1];
//     const auto& cur_layer = sols_all_waypoint[i];

//     dp_costs[i].assign(cur_layer.size(), std::numeric_limits<double>::infinity());
//     prev_idx[i].assign(cur_layer.size(), -1);
//     bool all_edges_rejected_by_jump_limit = true;

//     for (size_t j = 0; j < cur_layer.size(); ++j)
//     {
//       const auto& qj = cur_layer[j];

//       for (size_t k = 0; k < prev_layer.size(); ++k)
//       {
//         if (!std::isfinite(dp_costs[i - 1][k]))
//           continue;
        
//         const auto& qk = prev_layer[k];
//         const double delta = max_joint_delta(qk, qj);
        
//         if (!std::isfinite(delta) || delta > settings_.max_joint_jump_rad)
//         {
//           continue;
//         }
//         all_edges_rejected_by_jump_limit = false;

//         double total = dp_costs[i - 1][k] + cost_func.compute(qk, qj);

//         if (total < dp_costs[i][j])
//         {
//           dp_costs[i][j] = total;
//           prev_idx[i][j] = static_cast<int>(k);
//         }
//       }
//     }

//     bool any_ok = false;
//     for (double v : dp_costs[i])
//     {
//       if (std::isfinite(v))
//       {
//         any_ok = true;
//         break;
//       }
//     }
    
//     if (!any_ok)
//     {
//       if (all_edges_rejected_by_jump_limit)
//       {
//         LOGE("[solve_executor][straight_planner] All transitions rejected by max_joint_jump_rad at layer {}", i);
//       }
//       LOGE("[solve_executor][straight_planner] DP disconnected at layer {}", i);
//       return std::nullopt;
//     }
//   }

//   // 回溯得到最优路径
//   std::vector<std::vector<double>> best_path_rev;
//   best_path_rev.reserve(N);

//   size_t last_idx = 0;
//   double best_last_cost = std::numeric_limits<double>::infinity();
//   for (size_t j = 0; j < dp_costs[N - 1].size(); ++j)
//   {
//     if (dp_costs[N - 1][j] < best_last_cost)
//     {
//       best_last_cost = dp_costs[N - 1][j];
//       last_idx = j;
//     }
//   }

//   int cur_idx = static_cast<int>(last_idx);
//   for (int i = static_cast<int>(N) - 1; i >= 0; --i)
//   {
//     best_path_rev.push_back(sols_all_waypoint[i][cur_idx]);
//     cur_idx = prev_idx[i][cur_idx];
//     if (i > 0 && cur_idx < 0)
//     {
//       LOGE("[solve_executor][straight_planner] DP traceback failed at layer {}", i);
//       return std::nullopt;
//     }
//   }

//   std::vector<std::vector<double>> q_path;
//   q_path.reserve(N);
//   for (auto it = best_path_rev.rbegin(); it != best_path_rev.rend(); ++it)
//     q_path.push_back(*it);

//   // 首点平滑插值，避免突变
//   std::vector<double> q_start;
//   start_state.copyJointGroupPositions(jmg, q_start);

//   std::vector<std::vector<double>> q_path_interp;
//   q_path_interp.reserve(q_path.size() + 8);
//   q_path_interp.push_back(q_start);
//   if (!q_path.empty())
//   {
//     const auto& q_first = q_path.front();
//     const double max_delta_to_first = max_joint_delta(q_start, q_first);
//     if (std::isfinite(max_delta_to_first) && settings_.max_joint_jump_rad > 1e-9)
//     {
//       const int start_segments =
//           std::max(1, static_cast<int>(std::ceil(max_delta_to_first / settings_.max_joint_jump_rad)));
//       for (int seg = 1; seg < start_segments; ++seg)
//       {
//         const double t = static_cast<double>(seg) / static_cast<double>(start_segments);
//         std::vector<double> q_mid(q_start.size(), 0.0);
//         for (std::size_t idx = 0; idx < q_mid.size(); ++idx)
//         {
//           q_mid[idx] = q_start[idx] + (q_first[idx] - q_start[idx]) * t;
//         }
//         q_path_interp.push_back(std::move(q_mid));
//       }
//     }
//   }
  
//   for (auto& q : q_path)
//   {
//     q_path_interp.push_back(std::move(q));
//   }

//   Trajectory traj;
//   traj.joint_names = jmg->getVariableNames();
//   traj.points.reserve(q_path_interp.size());
//   for (const auto& q : q_path_interp)
//   {
//     types::TrajectoryPoint pt;
//     pt.positions = q;
//     traj.points.push_back(std::move(pt));
//   }

//   return traj;
// }

// }    // namespace planner
// #include "calculator/hybrid_ik.hpp"

// #include <random>
// #include <cmath>
// #include <algorithm>

// #include "log_tools/log.hpp"

// namespace calculator
// {

// //判断是否为关节添加了角度限制，确保每个关节都有限制
// static bool isFiniteVec(const std::vector<double>& q)
// {
//   for (double v : q)
//   {
//     if (!std::isfinite(v))
//       return false;  // isfinite() 判断是否为有限制
//   }
//   return true;
// }

// // 距离去重，如果两个解的关节角差相近，将被判定为同一个解
// static bool nearSame(const std::vector<double>& a, const std::vector<double>& b, double eps)
// {  // eps是去重阈值，
//   if (a.size() != b.size())
//     return false;
//   double m = 0.0;
//   for (size_t i = 0; i < a.size(); ++i)
//   {
//     m = std::max(m, std::abs(a[i] - b[i]));
//   }
//   return m < eps;
// }

// HybridIK::HybridIK(const moveit::core::RobotModelConstPtr& model, const std::string& group_name,
//                    const std::string& ee_link)
//   : robot_model_(model), group_name_(group_name), ee_link_(ee_link)
// {
// }

// bool HybridIK::solveAll(const moveit::core::RobotState& seed_state, const Eigen::Isometry3d& target_pose,
//                         const IKOptions& opt, std::vector<std::vector<double>>& solutions) const
// {
//   solutions.clear();

//   if (!robot_model_)
//   {
//     LOGE("[solve_executor][hybrid_ik] Robot model is null");
//     return false;
//   }
//   if (group_name_.empty())
//   {
//     LOGE("[solve_executor][hybrid_ik] Planning group is empty");
//     return false;
//   }
//   if (ee_link_.empty())
//   {
//     LOGE("[solve_executor][hybrid_ik] End-effector link is empty");
//     return false;
//   }
//   if (opt.max_attempts <= 0 || opt.max_solutions <= 0)
//   {
//     LOGE("[solve_executor][hybrid_ik] Invalid IK options: max_attempts={}, max_solutions={}", opt.max_attempts,
//          opt.max_solutions);
//     return false;
//   }

//   const auto* jmg = robot_model_->getJointModelGroup(group_name_);
//   if (!jmg)
//   {
//     LOGE("[solve_executor][hybrid_ik] JointModelGroup not found: {}", group_name_);
//     return false;
//   }

//   // 随机源：默认使用 random_device；想复现就改成固定 seed，例如 std::mt19937 rng(42);
//   std::mt19937 rng(std::random_device{}());
//   std::normal_distribution<double> noise(0.0, opt.noise_sigma);

//   // 基准关节，做有限制校验
//   std::vector<double> q_base;
//   seed_state.copyJointGroupPositions(jmg, q_base);
//   if (!isFiniteVec(q_base))
//   {
//     LOGE("[solve_executor][hybrid_ik] Seed joint vector has non-finite values");
//     return false;
//   }

//   for (int attempt = 0; attempt < opt.max_attempts; ++attempt)
//   {
//     // 1) 从 seed_state 复制一份出来做扰动（不改原 seed_state）
//     moveit::core::RobotState perturbed(seed_state);

//     // 2) 在基准关节附近加扰动
//     std::vector<double> q = q_base;
//     for (size_t j = 0; j < q.size(); ++j)
//     {
//       q[j] += noise(rng);
//     }

//     // 3) 写回并限制到关节范围
//     perturbed.setJointGroupPositions(jmg, q);
//     perturbed.enforceBounds(jmg);
//     perturbed.update();

//     // 4) 用 perturbed 作为 seed 求 IK
//     if (!perturbed.setFromIK(jmg, target_pose, ee_link_, opt.timeout))
//     {
//       continue;
//     }

//     // 5) 取解
//     std::vector<double> q_sol;
//     perturbed.copyJointGroupPositions(jmg, q_sol);
//     if (!isFiniteVec(q_sol))
//     {
//       continue;
//     }

//     // 6) 再次检查 bounds（保险）
//     if (!perturbed.satisfiesBounds(jmg))
//     {
//       continue;
//     }

//     // 7) 去重：避免同一个解重复收集（可选但强烈建议）
//     bool dup = false;
//     for (const auto& exist : solutions)
//     {
//       if (nearSame(q_sol, exist, opt.dedup_eps))
//       {
//         dup = true;
//         break;
//       }
//     }
//     if (dup)
//       continue;

//     solutions.push_back(std::move(q_sol));

//     if ((int)solutions.size() >= opt.max_solutions)
//     {
//       break;
//     }
//   }

//   if (solutions.empty())
//   {
//     LOGE("[solve_executor][hybrid_ik] IK solve failed: no valid solutions");
//     return false;
//   }
//   return true;
// }

// }  // namespace calculator


// // /home/zc/Desktop/rm_engineer/src/capabilities/arm/calculator/src/hybrid_ik.cpp
#include "calculator/hybrid_ik.hpp"

#include <cmath>
#include <algorithm>
#include "log_tools/log.hpp"

namespace calculator
{

// 检查是否包含非有限值（NaN 或 Inf）
static bool isFiniteVec(const std::vector<double>& q)
{
  for (double v : q)
  {
    if (!std::isfinite(v))
      return false;  
  }
  return true;
}

HybridIK::HybridIK(const moveit::core::RobotModelConstPtr& model, const std::string& group_name,
                   const std::string& ee_link)
  : robot_model_(model), group_name_(group_name), ee_link_(ee_link)
{
}

bool HybridIK::solveAll(const moveit::core::RobotState& seed_state, const Eigen::Isometry3d& target_pose,
                        const IKOptions& opt, std::vector<std::vector<double>>& solutions) const
{
  solutions.clear();

  if (!robot_model_)
  {
    LOGE("[solve_executor][hybrid_ik] Robot model is null");
    return false;
  }
  if (group_name_.empty() || ee_link_.empty())
  {
    LOGE("[solve_executor][hybrid_ik] Planning group or End-effector link is empty");
    return false;
  }

  const auto* jmg = robot_model_->getJointModelGroup(group_name_);
  if (!jmg)
  {
    LOGE("[solve_executor][hybrid_ik] JointModelGroup not found: {}", group_name_);
    return false;
  }

  // 1) 直接使用传入的 seed_state 进行逆解，去除所有的随机扰动
  // 因为规划器外层已经使用了 prev_solutions 作为种子，数值 IK 自然会收敛到连续的最优解
  moveit::core::RobotState ik_state(seed_state);

  // 2) 调用 MoveIt 的 setFromIK 进行一次确定性求解
  if (ik_state.setFromIK(jmg, target_pose, ee_link_, opt.timeout))
  {
    std::vector<double> q_sol;
    ik_state.copyJointGroupPositions(jmg, q_sol);

    // 3) 校验解的有效性以及是否越界
    if (isFiniteVec(q_sol) && ik_state.satisfiesBounds(jmg))
    {
      solutions.push_back(std::move(q_sol));
    }
  }

  // 如果无法基于当前的平滑种子求出解，直接返回失败让外层剪枝，减小无意义的算力浪费
  if (solutions.empty())
  {
    // LOGT("[solve_executor][hybrid_ik] IK solve failed for current seed (No perturbation).");
    return false;
  }
  
  return true;
}

}  // namespace calculator
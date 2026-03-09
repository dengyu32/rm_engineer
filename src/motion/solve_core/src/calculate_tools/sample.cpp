#include "solve_core/calculate_tools/sample.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include <Eigen/Geometry>
#include <moveit/robot_state/robot_state.h>

#include "log_utils/log.hpp"
#include "solve_core/calculate_tools/hybrid_ik.hpp"

namespace solve_core {
namespace {

// _正钳位，确保得到的值始终为正数
inline double clamp_positive(double v, double fallback) {
  return (v > 0.0) ? v : fallback;
}

// _确保采样数量至少为1，避免除零等问题
inline int clamp_count(int n) { return (n > 0) ? n : 1; }

// _数据清洗，确保使用模长为1的单位四元数来表示姿态
Eigen::Quaterniond pose_to_quaternion(const Pose &pose) {
  Eigen::Quaterniond q(pose.qw, pose.qx, pose.qy, pose.qz);
  if (q.norm() < 1e-12) {
    return Eigen::Quaterniond::Identity();
  }
  q.normalize();
  return q;
}

// _构造轴采样后的新目标位姿
Pose quaternion_to_pose_like(const Pose &base_pose, const Eigen::Quaterniond &q) {
  Pose out = base_pose;
  out.qx = q.x();
  out.qy = q.y();
  out.qz = q.z();
  out.qw = q.w();
  return out;
}

// _将轴采样的偏移量归一化到 [0, 1] 范围，并平方以得到代价，确保在评估函数中不同轴的偏移可以直接加权求和
double normalized_axis_offset_cost(double offset, double range) {
  const double safe_range = clamp_positive(range, 1e-6);
  const double r = offset / safe_range;
  return r * r;
}

// _Pose转为旋转矩阵  --单位四元数拼接一个平移向量，得到一个齐次变换矩阵
Eigen::Isometry3d pose_to_isometry(const Pose &pose) {
  Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();    //初始化为单位矩阵
  iso.translation() << pose.x, pose.y, pose.z;
  Eigen::Quaterniond q = pose_to_quaternion(pose);
  iso.linear() = q.toRotationMatrix();
  return iso;
}

} // namespace

std::vector<PoseSampleCandidate>
generate_roll_samples(const Pose &base_pose, const LimitPlannerOptions &opt) {
  std::vector<PoseSampleCandidate> out;

  // 无需采样
  if (!opt.enable_target_pose_sampling ||
      opt.sampling_mode != SamplingMode::ROLL_SAMPLE) {
    PoseSampleCandidate c;
    c.pose = base_pose;
    c.orientation_cost = 0.0;
    out.push_back(std::move(c));
    return out;
  }

  const int roll_n = clamp_count(opt.roll_samples);
  out.reserve(static_cast<std::size_t>(roll_n));

  const double roll_min = -std::abs(opt.roll_range_rad);
  const double roll_max = std::abs(opt.roll_range_rad);

  const double roll_step =
      (roll_n > 1) ? (roll_max - roll_min) / static_cast<double>(roll_n - 1) : 0.0;

  const Eigen::Quaterniond q_base = pose_to_quaternion(base_pose);
  const Eigen::Vector3d zyx_base = q_base.toRotationMatrix().eulerAngles(2, 1, 0);
  const double base_yaw = zyx_base[0];
  const double base_pitch = zyx_base[1];
  const double base_roll = zyx_base[2];

  for (int i = 0; i < roll_n; ++i) {
    const double roll_offset = roll_min + static_cast<double>(i) * roll_step;
    const double roll = base_roll + roll_offset;

    // 仅采样 roll，保持基准 yaw/pitch 不变
    Eigen::Quaterniond q_sample =
        Eigen::AngleAxisd(base_yaw, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(base_pitch, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
    q_sample.normalize();

    PoseSampleCandidate c;
    c.pose = quaternion_to_pose_like(base_pose, q_sample);
    c.roll_offset_rad = roll_offset;
    c.orientation_cost = normalized_axis_offset_cost(roll_offset, opt.roll_range_rad);
    out.push_back(std::move(c));
  }

  return out;
}


//计算欧几里得距离，可复用cost_func中的函数
double joint_distance_l2(const std::vector<double> &a, const std::vector<double> &b) {
  if (a.size() != b.size() || a.empty()) {
    return std::numeric_limits<double>::infinity();
  }
  double sum = 0.0;
  for (std::size_t i = 0; i < a.size(); ++i) {
    const double d = a[i] - b[i];
    sum += d * d;
  }
  return std::sqrt(sum);
}

//只针对单个解进行评估，计算总代价
double evaluate_candidate_cost(PoseSampleCandidate &candidate,
                               const std::vector<double> &current_joints,
                               const LimitPlannerOptions &opt) {
  if (!candidate.ik_valid || candidate.ik_solution.empty()) {
    candidate.ik_cost = std::numeric_limits<double>::infinity();
    candidate.total_cost = std::numeric_limits<double>::infinity();
    return candidate.total_cost;
  }

  const double motion = joint_distance_l2(current_joints, candidate.ik_solution);
  candidate.ik_cost = motion;
  candidate.total_cost = opt.orientation_weight * candidate.orientation_cost +
                         opt.ik_distance_weight * candidate.ik_cost +
                         opt.joint_motion_weight * motion;
  return candidate.total_cost;
}

//ik解算，在每次采样中找到最优解，比较不同采样对应的解的代价
void evaluate_candidates_with_ik(std::vector<PoseSampleCandidate> &candidates,
                                 const moveit::core::RobotModelConstPtr &robot_model,
                                 const std::string &group_name,
                                 const std::string &ee_link,
                                 const moveit::core::RobotState &seed_state,
                                 const std::vector<double> &current_joints,
                                 const IKOptions &ik_opt,
                                 const LimitPlannerOptions &opt) {
  if (candidates.empty()) {
    LOGE("[solve_core][sample] Empty candidates input");
    return;
  }
  if (!robot_model) {
    LOGE("[solve_core][sample] Robot model is null");
    return;
  }
  if (group_name.empty()) {
    LOGE("[solve_core][sample] Planning group is empty");
    return;
  }
  if (ee_link.empty()) {
    LOGE("[solve_core][sample] End-effector link is empty");
    return;
  }
  if (current_joints.empty()) {
    LOGE("[solve_core][sample] Current joints are empty");
    return;
  }

  HybridIK hybrid_ik(robot_model, group_name, ee_link);

  int valid_count = 0;
  for (auto &c : candidates) {
    c.ik_solution.clear();
    c.ik_valid = false;
    c.ik_cost = std::numeric_limits<double>::infinity();    //正无穷大的浮点值
    c.total_cost = std::numeric_limits<double>::infinity();

    std::vector<std::vector<double>> all_solutions;
    if (!hybrid_ik.solveAll(seed_state, pose_to_isometry(c.pose), ik_opt, all_solutions)) {
      continue;
    }

    if (all_solutions.empty()) {
      continue;
    }

    auto best_it = all_solutions.begin();
    double best_dist = joint_distance_l2(current_joints, *best_it);
    for (auto it = std::next(all_solutions.begin()); it != all_solutions.end(); ++it) {
      const double dist = joint_distance_l2(current_joints, *it);
      if (dist < best_dist) {
        best_dist = dist;
        best_it = it;
      }
    }

    c.ik_solution = *best_it;
    c.ik_valid = true;
    ++valid_count;
    evaluate_candidate_cost(c, current_joints, opt);
  }

  if (valid_count == 0) {
    LOGE("[solve_core][sample] IK evaluation finished with zero valid candidates");
  }

  // IK 评估后按总代价筛选 Top-K（<=0 表示不过滤）
  select_best_candidates(candidates, opt.top_k_after_ik);
}

void select_best_candidates(std::vector<PoseSampleCandidate> &candidates, int top_k) {
  if (candidates.empty()) {
    return;
  }

  std::stable_sort(candidates.begin(), candidates.end(),
                   [](const PoseSampleCandidate &a, const PoseSampleCandidate &b) {
                     const bool a_ok = a.ik_valid && std::isfinite(a.total_cost);
                     const bool b_ok = b.ik_valid && std::isfinite(b.total_cost);
                     if (a_ok != b_ok) {
                       return a_ok;
                     }
                     return a.total_cost < b.total_cost;
                   });

  const int keep = (top_k > 0) ? top_k : static_cast<int>(candidates.size());
  if (static_cast<int>(candidates.size()) > keep) {
    candidates.resize(static_cast<std::size_t>(keep));
  }
}

} // namespace solve_core

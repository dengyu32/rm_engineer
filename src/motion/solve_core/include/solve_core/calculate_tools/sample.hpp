#pragma once

#include <limits>
#include <string>
#include <vector>

#include <moveit/robot_model/robot_model.h>

#include "solve_core/solve_core.hpp"
#include "solve_core/calculate_tools/hybrid_ik.hpp"
namespace solve_core {

/*
  姿态采样模式：
  NORMAL: 不采样，直接用输入姿态
  ROLL_SAMPLE: 仅对局部 X 轴(roll)采样，pitch/yaw 保持不变
*/
enum class SamplingMode {
  NORMAL = 0,
  ROLL_SAMPLE = 1,
};

struct LimitPlannerOptions {
  SamplingMode sampling_mode{SamplingMode::NORMAL};

  // 是否在目标姿态层做采样
  bool enable_target_pose_sampling{false};

  // 采样配置：仅对 roll 轴采样
  int roll_samples{5};  //roll轴采样数
  double roll_range_rad{0.35};  //roll轴采样范围（±rad）

  // 候选筛选配置
  int top_k_after_ik{3};              // IK 评估后保留的候选解数量，<=0 则不过滤
  double orientation_weight{1.0};     //目标位姿权重
  double ik_distance_weight{1.0};     //关节空间距离权重
  double joint_motion_weight{1.0};    //关节运动量权重
};

struct PoseSampleCandidate {
  Pose pose{};
  double roll_offset_rad{0.0};  //相对于输入姿态的 roll 偏移量（rad），用于评估和调试

  double orientation_cost{0.0};
  double ik_cost{std::numeric_limits<double>::infinity()};
  double total_cost{std::numeric_limits<double>::infinity()};

  bool ik_valid{false};
  std::vector<double> ik_solution;
};

std::vector<PoseSampleCandidate>
generate_roll_samples(const Pose &base_pose, const LimitPlannerOptions &opt);

double joint_distance_l2(const std::vector<double> &a, const std::vector<double> &b);

double evaluate_candidate_cost(PoseSampleCandidate &candidate,
                               const std::vector<double> &current_joints,
                               const LimitPlannerOptions &opt);

void evaluate_candidates_with_ik(std::vector<PoseSampleCandidate> &candidates,
                                 const moveit::core::RobotModelConstPtr &robot_model,
                                 const std::string &group_name,
                                 const std::string &ee_link,
                                 const moveit::core::RobotState &seed_state,
                                 const std::vector<double> &current_joints,
                                 const IKOptions &ik_opt,
                                 const LimitPlannerOptions &opt);

void select_best_candidates(std::vector<PoseSampleCandidate> &candidates, int top_k);
}

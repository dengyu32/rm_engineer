#include "solve_core/solve_core.hpp"

#include <algorithm>
#include <cmath>

#include "log_utils/log.hpp"
#include "solve_core/planner/limit_planner.hpp"

#include <Eigen/Geometry>

#include <moveit/collision_detection/collision_common.h>
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

namespace solve_core {
namespace {

Pose isometry_to_pose(const Eigen::Isometry3d &iso) {
  Pose p;
  p.x = iso.translation().x();
  p.y = iso.translation().y();
  p.z = iso.translation().z();
  const Eigen::Quaterniond q(iso.rotation());
  p.qx = q.x();
  p.qy = q.y();
  p.qz = q.z();
  p.qw = q.w();
  return p;
}

} // namespace

LimitPlanner::LimitPlanner(std::shared_ptr<MoveItAdapter> adapter) : adapter_(adapter) {}

std::optional<Trajectory>
LimitPlanner::plan(const moveit::core::JointModelGroup *jmg_,
                   const std::string ee_link_,
                   moveit::core::RobotState &start_state,
                   const Eigen::Isometry3d &target_pose,
                   const LimitPlannerOptions &opt, std::string &err,
                   const PlannerConfigs &planner_configs,
                   std::vector<std::vector<double>> *joint_path_out
                   ) {
  if (!adapter_) {
    err = "MoveIt adapter not set";
    LOGE("[solve_core][limit_planner] {}", err);
    return std::nullopt;
  }
  if (!jmg_) {
    err = "JointModelGroup is null";
    LOGE("[solve_core][limit_planner] {}", err);
    return std::nullopt;
  }
  if (ee_link_.empty()) {
    err = "End-effector link is empty";
    LOGE("[solve_core][limit_planner] {}", err);
    return std::nullopt;
  }

  std::vector<double> q_target;

  // 采样模式：先在姿态层做 roll 采样，再基于代价选择最优 IK 解
  if (opt.enable_target_pose_sampling && opt.sampling_mode == SamplingMode::ROLL_SAMPLE) {
    const auto model = adapter_->robot_model();
    if (!model) {
      err = "RobotModel is null";
      LOGE("[solve_core][limit_planner] {}", err);
      return std::nullopt;
    }

    std::vector<double> current_joints;
    start_state.copyJointGroupPositions(jmg_, current_joints);

    IKOptions ik_opt;
    auto candidates = generate_roll_samples(isometry_to_pose(target_pose), opt);
    evaluate_candidates_with_ik(candidates,
                                model,
                                jmg_->getName(),
                                ee_link_,
                                start_state,
                                current_joints,
                                ik_opt,
                                opt);

    const auto best_it = std::find_if(candidates.begin(), candidates.end(),
                                      [](const PoseSampleCandidate &c) {
                                        return c.ik_valid && !c.ik_solution.empty();
                                      });
    if (best_it == candidates.end()) {
      err = "No valid IK solution from roll samples";
      LOGE("[solve_core][limit_planner] {}", err);
      return std::nullopt;
    }

    q_target = best_it->ik_solution;
  } else {
    moveit::core::RobotState ik_state(start_state);
    bool ik_ok = ik_state.setFromIK(jmg_, target_pose, ee_link_, 2.0); // IK求解，得到一个目标关节配置
    if (!ik_ok) {
      const Eigen::Isometry3d current_fk =
          start_state.getGlobalLinkTransform(ee_link_);
      ik_ok = ik_state.setFromIK(
          jmg_, current_fk, ee_link_,
          2.0); // 如果目标位姿不可达，尝试用当前位姿作为目标再次求解，确保至少有一个解
      if (!ik_ok) {
        LOGE("[solve_core][limit_planner] IK failed");
        err = "IK failed";
        return std::nullopt;
      }
    }
    ik_state.copyJointGroupPositions(jmg_, q_target);
  }

  if (q_target.size() != jmg_->getVariableCount()) {
    LOGE("[solve_core][limit_planner] IK variables size mismatch");
    err = "IK variables size mismatch";
    return std::nullopt;
  }

  if (joint_path_out) {
    joint_path_out->clear();
    joint_path_out->push_back(q_target);
  }

  const auto joint_names = jmg_->getVariableNames();
  // 采样式规划
  auto plan_res = adapter_->plan_to_joint_target(joint_names, q_target, planner_configs);
  if (!plan_res) {
    LOGE("[solve_core] plan_to_joint_target failed");
    err = "plan_to_joint_target failed";
    return std::nullopt;
  }

  return plan_res;
}
} // namespace solve_core

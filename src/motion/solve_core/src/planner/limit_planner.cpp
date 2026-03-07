#include "solve_core/solve_core.hpp"

#include <cmath>

#include "log_utils/log.hpp"
#include "solve_core/error_code.hpp"
#include "solve_core/planner/limit_planner.hpp"

#include <Eigen/Geometry>

#include <moveit/collision_detection/collision_common.h>
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

namespace solve_core {

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
  moveit::core::RobotState ik_state(start_state);
  bool ik_ok = ik_state.setFromIK(jmg_, target_pose, ee_link_,2.0); // IK求解，得到一个目标关节配置
  if (!ik_ok) {
    const Eigen::Isometry3d current_fk =
        start_state.getGlobalLinkTransform(ee_link_);
    ik_ok = ik_state.setFromIK(
        jmg_, current_fk, ee_link_,
        2.0); // 如果目标位姿不可达，尝试用当前位姿作为目标再次求解，确保至少有一个解
    if (!ik_ok) {
      LOGE("[solve_core] IK failed");
      err = "IK failed";
      return std::nullopt;
    }
  }
  std::vector<double> q_target;
  ik_state.copyJointGroupPositions(jmg_, q_target);
  if (q_target.size() != jmg_->getVariableCount()) {
    LOGE("[solve_core] IK variables size mismatch");
    err = "IK variables size mismatch";
    return std::nullopt;
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

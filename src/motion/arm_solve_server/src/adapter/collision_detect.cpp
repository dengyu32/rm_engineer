#pragma once

/*
    自碰撞检测，基于moveit的轻量级代码实现
*/

#include "executor.hpp"

namespace arm_solve {
namespace solve_executor {
bool SolveExecutor::check_self_collision(const moveit::core::RobotState &state,
                                         const std::string &group_name) {
  if (!psm_) {
    RCLCPP_ERROR(get_logger(),"[scope=arm_solve_server][status=error] Planning scene not ready");
    return false;
  }
  planning_scene_monitor::LockedPlanningSceneRO scene(psm_);
  if (!scene) {
    RCLCPP_ERROR(get_logger(),"[scope=arm_solve_server][status=error] Planning scene unavailable");
    return false;
  }
  collision_detection::CollisionRequest req;
  collision_detection::CollisionResult res;
  req.group_name = group_name;
  req.contacts = false;
  req.max_contacts = 0;
  scene->checkSelfCollision(req, res, state);
  if (res.collision) {
    RCLCPP_ERROR(get_logger(),"[scope=arm_solve_server][status=error] Self collision detected");
    return false;
  }
  return true;
}
} // namespace solve_executor
} // namespace arm_solve
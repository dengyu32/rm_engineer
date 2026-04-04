#include "teleop_node/collision_checker.hpp"

#include <moveit/collision_detection/collision_common.h>
#include <moveit/robot_state/robot_state.h>

namespace engineer_teleop {

// ============================================================================
//  ctor
// ============================================================================

CollisionChecker::CollisionChecker( 
    const rclcpp::Node::SharedPtr& node,
    const std::string& group_name,  
    const std::vector<std::string>& joint_names)
  : group_name_(group_name), joint_names_(joint_names) {

  robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(
      node, "robot_description", false);

  robot_model_ = robot_model_loader_->getModel();

  if (!robot_model_) {
    RCLCPP_ERROR(node->get_logger(), "Failed to load robot model from 'robot_description'");
    throw std::runtime_error("Robot Model Init Failed!!!");
  }

  planning_scene_ = std::make_shared<planning_scene::PlanningScene>(robot_model_);

  if (!robot_model_->hasJointModelGroup(group_name_)) {
    RCLCPP_ERROR(node->get_logger(), "Joint group '%s' not found in robot model", group_name_.c_str());
    throw std::runtime_error("Invalid Param : group_name ");
  }
}

// ============================================================================
//  Self Conllison Check Func
// ============================================================================

bool CollisionChecker::isSelfCollisionFree(
    const std::vector<double> &positions) const {

  if (positions.size() != joint_names_.size()) {
    return false;
  }

  moveit::core::RobotState candidate_state(robot_model_);
  candidate_state.setToDefaultValues();  // 按照 urdf srdf 默认值设置关节状态
  for (std::size_t i = 0; i < joint_names_.size(); ++i) {
    candidate_state.setJointPositions(joint_names_[i], &positions[i]);
  }
  candidate_state.update();

  collision_detection::CollisionRequest req;
  collision_detection::CollisionResult res;
  req.group_name = group_name_;
  req.contacts = false;
  req.max_contacts = 0;

  planning_scene_->checkSelfCollision(req, res, candidate_state);
  return !res.collision;
}

} // namespace engineer_teleop

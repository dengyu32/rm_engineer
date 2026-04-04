#pragma once

// Rely

//< ROS 2
#include <rclcpp/rclcpp.hpp>

//< MoveIt
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

namespace engineer_teleop {

// ============================================================================
//  CollisionChecker
// ----------------------------------------------------------------------------
//   - 使用 MoveIt! 的 PlanningScene 进行自碰撞检测
//   - 严格不依赖 ROS Topic 
//   - 提供函数接口 isSelfCollisionFree
// ============================================================================

class CollisionChecker {
public:
  CollisionChecker(
    const rclcpp::Node::SharedPtr& node,
    const std::string& group_name,
    const std::vector<std::string>& joint_names);

  //< Self Conllison Check Func
  bool isSelfCollisionFree(
    const std::vector<double> &positions) const;

private:
  //< Robotic Arm Params
  std::string group_name_;
  std::vector<std::string> joint_names_;

  //< MoveIt! Components 
  robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
  moveit::core::RobotModelPtr robot_model_;
  planning_scene::PlanningScenePtr planning_scene_;
};

} // namespace engineer_teleop

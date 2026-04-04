#pragma once

#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <moveit/collision_detection/collision_common.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

namespace collision
{
struct SelfCollisionRequest
{
  int64_t max_contacts{ 8 };
  int64_t max_contacts_per_pair{ 1 };
};

struct SelfCollisionCheckResult
{
  bool collision_free{ true };
  std::vector<std::pair<std::string, std::string>> contacts;
  std::string message;
};

class SelfCollisionDetector
{
public:
  SelfCollisionDetector(std::string group_name, SelfCollisionRequest request, moveit::core::RobotModelConstPtr robot_model,
                        planning_scene::PlanningScenePtr planning_scene);

  bool check(const std::vector<double>& joint_positions, SelfCollisionCheckResult& result) const;

private:
  bool isValid(std::string& err) const;

  std::string group_name_;
  SelfCollisionRequest request_{};
  collision_detection::CollisionRequest req_{};

  mutable std::mutex init_mutex_;
  moveit::core::RobotModelConstPtr robot_model_;
  planning_scene::PlanningScenePtr planning_scene_;
};

}  // namespace collision

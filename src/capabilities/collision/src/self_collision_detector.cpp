#include "collision/self_collision_detector.hpp"

#include <algorithm>
#include <sstream>
#include <utility>

#include "log_tools/log.hpp"

namespace collision
{
namespace
{
const moveit::core::JointModelGroup* get_jmg(const moveit::core::RobotModelConstPtr& robot_model,
                                             const std::string& group_name)
{
  if (!robot_model)
  {
    return nullptr;
  }
  return robot_model->getJointModelGroup(group_name);
}
}  // namespace

SelfCollisionDetector::SelfCollisionDetector(std::string group_name, SelfCollisionRequest request,
                                             moveit::core::RobotModelConstPtr robot_model,
                                             planning_scene::PlanningScenePtr planning_scene)
  : group_name_(std::move(group_name))
  , request_(std::move(request))
  , robot_model_(std::move(robot_model))
  , planning_scene_(std::move(planning_scene))
{
  req_.group_name = group_name_;
  req_.contacts = true;
  req_.max_contacts = static_cast<std::size_t>(std::max<int64_t>(1, request_.max_contacts));
  req_.max_contacts_per_pair = static_cast<std::size_t>(std::max<int64_t>(1, request_.max_contacts_per_pair));

  LOGI("[collision] SelfCollisionDetector initialized: group_name={}, max_contacts={}, max_contacts_per_pair={}",
       group_name_, request_.max_contacts, request_.max_contacts_per_pair);
}

bool SelfCollisionDetector::check(const std::vector<double>& joint_positions, SelfCollisionCheckResult& result) const
{
  result = SelfCollisionCheckResult{};

  std::string err;
  if (!isValid(err))
  {
    result.collision_free = false;
    result.message = err;
    LOGE("[collision] {}", err);
    return false;
  }

  const auto* joint_model_group = get_jmg(robot_model_, group_name_);
  const auto variable_count = joint_model_group->getVariableCount();
  if (joint_positions.size() != variable_count)
  {
    result.collision_free = false;
    result.message = "Joint positions size mismatch, expected " + std::to_string(variable_count) + ", got " +
                     std::to_string(joint_positions.size());
    LOGE("[collision] {}", result.message);
    return false;
  }

  moveit::core::RobotState state(robot_model_);
  state.setToDefaultValues();
  state.setJointGroupPositions(joint_model_group, joint_positions);
  state.update();

  collision_detection::CollisionResult res;
  {
    std::scoped_lock<std::mutex> lock(init_mutex_);
    planning_scene_->checkSelfCollision(req_, res, state);
  }

  if (!res.collision)
  {
    result.collision_free = true;
    return true;
  }

  result.collision_free = false;
  for (const auto& entry : res.contacts)
  {
    result.contacts.emplace_back(entry.first.first, entry.first.second);
  }

  std::ostringstream oss;
  oss << "Self collision detected in group '" << group_name_ << "'";
  if (!result.contacts.empty())
  {
    oss << ": ";
    for (std::size_t i = 0; i < result.contacts.size(); ++i)
    {
      if (i > 0)
      {
        oss << ", ";
      }
      oss << result.contacts[i].first << " <-> " << result.contacts[i].second;
    }
  }
  result.message = oss.str();
  return false;
}

bool SelfCollisionDetector::isValid(std::string& err) const
{
  if (group_name_.empty())
  {
    err = "Self collision group_name is empty";
    return false;
  }
  if (!robot_model_)
  {
    err = "Robot model is unavailable";
    return false;
  }
  if (!planning_scene_)
  {
    err = "Planning scene is unavailable";
    return false;
  }
  if (request_.max_contacts <= 0)
  {
    err = "max_contacts must be positive";
    return false;
  }
  if (request_.max_contacts_per_pair <= 0)
  {
    err = "max_contacts_per_pair must be positive";
    return false;
  }
  if (!get_jmg(robot_model_, group_name_))
  {
    err = "JointModelGroup not found: " + group_name_;
    return false;
  }
  return true;
}

}  // namespace collision

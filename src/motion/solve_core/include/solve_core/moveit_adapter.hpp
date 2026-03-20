#pragma once
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "solve_core/adapter.hpp"

namespace moveit::core {
class RobotModel;
class JointModelGroup;
class RobotState;
} // namespace moveit::core

namespace solve_core {
struct PlannerConfigs {
  double goal_position_tolerance{1e-3};
  double goal_orientation_tolerance{1e-3};
  double planning_time{2.0};
  int num_planning_attempts{5};
  double max_velocity_scaling{1.0};
  double max_acc_scaling{1.0};
};

class MoveItAdapter {
public:
  virtual ~MoveItAdapter() = default;

  virtual std::shared_ptr<const moveit::core::RobotModel>
  robot_model() const = 0;
  virtual const moveit::core::JointModelGroup *
  joint_model_group(const std::string &group_name) const = 0;
  virtual std::string group_name() const = 0;
  virtual std::string planning_frame() const = 0;
  virtual std::string end_effector_link() const = 0;
  virtual void set_start_state(const moveit::core::RobotState &state) = 0;

  virtual std::optional<Trajectory>
  plan_to_joint_target(const std::vector<std::string> &joint_names,
                       const std::vector<double> &joint_values,
                       const PlannerConfigs &options) = 0;

  virtual bool check_self_collision(const moveit::core::RobotState &state,
                                    const std::string &group_name,
                                    std::string &err) const = 0;

  // Time-parameterize an existing joint trajectory without changing path.
  virtual bool
  time_parameterize_trajectory(const moveit::core::RobotState &start_state,
                               const std::string &group_name, Trajectory &traj,
                               double velocity_scaling, double accel_scaling,
                               std::string &err) const = 0;
};
} // namespace solve_core
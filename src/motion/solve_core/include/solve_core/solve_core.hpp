#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "error_code_utils/error_bus.hpp"

#include <rclcpp/rclcpp.hpp>

namespace moveit::core {
class RobotModel;
class JointModelGroup;
class RobotState;
} // namespace moveit::core

namespace solve_core {

// ============================================================================
//  Basic data types (ROS-agnostic)
// ============================================================================
struct Pose {
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double qx{0.0};
  double qy{0.0};
  double qz{0.0};
  double qw{1.0};
};

struct JointState {
  std::vector<std::string> names;
  std::vector<double> positions;
};

struct PlannerOptions {
  double goal_position_tolerance{1e-3};
  double goal_orientation_tolerance{1e-3};
  double planning_time{2.0};
  int num_planning_attempts{5};
  double max_velocity_scaling{0.6};
  double max_acc_scaling{0.6};
};

enum class PlanOption {
  NORMAL = 0,
  CARTESIAN = 1,
  JOINTS = 2,
};

struct SolveRequest {
  PlanOption option{PlanOption::NORMAL};
  Pose target_pose{};
  std::vector<double> target_joints;
  JointState current_joints;
  PlannerOptions planner;
  std::string group_name;
  std::string ee_link;
};

struct TrajectoryPoint {
  std::vector<double> positions;
  std::vector<double> velocities;
  double time_from_start{0.0};
};

struct Trajectory {
  std::vector<std::string> joint_names;
  std::vector<TrajectoryPoint> points;
};

struct SolveResponse {
  Trajectory trajectory;
  std::vector<std::vector<double>> joint_path;
};

// ============================================================================
//  MoveIt adapter interface (implemented by ROS shell)
// ============================================================================
class MoveItAdapter {
public:
  virtual ~MoveItAdapter() = default;

  virtual std::shared_ptr<const moveit::core::RobotModel> robot_model() const = 0;
  virtual const moveit::core::JointModelGroup *joint_model_group(
      const std::string &group_name) const = 0;
  virtual std::string group_name() const = 0;
  virtual std::string planning_frame() const = 0;
  virtual std::string end_effector_link() const = 0;
  virtual void set_start_state(const moveit::core::RobotState &state) = 0;

  virtual std::optional<Trajectory> plan_to_joint_target(
      const std::vector<std::string> &joint_names,
      const std::vector<double> &joint_values,
      const PlannerOptions &options) = 0;

  virtual bool check_self_collision(
      const moveit::core::RobotState &state,
      const std::string &group_name,
      std::string &err) const = 0;
};

// ============================================================================
//  SolveCore
// ============================================================================
class SolveCore {
public:
  explicit SolveCore(std::shared_ptr<MoveItAdapter> adapter);

  std::optional<SolveResponse> plan(const SolveRequest &req);
  void set_error_bus(const std::shared_ptr<error_code_utils::ErrorBus> &bus);

private:
  std::shared_ptr<MoveItAdapter> adapter_;
  std::shared_ptr<error_code_utils::ErrorBus> error_bus_;
  rclcpp::Logger logger_{rclcpp::get_logger("solve_core")};

  std::optional<SolveResponse> plan_normal(const SolveRequest &req);
  std::optional<SolveResponse> plan_cartesian(const SolveRequest &req);
  std::optional<SolveResponse> plan_joints(const SolveRequest &req);

  void publish_error(const error_code_utils::Error &err) const;
};

} // namespace solve_core

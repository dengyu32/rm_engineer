
#pragma once

#include <sstream>
#include <string>
#include <stdexcept>
#include <array>
#include <vector>

#include <cmath>
#include <unordered_map>
#include <Eigen/Geometry>

#include <moveit/collision_detection/collision_common.h>
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>


namespace solve_core {

struct SolveCoreConfig {
  // 跨 planner 共享的默认规划参数
  double goal_position_tolerance{1e-3};
  double goal_orientation_tolerance{1e-3};
  double planning_time{2.0};
  int num_planning_attempts{5};
  double max_velocity_scaling{1.0};
  double max_acc_scaling{1.0};

  void validate() const;
  std::string summary() const;
};

struct JointsPlannerConfigs {
  double max_step_rad{0.05};

  void validate() const;
  std::string summary() const;
};

inline void SolveCoreConfig::validate() const {
  if (goal_position_tolerance < 0.0 || goal_position_tolerance > 1.0) {
    throw std::runtime_error("SolveCoreConfig: goal_position_tolerance must be in [0, 1]");
  }
  if (goal_orientation_tolerance < 0.0 || goal_orientation_tolerance > 1.0) {
    throw std::runtime_error("SolveCoreConfig: goal_orientation_tolerance must be in [0, 1]");
  }
  if (planning_time < 0.0 || planning_time > 30.0) {
    throw std::runtime_error("SolveCoreConfig: planning_time must be in [0, 30]");
  }
  if (num_planning_attempts < 1 || num_planning_attempts > 50) {
    throw std::runtime_error("SolveCoreConfig: num_planning_attempts must be in [1, 50]");
  }
  if (max_velocity_scaling < 0.0 || max_velocity_scaling > 1.0) {
    throw std::runtime_error("SolveCoreConfig: max_velocity_scaling must be in [0, 1]");
  }
  if (max_acc_scaling < 0.0 || max_acc_scaling > 1.0) {
    throw std::runtime_error("SolveCoreConfig: max_acc_scaling must be in [0, 1]");
  }
}

inline std::string SolveCoreConfig::summary() const {
  std::ostringstream oss;
  oss << "=========\n";
  oss << " SolveCore Configuration\n\n";
  oss << " MoveIt Defaults:\n";
  oss << "   - goal_position_tolerance           : " << goal_position_tolerance << "\n";
  oss << "   - goal_orientation_tolerance        : " << goal_orientation_tolerance << "\n";
  oss << "   - planning_time                     : " << planning_time << "\n";
  oss << "   - num_planning_attempts             : " << num_planning_attempts << "\n";
  oss << "   - max_velocity_scaling              : " << max_velocity_scaling << "\n";
  oss << "   - max_acc_scaling                   : " << max_acc_scaling << "\n";
  oss << "=========\n";
  return oss.str();
}

inline void JointsPlannerConfigs::validate() const {
  if (max_step_rad <= 0.0) {
    throw std::runtime_error("JointsPlannerConfigs: max_step_rad must be > 0");
  }
}

inline std::string JointsPlannerConfigs::summary() const {
  std::ostringstream oss;
  oss << " Joints Planner:\n";
  oss << "   - max_step_rad                      : " << max_step_rad << "\n";
  return oss.str();
}



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
  // std::vector<std::vector<double>> joint_path;
};

// 将关节角写进robotstate，必须齐全
inline bool fill_joint_state_require_all(const JointState &js,
                                  const moveit::core::JointModelGroup *jmg,
                                  moveit::core::RobotState &state,
                                  std::string &err) {
  if (!jmg) {
    err = "JointModelGroup null";
    return false;
  }
  const auto &group_joint_names = jmg->getVariableNames();
  if (js.names.empty() || js.positions.empty()) {
    err = "Missing joint state";
    return false;
  }
  if (js.names.size() != js.positions.size()) {
    err = "Joint names/positions size mismatch";
    return false;
  }
  std::unordered_map<std::string, double> pos_map;
  pos_map.reserve(js.names.size());
  for (std::size_t i = 0; i < js.names.size(); ++i) {
    pos_map[js.names[i]] = js.positions[i];
  }
  for (const auto &jn : group_joint_names) {
    auto it = pos_map.find(jn);
    if (it == pos_map.end()) {
      err = "Missing joint state";
      return false;
    }
    state.setVariablePosition(it->first, it->second);
  }
  return true;
}
}

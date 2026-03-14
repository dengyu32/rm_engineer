#pragma once

#include <engineer_interfaces/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>

namespace task_step_library {

struct RuntimeContext {
  bool has_vision_pose{false};
  engineer_interfaces::msg::Pose vision_pose{};

  bool has_vision_vector{false};
  geometry_msgs::msg::Vector3 vision_vector{};

  bool has_selected_slot{false};
  int selected_slot{-1};
};

struct StepResult {
  bool has_vision_pose{false};
  engineer_interfaces::msg::Pose vision_pose{};

  bool has_vision_vector{false};
  geometry_msgs::msg::Vector3 vision_vector{};

  bool has_selected_slot{false};
  int selected_slot{-1};
};

} // namespace task_step_library

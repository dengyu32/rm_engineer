#pragma once

#include <array>
#include <cstdint>

#include <engineer_interfaces/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>

namespace engineer_auto::arm_solve_client {

enum class PlanOption : uint8_t {
  NORMAL = 0,
  CARTESIAN = 1,
  JOINTS = 2,
};

struct ArmMoveSpec {
  engineer_interfaces::msg::Pose pose{};
  std::array<float, 6> joints{{0.f, 0.f, 0.f, 0.f, 0.f, 0.f}};
  geometry_msgs::msg::Vector3 vector{};

  PlanOption plan_option{PlanOption::NORMAL};
};

} // namespace engineer_auto::arm_solve_client

#pragma once

#include <cstdint>

namespace engineer_auto::gripper_control_node {

enum class GripperCommand : uint8_t {
  OPEN = 0,
  CLOSE = 1,
};

} // namespace engineer_auto::gripper_control_node

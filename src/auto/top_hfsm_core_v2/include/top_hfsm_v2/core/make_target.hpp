#pragma once

#include <engineer_interfaces/msg/target.hpp>

namespace top_hfsm_v2 {

inline engineer_interfaces::msg::Target
makeTarget(double x, double y, double z, double qx, double qy, double qz,
           double qw) {
  engineer_interfaces::msg::Target target;
  target.x = x;
  target.y = y;
  target.z = z;
  target.qx = qx;
  target.qy = qy;
  target.qz = qz;
  target.qw = qw;
  return target;
}

} // namespace top_hfsm_v2

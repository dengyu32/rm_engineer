// ============================================================================
//  Target helper
// ----------------------------------------------------------------------------
//  - 构造 engineer_interfaces::msg::Target 的便捷函数
// ============================================================================
#pragma once

#include <engineer_interfaces/msg/target.hpp>

namespace top_hfsm {

inline engineer_interfaces::msg::Target makeTarget(double x, double y,
                                                   double z, double qx,
                                                   double qy, double qz,
                                                   double qw) {
  engineer_interfaces::msg::Target t;
  t.x = x;
  t.y = y;
  t.z = z;

  t.qx = qx;
  t.qy = qy;
  t.qz = qz;
  t.qw = qw;
  return t;
}

} // namespace top_hfsm


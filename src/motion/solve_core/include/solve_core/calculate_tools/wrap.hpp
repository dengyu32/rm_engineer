//ik_continuity_utils.hpp
#pragma once
#include <cmath>
#include <vector>
#include <algorithm>
#include <limits>

/*
  工具函数：用于处理路径点之间的连续性和安全限幅问题
  包括：在线处理，后处理，离线校验过程
- wrapToNearby：将一个角度值 q 包裹到 q_ref 附近
- wrapVectorToNearby：对一个向量的每个元素进行 wrapToNearby
- clampDelta：将 q_new 限幅到 q_ref 的 dq_max_abs 范围内
- unwrapTrajectory：后处理，对一系列路径点进行 wrapVectorToNearby 以保证连续性(我总认为这个函数的名字有歧义)
- validateMaxStep：离线校验，检查路径点之间的变化是否超过 dq_max_abs 的限制
*/
namespace ikc {

inline double wrapToNearby(double q, double q_ref) {
  double d = q - q_ref;
  while (d >  M_PI) { q -= 2.0 * M_PI; d = q - q_ref; }
  while (d < -M_PI) { q += 2.0 * M_PI; d = q - q_ref; }
  return q;
}

inline void wrapVectorToNearby(std::vector<double>& q, const std::vector<double>& q_ref) {
  const size_t n = std::min(q.size(), q_ref.size());  //可能存在维度不匹配的情况
  for (size_t i = 0; i < n; ++i) q[i] = wrapToNearby(q[i], q_ref[i]);
}

inline void clampDelta(std::vector<double>& q_new,
                       const std::vector<double>& q_ref,
                       const std::vector<double>& dq_max_abs) { // dq_max_abs 是每个关节允许的单步最大变化量（绝对值）
 const size_t n = std::min({q_new.size(), q_ref.size(), dq_max_abs.size()});
  for (size_t i = 0; i < n; ++i) {
    const double lim = std::abs(dq_max_abs[i]);
    const double dq = q_new[i] - q_ref[i];
    const double dq_clamped = std::max(-lim, std::min(lim, dq));
    q_new[i] = q_ref[i] + dq_clamped;
  }
}
// 做再检验与等效 
inline bool unwrapTrajectory(std::vector<std::vector<double>>& q_points) {
  if (q_points.empty()) return true;
  for (size_t k = 1; k < q_points.size(); ++k) {
    wrapVectorToNearby(q_points[k], q_points[k - 1]);
  }
  return true;
}

//对规划后的结果再做一次检测 
inline bool validateMaxStep(const std::vector<std::vector<double>>& q_points,
                            const std::vector<double>& dq_max_abs,
                            size_t* bad_k = nullptr,
                            size_t* bad_joint = nullptr) {
  if (q_points.size() < 2) return true;
  for (size_t k = 1; k < q_points.size(); ++k) {
    const size_t n = std::min({dq_max_abs.size(), q_points[k].size(), q_points[k-1].size()});
    for (size_t i = 0; i < n; ++i) {
      const double dq = std::abs(q_points[k][i] - q_points[k - 1][i]);
      if (dq > std::abs(dq_max_abs[i])) {
        if (bad_k) *bad_k = k;
        if (bad_joint) *bad_joint = i;
        return false;
      }
    }
  }
  return true;
}

} // namespace ikc


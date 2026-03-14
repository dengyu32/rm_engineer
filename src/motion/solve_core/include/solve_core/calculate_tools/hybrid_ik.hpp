#pragma once

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <Eigen/Geometry>
#include <string>
#include <vector>
#include <sstream>

#include "log_utils/log.hpp"

/*
  预期做混合IK（如果后面能上IKFast的话）
  混合IK思路：
  1)使用解析IK进行逆解算
  2）添加噪声对得到的解析解

  使用KDL作为解算器：
  1) 从 seed_state 复制一份出来做扰动（不改原 seed_state）
  2) 在基准关节附近加扰动
  3) 写回并限制到关节范围
  4) 用 perturbed 作为 seed 求 IK
  5) 取解
  6) 再次检查 bounds（保险）
*/

namespace solve_core {

struct IKOptions {
  int max_attempts{50};    // 总尝试次数
  int max_solutions{10};   // 最多收集多少个解
  double timeout{0.05};    // setFromIK timeout (seconds)
  double noise_sigma{0.2}; // 高斯噪声标准差
  double dedup_eps{1e-3};  // 去重阈值（L∞）

  void log() const {
    std::ostringstream oss;
    oss << "=========\n";
    oss << " HybridIK Options\n\n";
    oss << "   - max_attempts  : " << max_attempts << "\n";
    oss << "   - max_solutions : " << max_solutions << "\n";
    oss << "   - timeout       : " << timeout << "\n";
    oss << "   - noise_sigma   : " << noise_sigma << "\n";
    oss << "   - dedup_eps     : " << dedup_eps << "\n";
    oss << "=========\n";
    LOGI("{}", oss.str());
  }
};

class HybridIK {
public:
  HybridIK(const moveit::core::RobotModelConstPtr& model,
           const std::string& group_name,
           const std::string& ee_link);

  // 注意：seed_state 作为“基准 seed”，不会被修改
  bool solveAll(const moveit::core::RobotState& seed_state,
                const Eigen::Isometry3d& target_pose,
                const IKOptions& opt,
                std::vector<std::vector<double>>& solutions) const;

private:
  moveit::core::RobotModelConstPtr robot_model_;
  std::string group_name_;
  std::string ee_link_;
};

} // namespace solve_core

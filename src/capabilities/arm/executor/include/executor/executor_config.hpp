#pragma once

#include <rclcpp/rclcpp.hpp>

#include <sstream>
#include <string>

#include "robot_config/robot_config.hpp"

namespace solve_executor {

// ============================================================================
//  SolveExecutorConfig
// ----------------------------------------------------------------------------
//  - SolveExecutor节点参数加载，包括：
//  - MoveItResetConfig
//  - JointResetConfig
//  - SolveExecutorResetConfig
// ============================================================================
struct SolveExecutorConfig : public params_utils::MoveItResetConfig,
                             public params_utils::JointResetConfig,
                             public params_utils::SolveExecutorResetConfig
{
  static SolveExecutorConfig Load(rclcpp::Node& node)
  {
    SolveExecutorConfig cfg;
    params_utils::MoveItResetConfig::Load(node, cfg);
    params_utils::JointResetConfig::Load(node, cfg);
    params_utils::SolveExecutorResetConfig::Load(node, cfg);
    cfg.validate();
    return cfg;
  }

  void validate() const
  {
    params_utils::MoveItResetConfig::validate();
    params_utils::JointResetConfig::validate();
    params_utils::SolveExecutorResetConfig::validate();
  }

  std::string summary() const
  {
    std::ostringstream oss;
    oss << "=========\n";
    oss << " SolveExecutor Configuration\n\n";
    oss << params_utils::MoveItResetConfig::summary();
    oss << params_utils::JointResetConfig::summary();
    oss << params_utils::SolveExecutorResetConfig::summary();
    oss << "=========\n";
    return oss.str();
  }
};

} // namespace solve_executor

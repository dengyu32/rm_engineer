#pragma once

#include <rclcpp/rclcpp.hpp>

#include <cstddef>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "robot_config/robot_config.hpp"

namespace fake_system {

// ============================================================================
//  FakeSystemConfig
// ----------------------------------------------------------------------------
//  - 关节布局 + 话题 + 初始化参数
// ============================================================================

struct FakeSystemConfig : public params_utils::JointResetConfig,
                          public params_utils::IntentResetConfig,
                          public params_utils::GripperResetConfig {
  //  Timing
  int publish_period_ms{33};

  //  Initialization
  std::vector<double> initial_joint_positions{0, 0, 0, 0, 0, 0};
  double initial_gripper_position{0.0};
  std::string slot_cmd_topic{"/slot_cmds"};
  std::string slot_state_topic{"/slot_states"};
  std::vector<bool> initial_slot_status{false, false};

  //  API
  static FakeSystemConfig Load(rclcpp::Node &node) {
    FakeSystemConfig cfg;

    //  Load base layout + topics
    params_utils::JointResetConfig::Load(node, cfg);
    params_utils::IntentResetConfig::Load(node, cfg);
    params_utils::GripperResetConfig::Load(node, cfg);

    using params_utils::detail::declare_get;
    using params_utils::detail::declare_get_checked;

    //  Timing
    declare_get_checked(
        node, "publish_period_ms", cfg.publish_period_ms,
        [](int v) { return v > 0; }, "must be > 0");

    //  Initialization
    cfg.initial_joint_positions.assign(static_cast<std::size_t>(cfg.joint_count), 0.0);

    declare_get(node, "initial_joint_positions", cfg.initial_joint_positions);
    declare_get_checked(
        node, "initial_gripper_position", cfg.initial_gripper_position,
        [](double v) { return v >= 0.0 && v <= 0.03; }, "must be in [0, 0.03]");
    declare_get(node, "slot_cmd_topic", cfg.slot_cmd_topic);
    declare_get(node, "slot_state_topic", cfg.slot_state_topic);
    declare_get(node, "initial_slot_status", cfg.initial_slot_status);
    if (cfg.initial_slot_status.size() != 2U) {
      std::ostringstream oss;
      oss << "FakeSystemConfig: initial_slot_status must have 2 elements, got "
          << cfg.initial_slot_status.size();
      throw std::runtime_error(oss.str());
    }

    //  Finalize
    cfg.validate();
    return cfg;
  }
  void validate() const;
  std::string summary() const;
};

inline void FakeSystemConfig::validate() const {
  params_utils::JointResetConfig::validate();
  params_utils::IntentResetConfig::validate();
  params_utils::GripperResetConfig::validate();

  if (initial_joint_positions.size() != static_cast<std::size_t>(joint_count)) {
    std::ostringstream oss;
    oss << "FakeSystemConfig: initial_joint_positions must have " << joint_count
        << " elements, got "
        << initial_joint_positions.size();
    throw std::runtime_error(oss.str());
  }
}

inline std::string FakeSystemConfig::summary() const {
  std::ostringstream oss;

  oss << "=============================================================================\n";
  oss << " FakeSystemNode Configuration\n";

  oss << " Timing:\n";
  oss << "   - publish_period_ms           : " << publish_period_ms << "\n";

  oss << params_utils::JointResetConfig::summary();
  oss << params_utils::IntentResetConfig::summary();
  oss << params_utils::GripperResetConfig::summary();

  oss << " Initialization:\n";
  oss << "   - initial_joint_positions     : [";
  for (std::size_t i = 0; i < initial_joint_positions.size(); ++i) {
    oss << initial_joint_positions[i];
    if (i + 1 < initial_joint_positions.size()) {
      oss << ", ";
    }
  }
  oss << "]\n";
  oss << "   - initial_gripper_position    : " << initial_gripper_position << "\n\n";
  oss << " Slot:\n";
  oss << "   - slot_cmd_topic              : " << slot_cmd_topic << "\n";
  oss << "   - slot_state_topic            : " << slot_state_topic << "\n";
  oss << "   - initial_slot_status         : ["
      << (initial_slot_status[0] ? 1 : 0) << ", "
      << (initial_slot_status[1] ? 1 : 0) << "]\n\n";
  oss << "=============================================================================\n\n";

  return oss.str();
}

} // namespace fake_system

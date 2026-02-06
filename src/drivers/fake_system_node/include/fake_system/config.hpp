#pragma once

#include <rclcpp/rclcpp.hpp>
#include "engineer_bringup/base_config.hpp"

#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace fake_system {

struct FakeSystemConfig : public engineer_bringup::BaseRobotConfig {
  //  Timing 
  int publish_period_ms{20};
  bool publish_intent{false};

  //  Initialization 
  std::vector<double> initial_joint_positions{0, 0, 0, 0, 0, 0};
  double initial_gripper_position{0.0};

  //  API 
  static FakeSystemConfig Load(rclcpp::Node& node);
  void validate() const;      // keep only semantic / cross-field checks
  std::string summary() const;
};

inline FakeSystemConfig FakeSystemConfig::Load(rclcpp::Node& node) {
  FakeSystemConfig cfg;

  //  Load BaseConfig
  engineer_bringup::BaseRobotConfig::Load(
      node, static_cast<engineer_bringup::BaseRobotConfig &>(cfg));

  //  Small helpers 
  auto declare_get = [&](const std::string& name, auto& value) {
    node.declare_parameter(name, value);
    node.get_parameter(name, value);
  };

  auto declare_get_checked = [&](const std::string &name, auto &value, auto check, const char *msg) {
    node.declare_parameter(name, value);
    node.get_parameter(name, value);
    if (!check(value)) {
      throw std::runtime_error("ArmServoConfig: " + name + ": " + msg);
    }
  };

  //  Timing 
  declare_get_checked(
      "publish_period_ms", cfg.publish_period_ms,
      [](int v) { return v > 0; }, "must be > 0");
  declare_get("publish_intent", cfg.publish_intent);

  //  Initialization 
  cfg.initial_joint_positions.assign(static_cast<std::size_t>(cfg.joint_count), 0.0);

  declare_get("initial_joint_positions", cfg.initial_joint_positions);
  declare_get_checked(
      "initial_gripper_position", cfg.initial_gripper_position,
      [](double v) { return v >= 0.0 && v <= 0.03; }, "must be in [0, 0.03]");

  //  Finalize 
  cfg.validate();
  return cfg;
}

inline void FakeSystemConfig::validate() const {
  //  Semantic / cross-field checks 
  // 字段范围校验已在 declare_get_checked 做完，这里只保留组合语义校验。

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
  oss << "   - publish_intent              : " << (publish_intent ? "true" : "false") << "\n\n";

  oss << engineer_bringup::BaseRobotConfig::summary();

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
  oss << "=============================================================================\n\n";

  return oss.str();
}

}  // namespace fake_system

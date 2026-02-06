#pragma once

#include <rclcpp/rclcpp.hpp>
#include "engineer_bringup/base_config.hpp"

#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace arm_servo {

struct ArmServoConfig : public engineer_bringup::BaseRobotConfig {
  bool use_last_point_only{true};
  int publish_period_ms{30};   // 0 = no throttle

  static ArmServoConfig Load(rclcpp::Node &node);
  void validate() const;
  std::string summary() const;

};

inline ArmServoConfig ArmServoConfig::Load(rclcpp::Node &node) {
  ArmServoConfig cfg;

  auto declare_get = [&](const std::string &name, auto &value) {
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

  engineer_bringup::BaseRobotConfig::Load(node, static_cast<engineer_bringup::BaseRobotConfig &>(cfg));
  declare_get_checked("publish_period_ms", cfg.publish_period_ms,
                      [](int v) { return v >= 0 && v <= 1000; }, "must be in [0, 1000]");
  declare_get("use_last_point_only", cfg.use_last_point_only);

  cfg.validate();
  return cfg;
}

inline void ArmServoConfig::validate() const {
  if (publish_period_ms < 0 || publish_period_ms > 1000) {
    throw std::runtime_error("ArmServoConfig: publish_period_ms must be in [0, 1000]");
  }
}

inline std::string ArmServoConfig::summary() const {
  std::ostringstream oss;
  oss << "=======================\n";
  oss << " ArmServoNode Configuration\n\n";
  oss << engineer_bringup::BaseRobotConfig::summary();
  oss << " Rate:\n";
  oss << "   - publish_period_ms    : " << publish_period_ms << "\n";
  oss << "   - use_last_point_only  : " << (use_last_point_only ? "true" : "false") << "\n\n";
  oss << "\n=======================\n";
  return oss.str();
}

}  // namespace arm_servo

#pragma once

#include <rclcpp/rclcpp.hpp>

#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace arm_servo {

struct ArmServoConfig {
  std::string servo_out_topic{"/moveit_servo/joint_trajectory"};
  std::string output_topic{"/joint_commands"};
  bool use_last_point_only{true};
  int publish_period_ms{30};   // 0 = no throttle

  int joint_count{6};
  std::vector<std::string> joint_names{};
  std::unordered_map<std::string, std::size_t> joint_index{};

  static ArmServoConfig Load(rclcpp::Node &node);
  void validate() const;
  std::string summary() const;

 private:
  static std::vector<std::string> default_joint_names(int count) {
    std::vector<std::string> names;
    if (count <= 0) {
      return names;
    }
    names.reserve(static_cast<std::size_t>(count));
    for (int i = 1; i <= count; ++i) {
      names.emplace_back("joint" + std::to_string(i));
    }
    return names;
  }
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

  declare_get_checked("servo_out_topic", cfg.servo_out_topic, [](const std::string &v) {
    return !v.empty();
  }, "must not be empty");
  declare_get_checked("output_topic", cfg.output_topic, [](const std::string &v) { return !v.empty(); },
                      "must not be empty");
  declare_get_checked("publish_period_ms", cfg.publish_period_ms,
                      [](int v) { return v >= 0 && v <= 1000; }, "must be in [0, 1000]");
  declare_get("use_last_point_only", cfg.use_last_point_only);
  declare_get_checked("joint_count", cfg.joint_count,
                      [](int v) { return v >= 1 && v <= 12; }, "must be in [1, 12]");

  cfg.joint_names = default_joint_names(cfg.joint_count);
  declare_get("joint_names", cfg.joint_names);
  if (cfg.joint_names.empty()) {
    cfg.joint_names = default_joint_names(cfg.joint_count);
  }

  cfg.joint_index.clear();
  cfg.joint_index.reserve(cfg.joint_names.size());
  for (std::size_t i = 0; i < cfg.joint_names.size(); ++i) {
    cfg.joint_index.emplace(cfg.joint_names[i], i);
  }

  cfg.validate();
  return cfg;
}

inline void ArmServoConfig::validate() const {
  if (publish_period_ms < 0 || publish_period_ms > 1000) {
    throw std::runtime_error("ArmServoConfig: publish_period_ms must be in [0, 1000]");
  }
  if (joint_count < 1 || joint_count > 12) {
    throw std::runtime_error("ArmServoConfig: joint_count must be in [1, 12]");
  }
  if (static_cast<int>(joint_names.size()) != joint_count) {
    throw std::runtime_error("ArmServoConfig: joint_names size must match joint_count");
  }
  if (joint_index.size() != joint_names.size()) {
    throw std::runtime_error("ArmServoConfig: joint_index size must match joint_names size");
  }
  for (const auto &name : joint_names) {
    if (name.empty()) {
      throw std::runtime_error("ArmServoConfig: joint_names must not contain empty strings");
    }
  }
}

inline std::string ArmServoConfig::summary() const {
  std::ostringstream oss;
  oss << "=======================\n";
  oss << " ArmServoNode Configuration\n\n";
  oss << " Topics:\n";
  oss << "   - servo_out_topic      : " << servo_out_topic << "\n";
  oss << "   - output_topic         : " << output_topic << "\n\n";
  oss << " Rate:\n";
  oss << "   - publish_period_ms    : " << publish_period_ms << "\n";
  oss << "   - use_last_point_only  : " << (use_last_point_only ? "true" : "false") << "\n\n";
  oss << " Joint layout:\n";
  oss << "   - joint_count          : " << joint_count << "\n";
  oss << "   - joint_names          : ";
  for (std::size_t i = 0; i < joint_names.size(); ++i) {
    oss << joint_names[i];
    if (i + 1 < joint_names.size()) {
      oss << ", ";
    }
  }
  oss << "\n=======================\n";
  return oss.str();
}

}  // namespace arm_servo

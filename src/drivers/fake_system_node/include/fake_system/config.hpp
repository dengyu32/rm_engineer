#pragma once

#include <rclcpp/rclcpp.hpp>

#include <cstddef>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace fake_system {

struct FakeSystemConfig {
  //  Timing 
  int publish_period_ms{20};
  bool publish_intent{false};

  //  Topics 
  std::string hfsm_intent_topic{"/hfsm_intents"};
  std::string joint_states_topic{"/joint_states"};
  std::string joint_states_verbose_topic{"/joint_states_verbose"};
  std::string joint_states_custom_topic{"/joint_states_custom"};
  std::string joint_cmd_topic{"/joint_commands"};
  std::string gripper_cmd_topic{"/gripper_commands"};

  //  Joint layout
  int joint_count{6};
  std::vector<std::string> joint_names{};

  //  Initialization 
  std::vector<double> initial_joint_positions{0, 0, 0, 0, 0, 0};
  double initial_gripper_position{0.0};

  //  API 
  static FakeSystemConfig Load(rclcpp::Node& node);
  void validate() const;      // keep only semantic / cross-field checks
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

inline FakeSystemConfig FakeSystemConfig::Load(rclcpp::Node& node) {
  FakeSystemConfig cfg;

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
      "publish_period_ms",
      cfg.publish_period_ms,
      [](int v) { return v > 0; },
      "must be > 0");

  declare_get("publish_intent", cfg.publish_intent);

  //  Topics (usually fixed; no check) 
  declare_get("hfsm_intent_topic", cfg.hfsm_intent_topic);
  declare_get("joint_states_topic", cfg.joint_states_topic);
  declare_get("joint_states_verbose_topic", cfg.joint_states_verbose_topic);
  declare_get("joint_states_custom_topic", cfg.joint_states_custom_topic);
  declare_get("joint_cmd_topic", cfg.joint_cmd_topic);
  declare_get("gripper_cmd_topic", cfg.gripper_cmd_topic);

  //  Joint Layout
  declare_get_checked("joint_count", cfg.joint_count,
                      [](int v){ return v >= 1 && v <= 12; },
                      "must be in [1, 12]");

  cfg.joint_names = default_joint_names(cfg.joint_count);
  declare_get("joint_names", cfg.joint_names);
  if (cfg.joint_names.empty()) {
    cfg.joint_names = default_joint_names(cfg.joint_count);
  }

  //  Initialization 
  declare_get("initial_joint_positions", cfg.initial_joint_positions);

  declare_get_checked(
      "initial_gripper_position",
      cfg.initial_gripper_position,
      [](double v) { return v >= 0.0 && v <= 0.03; },
      "must be in [0, 0.03]");

  //  Finalize 
  cfg.validate();
  return cfg;
}

inline void FakeSystemConfig::validate() const {
  //  Semantic / cross-field checks 
  // 字段范围校验已在 declare_get_checked 做完，这里只保留组合语义校验。

  if (static_cast<int>(joint_names.size()) != joint_count) {
    throw std::runtime_error("ArmServoConfig: joint_names size must match joint_count");
  }
  if (initial_joint_positions.size() != 6) {
    std::ostringstream oss;
    oss << "FakeSystemConfig: initial_joint_positions must have 6 elements, got "
        << initial_joint_positions.size();
    throw std::runtime_error(oss.str());
  }
  for (const auto &name : joint_names) {
    if (name.empty()) {
      throw std::runtime_error("ArmServoConfig: joint_names must not contain empty strings");
    }
  }
}

inline std::string FakeSystemConfig::summary() const {
  std::ostringstream oss;

  oss << "=============================================================================\n";
  oss << " FakeSystemNode Configuration\n";

  oss << " Timing:\n";
  oss << "   - publish_period_ms           : " << publish_period_ms << "\n";
  oss << "   - publish_intent              : " << (publish_intent ? "true" : "false") << "\n\n";

  oss << " Topics:\n";
  oss << "   - hfsm_intent_topic           : " << hfsm_intent_topic << "\n";
  oss << "   - joint_states_topic          : " << joint_states_topic << "\n";
  oss << "   - joint_states_verbose_topic  : " << joint_states_verbose_topic << "\n";
  oss << "   - joint_states_custom_topic   : " << joint_states_custom_topic << "\n";
  oss << "   - joint_cmd_topic             : " << joint_cmd_topic << "\n";
  oss << "   - gripper_cmd_topic           : " << gripper_cmd_topic << "\n\n";

  oss << " Joint layout:\n";
  oss << "   - joint_count          : " << joint_count << "\n";
  oss << "   - joint_names          : ";
  for (std::size_t i = 0; i < joint_names.size(); ++i) {
    oss << joint_names[i];
    if (i + 1 < joint_names.size()) {
      oss << ", ";
    }
  }

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
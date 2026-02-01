// Strongly-typed configuration for FakeSystemNode.
#pragma once

#include <rclcpp/rclcpp.hpp>

#include <param_utils/param_snapshot.hpp>

#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace fake_system {

// ============================================================================
// Strongly-typed configuration
// ============================================================================
struct FakeSystemConfig {
  // -----------------------------
  // Parameters
  // -----------------------------
  int publish_period_ms{20};

  std::string hfsm_intent_topic{"/hfsm_intents"};
  std::string joint_states_topic{"/joint_states"};
  std::string joint_states_verbose_topic{"/joint_states_verbose"};
  std::string joint_states_custom_topic{"/joint_states_custom"};
  std::string joint_cmd_topic{"/joint_commands"};
  std::string gripper_cmd_topic{"/gripper_commands"};

  std::vector<double> initial_joint_positions{0, 0, 0, 0, 0, 0};
  double initial_gripper_position{0.0};

  // 默认不再向 /hfsm_intents 主动刷 IDLE，避免覆盖外部意图
  bool publish_intent{false};

  // -----------------------------
  // Debug / summary
  // -----------------------------
  std::vector<param_utils::ParamKV> params_snapshot{};

  static FakeSystemConfig Load(rclcpp::Node& node);
};

// ============================================================================
// Load implementation (ROS-only, with lambda declare_get)
// ============================================================================
inline FakeSystemConfig FakeSystemConfig::Load(rclcpp::Node& node) {
  FakeSystemConfig cfg;

  // ------------------------------------------------------------
  // 1) declare + get helper
  //    - declare_parameter(name, default)
  //    - then read final effective value (YAML/launch overrides apply)
  // ------------------------------------------------------------
  auto declare_get = [&](const std::string& name, auto& value) {
    node.declare_parameter(name, value);
    node.get_parameter(name, value);
  };

  // ------------------------------------------------------------
  // 2) declare/get parameters
  // ------------------------------------------------------------
  declare_get("publish_period_ms", cfg.publish_period_ms);

  declare_get("hfsm_intent_topic", cfg.hfsm_intent_topic);
  declare_get("joint_states_topic", cfg.joint_states_topic);
  declare_get("joint_states_verbose_topic", cfg.joint_states_verbose_topic);
  declare_get("joint_states_custom_topic", cfg.joint_states_custom_topic);
  declare_get("joint_cmd_topic", cfg.joint_cmd_topic);
  declare_get("gripper_cmd_topic", cfg.gripper_cmd_topic);

  declare_get("initial_joint_positions", cfg.initial_joint_positions);
  declare_get("initial_gripper_position", cfg.initial_gripper_position);

  declare_get("publish_intent", cfg.publish_intent);

  // ------------------------------------------------------------
  // 3) basic validation (no atcf_core)
  // ------------------------------------------------------------
  if (cfg.publish_period_ms <= 0) {
    throw std::runtime_error("FakeSystemConfig: publish_period_ms must be > 0");
  }

  if (cfg.hfsm_intent_topic.empty()) {
    throw std::runtime_error("FakeSystemConfig: hfsm_intent_topic must not be empty");
  }
  if (cfg.joint_states_topic.empty()) {
    throw std::runtime_error("FakeSystemConfig: joint_states_topic must not be empty");
  }
  if (cfg.joint_cmd_topic.empty()) {
    throw std::runtime_error("FakeSystemConfig: joint_cmd_topic must not be empty");
  }

  if (cfg.initial_joint_positions.size() != 6) {
    std::ostringstream oss;
    oss << "FakeSystemConfig: initial_joint_positions must have 6 elements, got "
        << cfg.initial_joint_positions.size();
    throw std::runtime_error(oss.str());
  }

  // ------------------------------------------------------------
  // 4) snapshot (all declared params)
  // ------------------------------------------------------------
  cfg.params_snapshot = param_utils::CollectAllParams(node);

  return cfg;
}

}  // namespace fake_system

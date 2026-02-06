#pragma once

#include <rclcpp/rclcpp.hpp>

#include <sstream>
#include <stdexcept>
#include <string>

namespace top_hfsm {

struct ArmSolveClientConfig {
  std::string arm_action_name{"move_arm"};
  int arm_server_wait_ms{100};

  static ArmSolveClientConfig Load(rclcpp::Node &node);
  void validate() const;
  std::string summary() const;
};

struct MoveItServoClientConfig {
  std::string servo_start_service{"/servo_node/start_servo"};
  std::string servo_unpause_service{"/servo_node/unpause_servo"};
  std::string servo_pause_service{"/servo_node/pause_servo"};
  std::string servo_stop_service{"/servo_node/stop_servo"};
  std::string servo_reset_status_service{"/servo_node/reset_servo_status"};
  std::string delta_twist_topic{"/servo_node/delta_twist_cmds"};
  std::string delta_joint_topic{"/servo_node/delta_joint_cmds"};
  std::string joint_states_custom_topic{"/joint_states_custom"};
  std::string joint_states_verbose_topic{"/joint_states_verbose"};
  std::string servo_status_topic{};

  static MoveItServoClientConfig Load(rclcpp::Node &node);
  void validate() const;
  std::string summary() const;
};

struct GripperConfig {
  std::string gripper_cmd_topic{"/gripper_commands"};
  int gripper_publish_period_ms{20};

  static GripperConfig Load(rclcpp::Node &node);
  void validate() const;
  std::string summary() const;
};

struct TopHFSMConfig {
  std::string intent_out_topic{"/intents_from_hfsm"};
  int hfsm_update_period_ms{20};
  ArmSolveClientConfig arm_solve_client;
  MoveItServoClientConfig moveit_servo_client;
  GripperConfig gripper;

  static TopHFSMConfig Load(rclcpp::Node &node);
  void validate() const;
  std::string summary() const;
};

namespace detail {
inline bool not_empty(const std::string &v) { return !v.empty(); }
}  // namespace detail

inline ArmSolveClientConfig ArmSolveClientConfig::Load(rclcpp::Node &node) {
  ArmSolveClientConfig cfg;
  auto declare_get = [&](const std::string &name, auto &value) {
    node.declare_parameter(name, value);
    node.get_parameter(name, value);
  };
  auto declare_get_checked = [&](const std::string &name, auto &value, auto check, const char *msg) {
    node.declare_parameter(name, value);
    node.get_parameter(name, value);
    if (!check(value)) {
      throw std::runtime_error("ArmSolveClientConfig: " + name + ": " + msg);
    }
  };
  declare_get("arm_action_name", cfg.arm_action_name);
  declare_get_checked("arm_server_wait_ms", cfg.arm_server_wait_ms,
                      [](int v) { return v >= 100 && v <= 10000; }, "must be in [100, 10000]");
  cfg.validate();
  return cfg;
}

inline void ArmSolveClientConfig::validate() const {
  if (arm_action_name.empty()) {
    throw std::runtime_error("ArmSolveClientConfig: arm_action_name must not be empty");
  }
}

inline std::string ArmSolveClientConfig::summary() const {
  std::ostringstream oss;
  oss << "    - arm_action_name     : " << arm_action_name << "\n";
  oss << "    - arm_server_wait_ms  : " << arm_server_wait_ms << "\n";
  return oss.str();
}

inline MoveItServoClientConfig MoveItServoClientConfig::Load(rclcpp::Node &node) {
  MoveItServoClientConfig cfg;
  auto declare_get = [&](const std::string &name, auto &value) {
    node.declare_parameter(name, value);
    node.get_parameter(name, value);
  };

  declare_get("servo_start_service", cfg.servo_start_service);
  declare_get("servo_unpause_service", cfg.servo_unpause_service);
  declare_get("servo_pause_service", cfg.servo_pause_service);
  declare_get("servo_stop_service", cfg.servo_stop_service);
  declare_get("servo_reset_status_service", cfg.servo_reset_status_service);
  declare_get("delta_twist_topic", cfg.delta_twist_topic);
  declare_get("delta_joint_topic", cfg.delta_joint_topic);
  declare_get("joint_states_custom_topic", cfg.joint_states_custom_topic);
  declare_get("joint_states_verbose_topic", cfg.joint_states_verbose_topic);
  declare_get("servo_status_topic", cfg.servo_status_topic);

  cfg.validate();
  return cfg;
}

inline void MoveItServoClientConfig::validate() const {
  if (!detail::not_empty(servo_start_service) ||
      !detail::not_empty(servo_unpause_service) ||
      !detail::not_empty(servo_pause_service) ||
      !detail::not_empty(servo_stop_service) ||
      !detail::not_empty(servo_reset_status_service)) {
    throw std::runtime_error("MoveItServoClientConfig: all service names must be non-empty");
  }
  if (!detail::not_empty(delta_twist_topic) || !detail::not_empty(delta_joint_topic) ||
      !detail::not_empty(joint_states_custom_topic) || !detail::not_empty(joint_states_verbose_topic)) {
    throw std::runtime_error("MoveItServoClientConfig: all topic names must be non-empty");
  }
}

inline std::string MoveItServoClientConfig::summary() const {
  std::ostringstream oss;
  oss << "    - servo_start_service       : " << servo_start_service << "\n";
  oss << "    - servo_unpause_service     : " << servo_unpause_service << "\n";
  oss << "    - servo_pause_service       : " << servo_pause_service << "\n";
  oss << "    - servo_stop_service        : " << servo_stop_service << "\n";
  oss << "    - servo_reset_status_service: " << servo_reset_status_service << "\n";
  oss << "    - delta_twist_topic         : " << delta_twist_topic << "\n";
  oss << "    - delta_joint_topic         : " << delta_joint_topic << "\n";
  oss << "    - joint_states_custom_topic : " << joint_states_custom_topic << "\n";
  oss << "    - joint_states_verbose_topic: " << joint_states_verbose_topic << "\n";
  if (!servo_status_topic.empty()) {
    oss << "    - servo_status_topic        : " << servo_status_topic << "\n";
  }
  return oss.str();
}

inline GripperConfig GripperConfig::Load(rclcpp::Node &node) {
  GripperConfig cfg;
  auto declare_get = [&](const std::string &name, auto &value) {
    node.declare_parameter(name, value);
    node.get_parameter(name, value);
  };

  declare_get("gripper_cmd_topic", cfg.gripper_cmd_topic);
  declare_get("gripper_publish_period_ms", cfg.gripper_publish_period_ms);

  cfg.validate();
  return cfg;
}

inline void GripperConfig::validate() const {
  if (gripper_cmd_topic.empty()) {
    throw std::runtime_error("GripperConfig: gripper_cmd_topic must not be empty");
  }
  if (gripper_publish_period_ms < 1 || gripper_publish_period_ms > 1000) {
    throw std::runtime_error("GripperConfig: gripper_publish_period_ms must be in [1, 1000]");
  }
}

inline std::string GripperConfig::summary() const {
  std::ostringstream oss;
  oss << "    - gripper_cmd_topic        : " << gripper_cmd_topic << "\n";
  oss << "    - gripper_publish_period_ms: " << gripper_publish_period_ms << "\n";
  return oss.str();
}

inline TopHFSMConfig TopHFSMConfig::Load(rclcpp::Node &node) {
  TopHFSMConfig cfg;
  auto declare_get = [&](const std::string &name, auto &value) {
    node.declare_parameter(name, value);
    node.get_parameter(name, value);
  };

  declare_get("intent_out_topic", cfg.intent_out_topic);
  declare_get("hfsm_update_period_ms", cfg.hfsm_update_period_ms);

  cfg.arm_solve_client = ArmSolveClientConfig::Load(node);
  cfg.moveit_servo_client = MoveItServoClientConfig::Load(node);
  cfg.gripper = GripperConfig::Load(node);

  cfg.validate();
  return cfg;
}

inline void TopHFSMConfig::validate() const {
  if (intent_out_topic.empty()) {
    throw std::runtime_error("TopHFSMConfig: intent_out_topic must not be empty");
  }
  if (hfsm_update_period_ms < 1 || hfsm_update_period_ms > 1000) {
    throw std::runtime_error("TopHFSMConfig: hfsm_update_period_ms must be in [1, 1000]");
  }
}

inline std::string TopHFSMConfig::summary() const {
  std::ostringstream oss;
  oss << "=============================\n";
  oss << " TopHFSMNode Configuration\n\n";
  oss << " Topics:\n";
  oss << "   - intent_out_topic   : " << intent_out_topic << "\n";
  oss << "   - hfsm_update_period_ms: " << hfsm_update_period_ms << "\n\n";
  oss << " ArmSolve Client:\n";
  oss << arm_solve_client.summary() << "\n";
  oss << " MoveIt Servo Client:\n";
  oss << moveit_servo_client.summary() << "\n";
  oss << " Gripper Client:\n";
  oss << gripper.summary() << "\n";
  oss << "=============================\n";
  return oss.str();
}

}  // namespace top_hfsm

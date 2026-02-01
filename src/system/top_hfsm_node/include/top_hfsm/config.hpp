// Strongly-typed configs for top_hfsm package.
#pragma once

#include <rclcpp/rclcpp.hpp>

#include <param_utils/param_snapshot.hpp>

#include <stdexcept>
#include <string>
#include <vector>

namespace top_hfsm {

struct ArmSolveClientConfig {
  std::string arm_action_name{"move_arm"};
  int arm_server_wait_ms{100};

  std::vector<param_utils::ParamKV> params_snapshot{};

  static ArmSolveClientConfig Load(rclcpp::Node &node) {
    ArmSolveClientConfig cfg;
    auto declare_get = [&](const std::string& name, auto& value) {
      node.declare_parameter(name, value);
      node.get_parameter(name, value);
    };

    declare_get("arm_action_name", cfg.arm_action_name);
    declare_get("arm_server_wait_ms", cfg.arm_server_wait_ms);

    if (cfg.arm_action_name.empty()) {
      throw std::runtime_error("ArmSolveClientConfig: arm_action_name must not be empty");
    }
    if (cfg.arm_server_wait_ms < 1 || cfg.arm_server_wait_ms > 10000) {
      throw std::runtime_error("ArmSolveClientConfig: arm_server_wait_ms must be in [1, 10000]");
    }

    cfg.params_snapshot = param_utils::CollectAllParams(node);
    return cfg;
  }
};

struct MoveItServoClientConfig {
  // Services
  std::string servo_start_service{"/servo_node/start_servo"};
  std::string servo_unpause_service{"/servo_node/unpause_servo"};
  std::string servo_pause_service{"/servo_node/pause_servo"};
  std::string servo_stop_service{"/servo_node/stop_servo"};
  std::string servo_reset_status_service{"/servo_node/reset_servo_status"};

  // Topics
  std::string delta_twist_topic{"/servo_node/delta_twist_cmds"};
  std::string delta_joint_topic{"/servo_node/delta_joint_cmds"};
  std::string joint_states_custom_topic{"/joint_states_custom"};
  std::string joint_states_verbose_topic{"/joint_states_verbose"};
  std::string servo_status_topic{};

  std::vector<param_utils::ParamKV> params_snapshot{};

  static MoveItServoClientConfig Load(rclcpp::Node &node) {
    MoveItServoClientConfig cfg;
    auto declare_get = [&](const std::string& name, auto& value) {
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

    if (cfg.servo_start_service.empty()) {
      throw std::runtime_error("MoveItServoClientConfig: servo_start_service must not be empty");
    }
    if (cfg.servo_unpause_service.empty()) {
      throw std::runtime_error("MoveItServoClientConfig: servo_unpause_service must not be empty");
    }
    if (cfg.servo_pause_service.empty()) {
      throw std::runtime_error("MoveItServoClientConfig: servo_pause_service must not be empty");
    }
    if (cfg.servo_stop_service.empty()) {
      throw std::runtime_error("MoveItServoClientConfig: servo_stop_service must not be empty");
    }
    if (cfg.servo_reset_status_service.empty()) {
      throw std::runtime_error("MoveItServoClientConfig: servo_reset_status_service must not be empty");
    }
    if (cfg.delta_twist_topic.empty()) {
      throw std::runtime_error("MoveItServoClientConfig: delta_twist_topic must not be empty");
    }
    if (cfg.delta_joint_topic.empty()) {
      throw std::runtime_error("MoveItServoClientConfig: delta_joint_topic must not be empty");
    }
    if (cfg.joint_states_custom_topic.empty()) {
      throw std::runtime_error("MoveItServoClientConfig: joint_states_custom_topic must not be empty");
    }
    if (cfg.joint_states_verbose_topic.empty()) {
      throw std::runtime_error("MoveItServoClientConfig: joint_states_verbose_topic must not be empty");
    }

    cfg.params_snapshot = param_utils::CollectAllParams(node);
    return cfg;
  }
};

struct GripperConfig {
  std::string gripper_cmd_topic{"/gripper_commands"};
  int gripper_publish_period_ms{20};
  std::vector<param_utils::ParamKV> params_snapshot{};

  static GripperConfig Load(rclcpp::Node &node) {
    GripperConfig cfg;
    auto declare_get = [&](const std::string& name, auto& value) {
      node.declare_parameter(name, value);
      node.get_parameter(name, value);
    };

    declare_get("gripper_cmd_topic", cfg.gripper_cmd_topic);
    declare_get("gripper_publish_period_ms", cfg.gripper_publish_period_ms);

    if (cfg.gripper_cmd_topic.empty()) {
      throw std::runtime_error("GripperConfig: gripper_cmd_topic must not be empty");
    }
    if (cfg.gripper_publish_period_ms < 1 || cfg.gripper_publish_period_ms > 1000) {
      throw std::runtime_error("GripperConfig: gripper_publish_period_ms must be in [1, 1000]");
    }

    cfg.params_snapshot = param_utils::CollectAllParams(node);
    return cfg;
  }
};

struct TopHFSMConfig {
  std::string hfsm_intent_topic{"/hfsm_intents"};
  int hfsm_update_period_ms{20};

  ArmSolveClientConfig arm_solve_client;
  MoveItServoClientConfig moveit_servo_client;
  GripperConfig gripper;

  std::vector<param_utils::ParamKV> params_snapshot{};

  static TopHFSMConfig Load(rclcpp::Node &node) {
    TopHFSMConfig cfg;
    auto declare_get = [&](const std::string& name, auto& value) {
      node.declare_parameter(name, value);
      node.get_parameter(name, value);
    };

    declare_get("hfsm_intent_topic", cfg.hfsm_intent_topic);
    declare_get("hfsm_update_period_ms", cfg.hfsm_update_period_ms);

    if (cfg.hfsm_intent_topic.empty()) {
      throw std::runtime_error("TopHFSMConfig: hfsm_intent_topic must not be empty");
    }
    if (cfg.hfsm_update_period_ms < 1 || cfg.hfsm_update_period_ms > 1000) {
      throw std::runtime_error("TopHFSMConfig: hfsm_update_period_ms must be in [1, 1000]");
    }

    cfg.arm_solve_client = ArmSolveClientConfig::Load(node);
    cfg.moveit_servo_client = MoveItServoClientConfig::Load(node);
    cfg.gripper = GripperConfig::Load(node);

    cfg.params_snapshot = param_utils::CollectAllParams(node);
    return cfg;
  }
};

} // namespace top_hfsm

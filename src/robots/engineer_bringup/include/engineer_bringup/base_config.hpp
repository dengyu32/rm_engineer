#pragma once

#include <rclcpp/rclcpp.hpp>

#include <cstddef>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace engineer_bringup {

struct BaseRobotConfig {
  // Joint layout
  int joint_count{6};
  std::vector<std::string> joint_names{};
  std::unordered_map<std::string, std::size_t> joint_index{};

  // Common topics
  std::string intent_out_topic{"/intents_to_hfsm"};
  std::string intent_in_topic{"/intents_from_hfsm"};
  std::string joint_states_topic{"/joint_states"};
  std::string joint_states_custom_topic{"/joint_states_custom"};
  std::string joint_states_verbose_topic{"/joint_states_verbose"};
  std::string joint_cmd_topic{"/joint_commands"};
  std::string gripper_cmd_topic{"/gripper_commands"};

  // Servo pipeline topics
  std::string servo_out_topic{"/moveit_servo/joint_trajectory"};
  std::string output_topic{"/joint_commands"};

  static void Load(rclcpp::Node &node, BaseRobotConfig &cfg) {
    auto declare_get = [&](const std::string &name, auto &value) {
      node.declare_parameter(name, value);
      node.get_parameter(name, value);
    };

    auto declare_get_checked = [&](const std::string &name, auto &value, auto check, const char *msg) {
      node.declare_parameter(name, value);
      node.get_parameter(name, value);
      if (!check(value)) {
        throw std::runtime_error("BaseRobotConfig: " + name + ": " + msg);
      }
    };

    // Joint layout
    declare_get_checked("joint_count", cfg.joint_count,
                        [](int v) { return v >= 1 && v <= 12; },
                        "must be in [1, 12]");

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

    // Common topics
    declare_get("intent_out_topic", cfg.intent_out_topic);
    declare_get("intent_in_topic", cfg.intent_in_topic);
    declare_get("joint_states_topic", cfg.joint_states_topic);
    declare_get("joint_states_custom_topic", cfg.joint_states_custom_topic);
    declare_get("joint_states_verbose_topic", cfg.joint_states_verbose_topic);
    declare_get("joint_cmd_topic", cfg.joint_cmd_topic);
    declare_get("gripper_cmd_topic", cfg.gripper_cmd_topic);

    // Servo pipeline topics
    declare_get("servo_out_topic", cfg.servo_out_topic);
    declare_get("output_topic", cfg.output_topic);

    cfg.validate();
  }

  void validate() const {
    if (joint_count < 1 || joint_count > 12) {
      throw std::runtime_error("BaseRobotConfig: joint_count must be in [1, 12]");
    }
    if (static_cast<int>(joint_names.size()) != joint_count) {
      throw std::runtime_error("BaseRobotConfig: joint_names size must match joint_count");
    }
    if (joint_index.size() != joint_names.size()) {
      throw std::runtime_error("BaseRobotConfig: joint_index size must match joint_names size");
    }
    for (const auto &name : joint_names) {
      if (name.empty()) {
        throw std::runtime_error("BaseRobotConfig: joint_names must not contain empty strings");
      }
    }
    if (intent_out_topic.empty() || intent_in_topic.empty() || joint_states_topic.empty() ||
        joint_states_custom_topic.empty() || joint_states_verbose_topic.empty() ||
        joint_cmd_topic.empty() || gripper_cmd_topic.empty()) {
      throw std::runtime_error("BaseRobotConfig: topics must not be empty");
    }
    if (servo_out_topic.empty() || output_topic.empty()) {
      throw std::runtime_error("BaseRobotConfig: servo topics must not be empty");
    }
  }

  std::string summary() const {
    std::ostringstream oss;
    oss << " Joint layout:\n";
    oss << "   - joint_count         : " << joint_count << "\n";
    oss << "   - joint_names         : ";
    for (std::size_t i = 0; i < joint_names.size(); ++i) {
      oss << joint_names[i];
      if (i + 1 < joint_names.size()) {
        oss << ", ";
      }
    }
    oss << "\n\n";

    oss << " Topics:\n";
    oss << "   - intent_out_topic        : " << intent_out_topic << "\n";
    oss << "   - intent_in_topic        : " << intent_in_topic << "\n";
    oss << "   - joint_states_topic       : " << joint_states_topic << "\n";
    oss << "   - joint_states_custom_topic: " << joint_states_custom_topic << "\n";
    oss << "   - joint_states_verbose_topic: " << joint_states_verbose_topic << "\n";
    oss << "   - joint_cmd_topic          : " << joint_cmd_topic << "\n";
    oss << "   - gripper_cmd_topic        : " << gripper_cmd_topic << "\n";

    oss << " Servo topics:\n";
    oss << "   - servo_out_topic          : " << servo_out_topic << "\n";
    oss << "   - output_topic             : " << output_topic << "\n\n";

    return oss.str();
  }

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

}  // namespace engineer_bringup

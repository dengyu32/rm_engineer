#pragma once

#include <rclcpp/rclcpp.hpp>

#include <cstddef>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace engineer_bringup {

// # engineer_bringup::detail 只是给配置实现复用的内部 helper 不承诺作为稳定 API

namespace detail {

//--------------------------------------
// Basic helpers
//--------------------------------------
template <class NodeT, class T>
inline void declare_get(NodeT& node, const std::string& name, T& value) {
  if (!node.has_parameter(name)) {
    node.declare_parameter(name, value);
  }
  node.get_parameter(name, value);
}

template <class NodeT, class T, class Check>
inline void declare_get_checked(NodeT& node,
                                const std::string& name,
                                T& value,
                                Check check,
                                const char* msg) {
  declare_get(node, name, value);
  if (!check(value)) {
    throw std::runtime_error(std::string("Config: ") + name + ": " + msg);
  }
}

//--------------------------------------
// Validators
//--------------------------------------
inline auto in_range(int low, int high) {
  return [=](int v) { return v >= low && v <= high; };
}

}  // namespace detail


struct BaseRobotConfig {

  // Joint layout
  int joint_count{6};
  std::vector<std::string> joint_names{};
  std::unordered_map<std::string, std::size_t> joint_index{};

  // Common topics
  std::string intent_cmd_topic{"/hfsm/intent_commands"};
  std::string intent_fb_topic{"/hfsm/intent_feedback"};

  std::string joint_states_topic{"/joint_states"};
  std::string joint_states_custom_topic{"/joint_states_custom"};
  std::string joint_states_verbose_topic{"/joint_states_verbose"};
  std::string joint_cmd_topic{"/joint_commands"};
  std::string gripper_cmd_topic{"/gripper_commands"};


  static void Load(rclcpp::Node &node, BaseRobotConfig &cfg) {
    //  using
    using engineer_bringup::detail::declare_get;

    //--------------------------------------
    //   Joint Layout
    //--------------------------------------

    //  joint_count : 关节个数
    declare_get(node,"joint_count", cfg.joint_count);

    //  joint_names : 关节名称 
    cfg.joint_names = default_joint_names(cfg.joint_count);
    declare_get(node,"joint_names", cfg.joint_names);
    if (cfg.joint_names.empty()) {
      cfg.joint_names = default_joint_names(cfg.joint_count);
    }

    // joint_index : 关节名称到索引的映射
    cfg.joint_index.clear();
    cfg.joint_index.reserve(cfg.joint_names.size());
    for (std::size_t i = 0; i < cfg.joint_names.size(); ++i) {
      cfg.joint_index.emplace(cfg.joint_names[i], i);
    }

    //--------------------------------------
    //   Common Topics
    //-------------------------------------- topics
    declare_get(node,"intent_cmd_topic", cfg.intent_cmd_topic);
    declare_get(node,"intent_fb_topic", cfg.intent_fb_topic);
    declare_get(node,"joint_states_topic", cfg.joint_states_topic);
    declare_get(node,"joint_states_custom_topic", cfg.joint_states_custom_topic);
    declare_get(node,"joint_states_verbose_topic", cfg.joint_states_verbose_topic);
    declare_get(node,"joint_cmd_topic", cfg.joint_cmd_topic);
    declare_get(node,"gripper_cmd_topic", cfg.gripper_cmd_topic);

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
    if (intent_cmd_topic.empty() || intent_fb_topic.empty() || joint_states_topic.empty() ||
        joint_states_custom_topic.empty() || joint_states_verbose_topic.empty() ||
        joint_cmd_topic.empty() || gripper_cmd_topic.empty()) {
      throw std::runtime_error("BaseRobotConfig: topics must not be empty");
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
    oss << "   - intent_cmd_topic        : " << intent_cmd_topic << "\n";
    oss << "   - intent_fb_topic        : " << intent_fb_topic << "\n";
    oss << "   - joint_states_topic       : " << joint_states_topic << "\n";
    oss << "   - joint_states_custom_topic: " << joint_states_custom_topic << "\n";
    oss << "   - joint_states_verbose_topic: " << joint_states_verbose_topic << "\n";
    oss << "   - joint_cmd_topic          : " << joint_cmd_topic << "\n";
    oss << "   - gripper_cmd_topic        : " << gripper_cmd_topic << "\n";

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

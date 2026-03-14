#pragma once

#include <rclcpp/rclcpp.hpp>

#include <cstddef>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "params_utils/core/params.hpp"

namespace params_utils {

struct JointResetConfig {
  // Joint layout
  int joint_count{6};
  std::vector<std::string> joint_names{};
  std::unordered_map<std::string, std::size_t> joint_index{};

  // Joint topics
  std::string joint_states_topic{"/joint_states"};
  std::string joint_states_custom_topic{"/joint_states_custom"};
  std::string joint_states_verbose_topic{"/joint_states_verbose"};
  std::string joint_cmd_topic{"/joint_commands"};

  static void Load(rclcpp::Node &node, JointResetConfig &cfg) {
    using params_utils::detail::declare_get;
    using params_utils::detail::declare_get_checked;

    //--------------------------------------
    //   Joint Layout
    //--------------------------------------

    declare_get_checked(
        node, "joint_count", cfg.joint_count,
        [](int v) { return v >= 1 && v <= 12; },
        "must be in [1, 12]");

    cfg.joint_names = default_joint_names(cfg.joint_count);
    declare_get(node, "joint_names", cfg.joint_names);
    if (cfg.joint_names.empty()) {
      cfg.joint_names = default_joint_names(cfg.joint_count);
    }

    cfg.joint_index.clear();
    cfg.joint_index.reserve(cfg.joint_names.size());
    for (std::size_t i = 0; i < cfg.joint_names.size(); ++i) {
      cfg.joint_index.emplace(cfg.joint_names[i], i);
    }

    //--------------------------------------
    //   Joint Topics
    //--------------------------------------

    declare_get(node, "joint_states_topic", cfg.joint_states_topic);
    declare_get(node, "joint_states_custom_topic", cfg.joint_states_custom_topic);
    declare_get(node, "joint_states_verbose_topic", cfg.joint_states_verbose_topic);
    declare_get(node, "joint_cmd_topic", cfg.joint_cmd_topic);

    cfg.validate();
  }

  void validate() const {
    if (static_cast<int>(joint_names.size()) != joint_count) {
      throw std::runtime_error("JointResetConfig: joint_names size must match joint_count");
    }
    if (joint_index.size() != joint_names.size()) {
      throw std::runtime_error("JointResetConfig: joint_index size must match joint_names size");
    }
    for (const auto &name : joint_names) {
      if (name.empty()) {
        throw std::runtime_error("JointResetConfig: joint_names must not contain empty strings");
      }
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

    oss << " Joint topics:\n";
    oss << "   - joint_states_topic        : " << joint_states_topic << "\n";
    oss << "   - joint_states_custom_topic : " << joint_states_custom_topic << "\n";
    oss << "   - joint_states_verbose_topic: " << joint_states_verbose_topic << "\n";
    oss << "   - joint_cmd_topic           : " << joint_cmd_topic << "\n";

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

}  // namespace params_utils

#pragma once

#include <rclcpp/rclcpp.hpp>

#include <sstream>
#include <stdexcept>
#include <string>

#include "params_utils/core/params.hpp"

namespace params_utils {

struct IntentResetConfig {
  std::string intent_cmd_topic{"/hfsm/intent_commands"};
  std::string intent_fb_topic{"/hfsm/intent_feedback"};

  static void Load(rclcpp::Node &node, IntentResetConfig &cfg) {
    using params_utils::detail::declare_get;
    declare_get(node, "intent_cmd_topic", cfg.intent_cmd_topic);
    declare_get(node, "intent_fb_topic", cfg.intent_fb_topic);
    cfg.validate();
  }

  void validate() const {}

  std::string summary() const {
    std::ostringstream oss;
    oss << " Intent topics:\n";
    oss << "   - intent_cmd_topic       : " << intent_cmd_topic << "\n";
    oss << "   - intent_fb_topic        : " << intent_fb_topic << "\n";
    return oss.str();
  }
};

}  // namespace params_utils

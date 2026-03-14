#pragma once

#include <rclcpp/rclcpp.hpp>

#include <sstream>
#include <stdexcept>
#include <string>

#include "params_utils/core/params.hpp"

namespace params_utils {

struct MoveItResetConfig {
  std::string group_name{"engineer_arm"};
  std::string ee_link_name{"link6"};

  static void Load(rclcpp::Node &node, MoveItResetConfig &cfg) {
    using params_utils::detail::declare_get_checked;
    declare_get_checked(
        node, "group_name", cfg.group_name,
        [](const std::string &v) { return !v.empty(); },
        "must not be empty");
    declare_get_checked(
        node, "ee_link_name", cfg.ee_link_name,
        [](const std::string &v) { return !v.empty(); },
        "must not be empty");
    cfg.validate();
  }

  void validate() const {}

  std::string summary() const {
    std::ostringstream oss;
    oss << " MoveIt:\n";
    oss << "   - group_name              : " << group_name << "\n";
    oss << "   - ee_link_name            : " << ee_link_name << "\n";
    return oss.str();
  }
};

}  // namespace params_utils

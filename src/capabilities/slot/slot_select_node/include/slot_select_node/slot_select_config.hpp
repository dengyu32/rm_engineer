#pragma once

#include <rclcpp/rclcpp.hpp>

#include <sstream>
#include <string>

#include "robot_config/robot_config.hpp"

namespace engineer_auto::slot_select_node {

struct SlotSelectConfig {
  std::string slot_state_topic{"/slot_states"};
  int slot_query_period_ms{100};
  bool slot0_occupied{false};
  bool slot1_occupied{false};

  static SlotSelectConfig load(rclcpp::Node &node) {
    SlotSelectConfig cfg;
    params_utils::detail::declare_get(node, "slot_state_topic", cfg.slot_state_topic);
    params_utils::detail::declare_get_checked(
        node, "slot_query_period_ms", cfg.slot_query_period_ms,
        [](int v) { return v > 0; },
        "must be > 0");
    params_utils::detail::declare_get(node, "slot0_occupied", cfg.slot0_occupied);
    params_utils::detail::declare_get(node, "slot1_occupied", cfg.slot1_occupied);
    cfg.validate();
    return cfg;
  }

  void validate() const {}

  std::string summary() const {
    std::ostringstream oss;
    oss << "=============================================================================\n";
    oss << " SlotSelect Configuration\n\n";
    oss << " Topics:\n";
    oss << "   - slot_state_topic      : " << slot_state_topic << "\n\n";
    oss << " Timing:\n";
    oss << "   - slot_query_period_ms  : " << slot_query_period_ms << "\n\n";
    oss << " Initial:\n";
    oss << "   - slot0_occupied        : " << (slot0_occupied ? 1 : 0) << "\n";
    oss << "   - slot1_occupied        : " << (slot1_occupied ? 1 : 0) << "\n";
    oss << "=============================================================================\n";
    return oss.str();
  }
};

} // namespace engineer_auto::slot_select_node

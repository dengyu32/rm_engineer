// Strongly-typed configuration for UsbCdcNode.
#pragma once

#include <rclcpp/rclcpp.hpp>

#include <param_utils/param_snapshot.hpp>

#include <stdexcept>
#include <string>
#include <vector>

namespace usb_cdc {

struct UsbCdcConfig {
  // Device identifiers
  int vendor_id{0x0483};
  int product_id{0x5740};
  int open_retry_attempts{5};
  int open_retry_delay_ms{500};

  // Timers
  int publish_period_ms{20};
  int send_period_ms{20};

  // Topics
  std::string hfsm_intent_topic{"/hfsm_intents"};
  std::string joint_states_topic{"/joint_states"};
  std::string joint_states_custom_topic{"/joint_states_custom"};
  std::string joint_states_verbose_topic{"/joint_states_verbose"};
  std::string joint_cmd_topic{"/joint_commands"};
  std::string gripper_cmd_topic{"/gripper_commands"};

  std::vector<param_utils::ParamKV> params_snapshot{};

  static UsbCdcConfig Load(rclcpp::Node &node) {
    UsbCdcConfig cfg;

    auto declare_get = [&](const std::string& name, auto& value) {
      node.declare_parameter(name, value);
      node.get_parameter(name, value);
    };

    declare_get("vendor_id", cfg.vendor_id);
    declare_get("product_id", cfg.product_id);
    declare_get("open_retry_attempts", cfg.open_retry_attempts);
    declare_get("open_retry_delay_ms", cfg.open_retry_delay_ms);
    declare_get("publish_period_ms", cfg.publish_period_ms);
    declare_get("send_period_ms", cfg.send_period_ms);
    declare_get("hfsm_intent_topic", cfg.hfsm_intent_topic);
    declare_get("joint_states_topic", cfg.joint_states_topic);
    declare_get("joint_states_custom_topic", cfg.joint_states_custom_topic);
    declare_get("joint_states_verbose_topic", cfg.joint_states_verbose_topic);
    declare_get("joint_cmd_topic", cfg.joint_cmd_topic);
    declare_get("gripper_cmd_topic", cfg.gripper_cmd_topic);

    if (cfg.vendor_id < 1 || cfg.vendor_id > 0xFFFF) {
      throw std::runtime_error("UsbCdcConfig: vendor_id must be in [1, 65535]");
    }
    if (cfg.product_id < 1 || cfg.product_id > 0xFFFF) {
      throw std::runtime_error("UsbCdcConfig: product_id must be in [1, 65535]");
    }
    if (cfg.open_retry_attempts < 1 || cfg.open_retry_attempts > 20) {
      throw std::runtime_error("UsbCdcConfig: open_retry_attempts must be in [1, 20]");
    }
    if (cfg.open_retry_delay_ms < 1 || cfg.open_retry_delay_ms > 10000) {
      throw std::runtime_error("UsbCdcConfig: open_retry_delay_ms must be in [1, 10000]");
    }
    if (cfg.publish_period_ms < 1 || cfg.publish_period_ms > 1000) {
      throw std::runtime_error("UsbCdcConfig: publish_period_ms must be in [1, 1000]");
    }
    if (cfg.send_period_ms < 1 || cfg.send_period_ms > 1000) {
      throw std::runtime_error("UsbCdcConfig: send_period_ms must be in [1, 1000]");
    }
    if (cfg.hfsm_intent_topic.empty()) {
      throw std::runtime_error("UsbCdcConfig: hfsm_intent_topic must not be empty");
    }
    if (cfg.joint_states_topic.empty()) {
      throw std::runtime_error("UsbCdcConfig: joint_states_topic must not be empty");
    }
    if (cfg.joint_states_custom_topic.empty()) {
      throw std::runtime_error("UsbCdcConfig: joint_states_custom_topic must not be empty");
    }
    if (cfg.joint_states_verbose_topic.empty()) {
      throw std::runtime_error("UsbCdcConfig: joint_states_verbose_topic must not be empty");
    }
    if (cfg.joint_cmd_topic.empty()) {
      throw std::runtime_error("UsbCdcConfig: joint_cmd_topic must not be empty");
    }
    if (cfg.gripper_cmd_topic.empty()) {
      throw std::runtime_error("UsbCdcConfig: gripper_cmd_topic must not be empty");
    }

    cfg.params_snapshot = param_utils::CollectAllParams(node);
    return cfg;
  }
};

} // namespace usb_cdc

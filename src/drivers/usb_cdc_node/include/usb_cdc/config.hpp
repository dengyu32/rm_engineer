#pragma once

#include <rclcpp/rclcpp.hpp>

#include <sstream>
#include <stdexcept>
#include <string>

namespace usb_cdc {

struct UsbCdcConfig {
  int vendor_id{0x0483};
  int product_id{0x5740};
  int open_retry_attempts{5};
  int open_retry_delay_ms{500};
  int publish_period_ms{20};
  int send_period_ms{20};
  std::string hfsm_intent_topic{"/hfsm_intents"};
  std::string joint_states_topic{"/joint_states"};
  std::string joint_states_custom_topic{"/joint_states_custom"};
  std::string joint_states_verbose_topic{"/joint_states_verbose"};
  std::string joint_cmd_topic{"/joint_commands"};
  std::string gripper_cmd_topic{"/gripper_commands"};

  static UsbCdcConfig Load(rclcpp::Node &node);
  void validate() const;
  std::string summary() const;
};

inline UsbCdcConfig UsbCdcConfig::Load(rclcpp::Node &node) {
  UsbCdcConfig cfg;

  auto declare_get = [&](const std::string &name, auto &value) {
    node.declare_parameter(name, value);
    node.get_parameter(name, value);
  };

  auto declare_get_checked = [&](const std::string &name, auto &value, auto check, const char *msg) {
    node.declare_parameter(name, value);
    node.get_parameter(name, value);
    if (!check(value)) {
      throw std::runtime_error("UsbCdcConfig: " + name + ": " + msg);
    }
  };

  declare_get_checked("vendor_id", cfg.vendor_id,
                      [](int v) { return v >= 1 && v <= 0xFFFF; }, "must be in [1, 65535]");
  declare_get_checked("product_id", cfg.product_id,
                      [](int v) { return v >= 1 && v <= 0xFFFF; }, "must be in [1, 65535]");
  declare_get_checked("open_retry_attempts", cfg.open_retry_attempts,
                      [](int v) { return v >= 1 && v <= 20; }, "must be in [1, 20]");
  declare_get_checked("open_retry_delay_ms", cfg.open_retry_delay_ms,
                      [](int v) { return v >= 1 && v <= 10000; }, "must be in [1, 10000]");
  declare_get_checked("publish_period_ms", cfg.publish_period_ms,
                      [](int v) { return v >= 1 && v <= 1000; }, "must be in [1, 1000]");
  declare_get_checked("send_period_ms", cfg.send_period_ms,
                      [](int v) { return v >= 1 && v <= 1000; }, "must be in [1, 1000]");
  declare_get("hfsm_intent_topic", cfg.hfsm_intent_topic);
  declare_get("joint_states_topic", cfg.joint_states_topic);
  declare_get("joint_states_custom_topic", cfg.joint_states_custom_topic);
  declare_get("joint_states_verbose_topic", cfg.joint_states_verbose_topic);
  declare_get("joint_cmd_topic", cfg.joint_cmd_topic);
  declare_get("gripper_cmd_topic", cfg.gripper_cmd_topic);

  cfg.validate();
  return cfg;
}

inline void UsbCdcConfig::validate() const {
  if (hfsm_intent_topic.empty() || joint_states_topic.empty() ||
      joint_states_custom_topic.empty() || joint_states_verbose_topic.empty() ||
      joint_cmd_topic.empty() || gripper_cmd_topic.empty()) {
    throw std::runtime_error("UsbCdcConfig: topics must not be empty");
  }
}

inline std::string UsbCdcConfig::summary() const {
  std::ostringstream oss;
  oss << "======================\n";
  oss << " USB CDC Configuration\n\n";
  oss << " Device:\n";
  oss << "   - vendor_id           : " << vendor_id << "\n";
  oss << "   - product_id          : " << product_id << "\n";
  oss << "   - open_retry_attempts : " << open_retry_attempts << "\n";
  oss << "   - open_retry_delay_ms : " << open_retry_delay_ms << "\n\n";
  oss << " Timing:\n";
  oss << "   - publish_period_ms   : " << publish_period_ms << "\n";
  oss << "   - send_period_ms      : " << send_period_ms << "\n\n";
  oss << " Topics:\n";
  oss << "   - hfsm_intent_topic        : " << hfsm_intent_topic << "\n";
  oss << "   - joint_states_topic       : " << joint_states_topic << "\n";
  oss << "   - joint_states_custom_topic: " << joint_states_custom_topic << "\n";
  oss << "   - joint_states_verbose_topic: " << joint_states_verbose_topic << "\n";
  oss << "   - joint_cmd_topic          : " << joint_cmd_topic << "\n";
  oss << "   - gripper_cmd_topic        : " << gripper_cmd_topic << "\n";
  oss << "======================\n";
  return oss.str();
}

}  // namespace usb_cdc

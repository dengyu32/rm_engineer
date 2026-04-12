// ============================================================================
//  usb_cdc_config.hpp
// ----------------------------------------------------------------------------
//  - 定义 USB CDC 相关配置结构体与加载函数
//  - 依赖 rclcpp 参数服务器，支持动态参数更新
// ============================================================================
#pragma once

// ROS2
#include <rclcpp/rclcpp.hpp>

// C++
#include <cstddef>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

// Project
#include "robot_config/robot_config.hpp"

namespace usb_cdc {

// ============================================================================
//  UsbCdcConfig
// ----------------------------------------------------------------------------
//  - USB 设备 + 话题 + 关节布局
// ============================================================================
struct UsbCdcConfig : public params_utils::JointResetConfig,
                      public params_utils::IntentResetConfig,
                      public params_utils::GripperResetConfig {
  //  Device
  int vendor_id{0x0483};
  int product_id{0x5740};

  //  Timing
  int publish_period_ms{10};
  int send_period_ms{10};

  //  Mode
  bool servo_teleop_mode{false};
  bool debug_override_enabled{false};
  int debug_intent_id{0};
  std::string slot_state_topic{"/slot_states"};
  std::string slot_cmd_topic{"/slot_cmds"};
  std::vector<double> startup_joint_targets{0.0, -0.6109, -2.1293, 0.0, 0.0, 0.0};

  //  API
  static UsbCdcConfig Load(rclcpp::Node &node);
  void validate() const;
  std::string summary() const;
};

// ============================================================================
//  LOAD
// ============================================================================
inline UsbCdcConfig UsbCdcConfig::Load(rclcpp::Node &node) {
  using params_utils::detail::declare_get;
  using params_utils::detail::declare_get_checked;
  using params_utils::detail::in_range;

  UsbCdcConfig cfg;

  //  Base layout + topics
  params_utils::JointResetConfig::Load(node, cfg);
  params_utils::IntentResetConfig::Load(node, cfg);
  params_utils::GripperResetConfig::Load(node, cfg);

  //  Device
  declare_get_checked(node, "vendor_id", cfg.vendor_id, in_range(1, 0xFFFF),
                      "must be in [1, 65535]");
  declare_get_checked(node, "product_id", cfg.product_id, in_range(1, 0xFFFF),
                      "must be in [1, 65535]");

  //  Timing
  declare_get_checked(node, "publish_period_ms", cfg.publish_period_ms,
                      in_range(1, 1000), "must be in [1, 1000]");
  declare_get_checked(node, "send_period_ms", cfg.send_period_ms,
                      in_range(1, 1000), "must be in [1, 1000]");

  //  Mode
  declare_get(node, "servo_teleop_mode", cfg.servo_teleop_mode);
  declare_get(node, "debug_override_enabled", cfg.debug_override_enabled);
  declare_get_checked(node, "debug_intent_id", cfg.debug_intent_id,
                      in_range(0, 255), "must be in [0, 255]");
  declare_get(node, "slot_state_topic", cfg.slot_state_topic);
  declare_get(node, "slot_cmd_topic", cfg.slot_cmd_topic);
  declare_get(node, "startup_joint_targets", cfg.startup_joint_targets);

  //  Finalize
  if (cfg.joint_count < 1 || cfg.joint_count > 6) {
    throw std::runtime_error("UsbCdcConfig: joint_count must be in [1, 6]");
  }
  cfg.validate();
  return cfg;
}

// ============================================================================
//  VALIDATE
// ============================================================================
inline void UsbCdcConfig::validate() const {
  params_utils::JointResetConfig::validate();
  params_utils::IntentResetConfig::validate();
  params_utils::GripperResetConfig::validate();
  if (startup_joint_targets.size() != static_cast<std::size_t>(joint_count)) {
    throw std::runtime_error(
        "UsbCdcConfig: startup_joint_targets size must match joint_count");
  }
}

// ============================================================================
//  SUMMARY
// ============================================================================
inline std::string UsbCdcConfig::summary() const {
  std::ostringstream oss;
  oss << "=============================================================================\n";
  oss << " USB CDC Configuration\n\n";

  oss << " Device:\n";
  oss << "   - vendor_id           : " << vendor_id << "\n";
  oss << "   - product_id          : " << product_id << "\n";
  oss << "\n";

  oss << " Timing:\n";
  oss << "   - publish_period_ms   : " << publish_period_ms << "\n";
  oss << "   - send_period_ms      : " << send_period_ms << "\n";
  oss << "\n";

  oss << " Mode:\n";
  oss << "   - servo_teleop_mode           : "
      << (servo_teleop_mode ? "true" : "false") << "\n";
  oss << "   - debug_override_enabled        : "
      << (debug_override_enabled ? "true" : "false") << "\n";
  oss << "   - debug_intent_id             : " << debug_intent_id << "\n\n";
  oss << " Startup:\n";
  oss << "   - startup_joint_targets         : [";
  for (std::size_t i = 0; i < startup_joint_targets.size(); ++i) {
    oss << startup_joint_targets[i];
    if (i + 1 < startup_joint_targets.size()) {
      oss << ", ";
    }
  }
  oss << "]\n\n";
  oss << " Slot:\n";
  oss << "   - slot_state_topic    : " << slot_state_topic << "\n";
  oss << "   - slot_cmd_topic      : " << slot_cmd_topic << "\n\n";

  oss << params_utils::JointResetConfig::summary();
  oss << params_utils::IntentResetConfig::summary();
  oss << params_utils::GripperResetConfig::summary();
  oss << "=============================================================================\n";
  return oss.str();
}

} // namespace usb_cdc

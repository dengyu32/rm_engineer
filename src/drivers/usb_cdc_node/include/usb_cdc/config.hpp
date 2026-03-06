#pragma once

#include <rclcpp/rclcpp.hpp>
#include "engineer_bringup/base_config.hpp"

#include <sstream>
#include <stdexcept>
#include <string>

namespace usb_cdc {

struct UsbCdcConfig : public engineer_bringup::BaseRobotConfig {
  //  Device
  int vendor_id{0x0483};
  int product_id{0x5740};

  //  Timing
  int publish_period_ms{33};
  int send_period_ms{20};

  //  Mode
  bool servo_teleop_mode{false};

  //  API
  static UsbCdcConfig Load(rclcpp::Node &node);
  void validate() const;
  std::string summary() const;
};


inline UsbCdcConfig UsbCdcConfig::Load(rclcpp::Node &node) {
  //  using
  using engineer_bringup::detail::declare_get;
  using engineer_bringup::detail::declare_get_checked;
  using engineer_bringup::detail::in_range;

  UsbCdcConfig cfg;

  //  Base Config
  engineer_bringup::BaseRobotConfig::Load(
    node, static_cast<engineer_bringup::BaseRobotConfig &>(cfg));

  //  Device
  declare_get_checked(
      node, "vendor_id", cfg.vendor_id,
      in_range(1, 0xFFFF),
      "must be in [1, 65535]");
  declare_get_checked(
      node, "product_id", cfg.product_id,
      in_range(1, 0xFFFF),
      "must be in [1, 65535]");

  //  Timing
  declare_get_checked(
      node, "publish_period_ms", cfg.publish_period_ms,
      in_range(1, 1000),
      "must be in [1, 1000]");
  declare_get_checked(
      node, "send_period_ms", cfg.send_period_ms,
      in_range(1, 1000),
      "must be in [1, 1000]");

  //  Mode
  declare_get(node, "servo_teleop_mode", cfg.servo_teleop_mode);

  //  Finalize
  cfg.validate();
  return cfg;
}

inline void UsbCdcConfig::validate() const {
  if (joint_count < 1 || joint_count > 6) {
    throw std::runtime_error("UsbCdcConfig: joint_count must be in [1, 6]");
  }
}

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
  oss << "   - send_period_ms      : " << send_period_ms << "\n\n";

  oss << " Mode:\n";
  oss << "   - servo_teleop_mode   : " << (servo_teleop_mode ? "true" : "false") << "\n\n";

  oss << engineer_bringup::BaseRobotConfig::summary();
  oss << "=============================================================================\n";
  return oss.str();
}

}  // namespace usb_cdc

#pragma once

#include <rclcpp/rclcpp.hpp>

#include <sstream>
#include <string>

#include "robot_config/robot_config.hpp"

namespace engineer_auto {

// ============================================================================
//  AutoNodeConfig
// ----------------------------------------------------------------------------
//  私有参数（动态参数）
//  - update_period_ms: 定时器周期，用于定期检查和执行任务
//  - status_period_ms: 用于广播 AUTO 状态 <TODO: 接入串口，传给图传>
//  通用参数（静态参数）
//  - IntentResetConfig: intent_cmd_topic、intent_fb_topic
// ============================================================================

struct AutoNodeConfig : public params_utils::IntentResetConfig {
  int update_period_ms{20};
  int status_period_ms{200};
  std::string auto_status_topic{"auto_status"};

  static AutoNodeConfig load(rclcpp::Node &node) {
    AutoNodeConfig cfg;
    params_utils::IntentResetConfig::Load(node, cfg);
    params_utils::detail::declare_get_checked(
        node, "update_period_ms", cfg.update_period_ms,
        [](int v) { return v > 0; },
        "must be > 0");
    params_utils::detail::declare_get_checked(
        node, "status_period_ms", cfg.status_period_ms,
        [](int v) { return v > 0; },
        "must be > 0");
    params_utils::detail::declare_get_checked(
        node, "auto_status_topic", cfg.auto_status_topic,
        [](const std::string &v) { return !v.empty(); },
        "must not be empty");
    cfg.validate();
    return cfg;
  }

  void validate() const { params_utils::IntentResetConfig::validate(); }

  std::string summary() const {
    std::ostringstream oss;
    oss << "=============================================================================\n";
    oss << " AutoNode Configuration\n\n";
    oss << " Timing:\n";
    oss << "   - update_period_ms     : " << update_period_ms << "\n\n";
    oss << "   - status_period_ms     : " << status_period_ms << "\n\n";
    oss << " Topics:\n";
    oss << "   - auto_status_topic    : " << auto_status_topic << "\n\n";
    oss << params_utils::IntentResetConfig::summary();
    oss << "=============================================================================\n";
    return oss.str();
  }
};

} // namespace engineer_auto

#pragma once

// C++ standard library
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>

// ROS 2
#include <rclcpp/rclcpp.hpp>

// Interfaces
#include <engineer_interfaces/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <engineer_interfaces/srv/detect_energy_unit.hpp>

// Local
#include "robot_config/robot_config.hpp"
#include "auto_library/command.hpp"
#include "auto_library/execute_result.hpp"

namespace engineer_auto::vision_detect_client {

// -----------------------------------------------------------------------------
//  Types
// -----------------------------------------------------------------------------

struct VisionSpec {
  engineer_interfaces::msg::Pose pose{};
  geometry_msgs::msg::Vector3 vector{};
};

// -----------------------------------------------------------------------------
//  Config
// -----------------------------------------------------------------------------

struct VisionDetectClientConfig {
  std::string detect_service_name{"/vision/detect_energy_unit"};
  int service_wait_ms{200};

  static VisionDetectClientConfig load(rclcpp::Node &node) {
    VisionDetectClientConfig cfg;
    params_utils::detail::declare_get_checked(
        node, "vision_service_name", cfg.detect_service_name,
        [](const std::string &v) { return !v.empty(); },
        "must not be empty");
    params_utils::detail::declare_get_checked(
        node, "vision_service_wait_ms", cfg.service_wait_ms,
        [](int v) { return v >= 0; },
        "must be >= 0");
    cfg.validate();
    return cfg;
  }

  void validate() const {}

  std::string summary() const {
    std::ostringstream oss;
    oss << "=============================================================================\n";
    oss << " VisionDetectClient Configuration\n\n";
    oss << " Service:\n";
    oss << "   - vision_service_name    : " << detect_service_name << "\n";
    oss << "   - vision_service_wait_ms : " << service_wait_ms << "\n";
    oss << "=============================================================================\n";
    return oss.str();
  }
};

// -----------------------------------------------------------------------------
// VisionDetectClient
// -----------------------------------------------------------------------------

class VisionDetectClient {
public:
  explicit VisionDetectClient(rclcpp::Node &node);

  core::ExecuteResult execute(const core::Command &cmd);
  void cancel();

  std::string lastError() const { return getLastError(); }

private:
  using DetectEnergyUnit = engineer_interfaces::srv::DetectEnergyUnit;
  // 异步服务请求
  using PendingRequest = rclcpp::Client<DetectEnergyUnit>::FutureAndRequestId;

  bool buildRequest(const core::Command &cmd, bool &enable, std::string &error) const;
  bool sendRequest(bool enable);
  core::ExecuteResult consumePendingRequest(PendingRequest request);
  void setLastError(const std::string &error);
  std::string getLastError() const;

private:
  rclcpp::Node &node_;
  rclcpp::Logger logger_;
  VisionDetectClientConfig config_;
  rclcpp::Client<DetectEnergyUnit>::SharedPtr detect_client_;

  mutable std::mutex mutex_;
  std::optional<PendingRequest> pending_request_;
  std::string last_error_;
};

} // namespace engineer_auto::vision_detect_client

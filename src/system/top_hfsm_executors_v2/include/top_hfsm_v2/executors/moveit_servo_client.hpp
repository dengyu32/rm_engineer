#pragma once

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <mutex>
#include <string>
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "top_hfsm_v2/config.hpp"
#include "top_hfsm_v2/core/async_dispatcher.hpp"
#include "top_hfsm_v2/core/executor_interfaces.hpp"

namespace top_hfsm_v2::executors {

class MoveItServoClient : public top_hfsm_v2::ServoExecutor {
public:
  MoveItServoClient(rclcpp::Node &node, const MoveItServoClientConfig &config);
  ~MoveItServoClient() override;

  ServoCommandResult startServo() override;
  ServoCommandResult pauseServo() override;

private:
  void initRosInterfaces();
  void initRosServices();
  void pollAsyncResults();

  ServoCommandResult executeServoCommand(ServoCommandType command);
  ServoCommandResult doStartServo();
  ServoCommandResult doPauseServo();
  static const char *commandTag(ServoCommandType command);
  static ServoCommandType commandFromTag(const std::string &tag);
  static ServoCommandResult toServoResult(const AsyncDispatcher::Result &result);

  struct CallResult {
    ServoCommandStatus status{ServoCommandStatus::Failure};
    std::string message;
  };

  static CallResult callTriggerService(
      rclcpp::Node &node,
      const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr &client,
      const std::string &service_name,
      std::chrono::milliseconds wait_timeout,
      std::chrono::milliseconds call_timeout);
  static CallResult callEmptyService(
      rclcpp::Node &node,
      const rclcpp::Client<std_srvs::srv::Empty>::SharedPtr &client,
      const std::string &service_name,
      std::chrono::milliseconds wait_timeout,
      std::chrono::milliseconds call_timeout);

  bool startServoSync();
  bool pauseServoSync();
  bool unpauseServoSync();
  bool stopServoSync();
  bool resetServoStatusSync();

  rclcpp::Node &node_;
  MoveItServoClientConfig config_;
  rclcpp::Logger logger_;

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_servo_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr unpause_servo_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr pause_servo_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_servo_client_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr reset_status_client_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pause_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr unpause_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_status_srv_;
  rclcpp::TimerBase::SharedPtr async_poll_timer_;

  AsyncDispatcher async_dispatcher_;
  mutable std::mutex servo_mutex_;
  bool servo_pending_{false};
  bool servo_active_{false};
  bool terminal_result_ready_{false};
  ServoCommandType in_flight_command_{ServoCommandType::Pause};
  ServoCommandType terminal_result_command_{ServoCommandType::Pause};
  ServoCommandResult terminal_result_{};
};

} // namespace top_hfsm_v2::executors

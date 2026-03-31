#pragma once

// Rely

//< C++
#include <atomic>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>

//< ROS 2
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

//< Engineer Interfaces 
#include <engineer_interfaces/action/move.hpp>

//< Other Modules
#include "params_utils/param_utils.hpp"
#include "arm_solve_client/arm_types.hpp"
#include "auto_library/execute_result.hpp"
#include "auto_library/command.hpp"

namespace engineer_auto::arm_solve_client {

// ============================================================================
//  ArmSovleClientConfig:
// ----------------------------------------------------------------------------
//  私有参数（动态参数）
//  - update_period_ms: 定时器周期，用于定期检查和执行任务
//  - status_period_ms: 用于广播 AUTO 状态 <TODO: 接入串口，传给图传>
//  通用参数（静态参数）
//  - IntentResetConfig: intent_cmd_topic、intent_fb_topic
// ============================================================================

struct ArmSolveClientConfig {
  std::string action_name{"move_arm"};
  int server_wait_ms{200};

  static ArmSolveClientConfig load(rclcpp::Node &node) {
    ArmSolveClientConfig cfg;
    params_utils::detail::declare_get_checked(
        node, "arm_server_wait_ms", cfg.server_wait_ms,
        [](int v) { return v >= 0; },
        "must be >= 0");
    cfg.validate();
    return cfg;
  }

  void validate() const {}

  std::string summary() const {
    std::ostringstream oss;
    oss << "=============================================================================\n";
    oss << " ArmSolveClient Configuration\n\n";
    oss << " Action:\n";
    oss << "   - arm_action_name      : " << action_name << "\n";
    oss << "   - arm_server_wait_ms   : " << server_wait_ms << "\n";
    oss << "=============================================================================\n";
    return oss.str();
  }
};

// ============================================================================
//  ArmSovleClient: 机械臂求解客户端
// ----------------------------------------------------------------------------

// ============================================================================

class ArmSolveClient {
public:
  explicit ArmSolveClient(rclcpp::Node &node, const ArmSolveClientConfig &config);

  bool buildSpec(const core::Command &cmd,
                 ArmMoveSpec &out,
                 std::string &error) const;
  core::ExecuteResult execute(const ArmMoveSpec &spec);
  void cancel();
  std::string lastError() const;

private:

  enum class GoalPhase : uint8_t {
    None = 0,
    Pending = 1,
    Running = 2,
    Succeeded = 3,
    Failed = 4,
    Canceled = 5,
  };

  struct GoalContext {
    ArmMoveSpec request{};
    std::atomic<GoalPhase> phase{GoalPhase::Pending};
    std::atomic<bool> cancel_requested{false};
    mutable std::mutex msg_mutex;
    std::string error_msg;

    void succeed() {
      phase.store(GoalPhase::Succeeded);
      std::lock_guard<std::mutex> lock(msg_mutex);
      error_msg.clear();
    }

    void fail(const std::string &msg) {
      phase.store(GoalPhase::Failed);
      std::lock_guard<std::mutex> lock(msg_mutex);
      error_msg = msg;
    }

    void cancel() {
      phase.store(GoalPhase::Canceled);
      std::lock_guard<std::mutex> lock(msg_mutex);
      error_msg = "goal canceled";
    }

    std::string get_error() const {
      std::lock_guard<std::mutex> lock(msg_mutex);
      return error_msg;
    }
  };

  using Move = engineer_interfaces::action::Move;
  using GoalHandleMove = rclcpp_action::ClientGoalHandle<Move>;

  bool sendGoal(const ArmMoveSpec &command);

private:
  rclcpp::Node &node_;
  rclcpp::Logger logger_;
  ArmSolveClientConfig config_;
  rclcpp_action::Client<Move>::SharedPtr action_client_;

  mutable std::mutex mutex_;
  std::shared_ptr<GoalHandleMove> goal_handle_;
  std::shared_ptr<GoalContext> active_ctx_;
  std::string last_error_msg_;
  mutable std::mutex error_mutex_;
};

} // namespace engineer_auto::arm_solve_client

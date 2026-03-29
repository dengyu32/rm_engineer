#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <engineer_interfaces/action/move.hpp>

#include "params_utils/param_utils.hpp"
#include "arm_solve_client/arm_types.hpp"

namespace engineer_auto::arm_solve_client {

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

enum class CommandStatus : uint8_t {
  StartFailed = 0,
  Started = 1,
  Tracking = 2,
  Succeeded = 3,
  Failed = 4,
};

struct ExecuteResult {
  CommandStatus status;
  std::optional<std::string> error;
};

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

class ArmSolveClient {
public:
  explicit ArmSolveClient(rclcpp::Node &node, const ArmSolveClientConfig &config);

  ExecuteResult execute(const ArmMoveSpec &command);
  void cancel();
  std::string lastError() const;

private:
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

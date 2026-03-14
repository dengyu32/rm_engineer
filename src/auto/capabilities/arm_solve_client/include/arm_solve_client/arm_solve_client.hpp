#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <engineer_interfaces/action/move.hpp>

#include "task_step_library/step.hpp"
#include "params_utils/param_utils.hpp"

namespace engineer_auto::arm_solve_client {

struct ArmSolveClientConfig {
  std::string action_name{"move_arm"};
  int server_wait_ms{200};

  static ArmSolveClientConfig load(rclcpp::Node &node) {
    ArmSolveClientConfig cfg;
    params_utils::detail::declare_get_checked(
        node, "arm_action_name", cfg.action_name,
        [](const std::string &v) { return !v.empty(); },
        "must not be empty");
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

class ArmSolveClient {
public:
  explicit ArmSolveClient(rclcpp::Node &node, const ArmSolveClientConfig &config);

  CommandStatus execute(const task_step_library::ArmMoveSpec &command);
  void cancel();
  std::string lastError() const;

private:
  using Move = engineer_interfaces::action::Move;
  using GoalHandleMove = rclcpp_action::ClientGoalHandle<Move>;

  enum class GoalPhase : uint8_t {
    None = 0,
    Pending = 1,
    Running = 2,
    Succeeded = 3,
    Failed = 4,
    Canceled = 5,
  };

  struct GoalContext {
    task_step_library::ArmMoveSpec request{};
    std::atomic<GoalPhase> phase{GoalPhase::Pending};
    std::atomic<bool> cancel_requested{false};
    std::string error_msg;
  };

  bool sendGoal(const task_step_library::ArmMoveSpec &command);
  static bool sameTarget(const engineer_interfaces::msg::Pose &lhs,
                         const engineer_interfaces::msg::Pose &rhs);
  static bool sameVector(const geometry_msgs::msg::Vector3 &lhs,
                         const geometry_msgs::msg::Vector3 &rhs);
  static bool sameRequest(const task_step_library::ArmMoveSpec &lhs,
                          const task_step_library::ArmMoveSpec &rhs);

private:
  rclcpp::Node &node_;
  rclcpp::Logger logger_;
  ArmSolveClientConfig config_;
  rclcpp_action::Client<Move>::SharedPtr action_client_;

  mutable std::mutex mutex_;
  std::shared_ptr<GoalHandleMove> goal_handle_;
  std::shared_ptr<GoalContext> active_ctx_;
};

} // namespace engineer_auto::arm_solve_client

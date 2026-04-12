#pragma once

// Rely

//< C++
#include <atomic>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

//< ROS 2
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

//< Engineer Interfaces 
#include <engineer_interfaces/action/arm_move.hpp>

//< Other Modules
#include "arm_solve_client/arm_solve_client_config.hpp"
#include "arm_solve_client/arm_types.hpp"
#include "auto_library/execute_result.hpp"
#include "auto_library/command.hpp"

namespace engineer_auto::arm_solve_client {

// ============================================================================
//  ArmSovleClient: 机械臂求解客户端
// ----------------------------------------------------------------------------

// ============================================================================

class ArmSolveClient {
public:
  explicit ArmSolveClient(rclcpp::Node &node, const ArmSolveClientConfig &config);

  bool buildPoseSpec(const core::Command &cmd,
                     ArmMoveSpec &out,
                     std::string &error) const;
  bool buildJointsSpec(const core::Command &cmd,
                       ArmMoveSpec &out,
                       std::string &error) const;
  bool buildVectorSpec(const core::Command &cmd,
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

  using ArmMove = engineer_interfaces::action::ArmMove;
  using GoalHandleArmMove = rclcpp_action::ClientGoalHandle<ArmMove>;

  bool sendGoal(const ArmMoveSpec &command);

private:
  rclcpp::Node &node_;
  rclcpp::Logger logger_;
  ArmSolveClientConfig config_;
  rclcpp_action::Client<ArmMove>::SharedPtr action_client_;

  mutable std::mutex mutex_;
  std::shared_ptr<GoalHandleArmMove> goal_handle_;
  std::shared_ptr<GoalContext> active_ctx_;
  std::string last_error_msg_;
  mutable std::mutex error_mutex_;
};

} // namespace engineer_auto::arm_solve_client

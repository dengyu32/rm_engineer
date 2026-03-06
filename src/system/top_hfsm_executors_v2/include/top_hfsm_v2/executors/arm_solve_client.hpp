#pragma once

// ROS2 Action
#include <rclcpp_action/rclcpp_action.hpp>

// Error utils
#include "error_code_utils/error_bus.hpp"

// C++
#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

// ROS messages
#include <engineer_interfaces/action/move.hpp>

#include "top_hfsm_v2/config.hpp"
#include "top_hfsm_v2/core/executor_interfaces.hpp"
#include "top_hfsm_v2/types/command_types.hpp"

namespace top_hfsm_v2::executors {

// ============================================================================
//  GoalContext
// ----------------------------------------------------------------------------
//  - 记录当前 active goal 的目标、模式与状态
//  - 由 Action 回调写入进度，供查询接口读取
//  - 包含错误码/信息便于上层诊断
// ============================================================================
enum class GoalPhase : uint8_t {
  None = 0,
  Pending = 1,
  Running = 2,
  Succeeded = 3,
  Failed = 4,
  Canceled = 5,
};

struct GoalContext {
  // request
  top_hfsm_v2::ArmCommandSpec request{};

  // runtime state
  std::atomic<GoalPhase> phase{GoalPhase::Pending};
  std::atomic<bool> cancel_requested{false};
  std::atomic<float> progress{0.0f};
  std::string error_msg;
  std::atomic<int> result_code{
      static_cast<int>(rclcpp_action::ResultCode::UNKNOWN)};
};

using Move = engineer_interfaces::action::Move;
using GoalHandleMove = rclcpp_action::ClientGoalHandle<Move>;

// ============================================================================
//  ArmSolveClient
// ----------------------------------------------------------------------------
//  - Action 客户端封装，管理 Move.action 的发送/取消与状态查询
//  - 通过 active goal 上下文避免重复发送同一目标
//  - 暴露进度与上下文查询接口给 HFSM
// ============================================================================
class ArmSolveClient : public top_hfsm_v2::ArmExecutor {
public:
  // -----------------------------------------------------------------------
  //  Lifecycle
  // -----------------------------------------------------------------------
  ArmSolveClient(rclcpp::Node &node, const ArmSolveClientConfig &config);
  void set_error_bus(const std::shared_ptr<error_code_utils::ErrorBus> &bus);

  // -----------------------------------------------------------------------
  //  Goal sending
  // -----------------------------------------------------------------------
  ArmCommandStatus
  execute_arm_command(const top_hfsm_v2::ArmCommandSpec &command) override;

  // -----------------------------------------------------------------------
  //  Control
  // -----------------------------------------------------------------------
  void sendCancel() override;

  // -----------------------------------------------------------------------
  //  Query
  // -----------------------------------------------------------------------
  bool hasActiveGoal() const;
  GoalPhase goalPhase() const;
  float progress() const;
  std::string lastErrorMessage() const override;

  std::shared_ptr<const GoalContext> activeContext() const;

private:
  // -----------------------------------------------------------------------
  //  Init / utils
  // -----------------------------------------------------------------------

  // -----------------------------------------------------------------------
  //  Action helpers
  // -----------------------------------------------------------------------
  bool sendGoal(const top_hfsm_v2::ArmCommandSpec &command);
  static inline bool sameTarget(const engineer_interfaces::msg::Target &lhs,
                                const engineer_interfaces::msg::Target &rhs) {
    return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z &&
           lhs.qx == rhs.qx && lhs.qy == rhs.qy && lhs.qz == rhs.qz &&
           lhs.qw == rhs.qw;
  }

  inline bool matchesRequest(const GoalContext &ctx,
                             const top_hfsm_v2::ArmCommandSpec &command) const {
    return ctx.request.plan_option == command.plan_option &&
           sameTarget(ctx.request.pose, command.pose) &&
           ctx.request.joints == command.joints;
  }

  ArmCommandStatus processGoalRequest(const top_hfsm_v2::ArmCommandSpec &command);

private:
  // Node config
  rclcpp::Node &node_;
  ArmSolveClientConfig config_;
  rclcpp::Logger logger_;

  std::string action_name_{"move_arm"};
  int server_wait_ms_{100};

  rclcpp_action::Client<Move>::SharedPtr action_client_;

  // Active goal context
  mutable std::mutex active_mutex_;
  std::shared_ptr<GoalHandleMove> active_goal_handle_;
  std::shared_ptr<GoalContext> active_ctx_;

  void publish_error(const error_code_utils::Error &err) const;
  std::shared_ptr<error_code_utils::ErrorBus> error_bus_;
};

} // namespace top_hfsm_v2::executors

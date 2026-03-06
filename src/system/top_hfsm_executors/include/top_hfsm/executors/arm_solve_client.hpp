#pragma once

// ROS2 Action
#include <rclcpp_action/rclcpp_action.hpp>

// Error utils
#include "error_code_utils/error_bus.hpp"

// C++
#include <array>
#include <atomic>
#include <memory>
#include <mutex>
#include <string>

// ROS messages
#include <engineer_interfaces/action/move.hpp>

#include "top_hfsm/config.hpp"

namespace top_hfsm::executors {

// ============================================================================
//  PlanOptionType
// ----------------------------------------------------------------------------
//  - 规划模式选择，影响 MoveIt 规划策略
// ============================================================================
enum class PlanOptionType : uint8_t {
  NORMAL = 0,    // IK + planning
  CARTESIAN = 1, // 笛卡尔插补
  JOINTS = 2     // 关节空间直接规划
};

// ============================================================================
//  SendResultType
// ----------------------------------------------------------------------------
//  - sendOnceAndCheck 返回值，描述发送/执行状态
// ============================================================================
enum class SendResultType : uint8_t {
  SENT,         // 已发送且等待反馈
  IN_PROGRESS,  // 已接受，执行中
  SUCCEEDED,    // 执行成功
  FAILED,       // 执行失败
  SEND_FAILED   // 发送阶段失败
};

// ============================================================================
//  GoalContext
// ----------------------------------------------------------------------------
//  - 记录当前 active goal 的目标、模式与状态
//  - 由 Action 回调写入进度，供查询接口读取
//  - 包含错误码/信息便于上层诊断
// ============================================================================
struct GoalContext {
  PlanOptionType option{PlanOptionType::NORMAL};
  engineer_interfaces::msg::Target target_pose{};
  std::array<float, 6> target_joints{{0.f, 0.f, 0.f, 0.f, 0.f, 0.f}};

  std::atomic<bool> accepted{false};
  std::atomic<bool> done{false};
  std::atomic<bool> success{false};
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
//  - 通过 sendOnce 闩锁避免重复发送同一目标
//  - 暴露进度与上下文查询接口给 HFSM
// ============================================================================
class ArmSolveClient {
public:
  // -----------------------------------------------------------------------
  //  Lifecycle
  // -----------------------------------------------------------------------
  ArmSolveClient(rclcpp::Node &node, const ArmSolveClientConfig &config);
  void set_error_bus(const std::shared_ptr<error_code_utils::ErrorBus> &bus);

  // -----------------------------------------------------------------------
  //  Goal sending
  // -----------------------------------------------------------------------
  SendResultType
  sentArmGoal(const std::array<float, 6> &target_joints,
              PlanOptionType option = PlanOptionType::JOINTS);

  SendResultType
  sentArmGoal(const engineer_interfaces::msg::Target &target_pose,
               PlanOptionType option);

  // -----------------------------------------------------------------------
  //  Control
  // -----------------------------------------------------------------------
  void resetSendOnce();
  void sendCancel();

  // -----------------------------------------------------------------------
  //  Query
  // -----------------------------------------------------------------------
  bool hasActiveGoal() const;
  bool accepted() const;
  bool done() const;
  bool success() const;
  float progress() const;

  std::shared_ptr<const GoalContext> activeContext() const;

private:
  // -----------------------------------------------------------------------
  //  Init / utils
  // -----------------------------------------------------------------------

  // -----------------------------------------------------------------------
  //  Action helpers
  // -----------------------------------------------------------------------
  bool sendGoal(const engineer_interfaces::msg::Target &target_pose,
                const std::array<float, 6> &target_joints,
                PlanOptionType option);

  SendResultType
  sendOnceAndCheck(const engineer_interfaces::msg::Target &target_pose,
                   const std::array<float, 6> &target_joints,
                   PlanOptionType option);

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

  // sendOnce 闩锁（策略状态）
  std::atomic_bool send_once_latched_{false};

  // cancel 在 accepted 前的缓存
  std::atomic_bool pending_cancel_{false};

  void publish_error(const error_code_utils::Error &err) const;
  std::shared_ptr<error_code_utils::ErrorBus> error_bus_;
};

} // namespace top_hfsm::executors

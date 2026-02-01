// Executors
#include "top_hfsm/executors/arm_solve_client.hpp"
#include "top_hfsm/error_code.hpp"
#include "param_utils/param_snapshot.hpp"

// C++
#include <chrono>

namespace top_hfsm::executors {

// ============================================================================
// ctor
// ============================================================================

ArmSolveClient::ArmSolveClient(rclcpp::Node &node, const ArmSolveClientConfig &config)
    : node_(node),
      config_(config),
      logger_(node.get_logger()),
      action_name_(config.arm_action_name),
      server_wait_ms_(config.arm_server_wait_ms) {
  action_client_ = rclcpp_action::create_client<Move>(&node_, action_name_);
  param_utils::LogSnapshot(node_.get_logger(), config_.params_snapshot,
                           "[ARM_SOLVE_CLIENT][param] ");
  RCLCPP_INFO(logger_,
              "[ARM_SOLVE_CLIENT] started (action=%s, wait_ms=%d)",
              action_name_.c_str(), server_wait_ms_);
}

void ArmSolveClient::set_error_bus(const std::shared_ptr<error_code_utils::ErrorBus> &bus) {
  error_bus_ = bus;
}

void ArmSolveClient::publish_error(const error_code_utils::Error &err) const {
  if (!error_bus_) {
    return;
  }
  error_bus_->publish(err);
}

// ============================================================================
//  Init / utils
// ============================================================================

// ============================================================================
//  Goal sending
// ============================================================================

bool ArmSolveClient::sendGoal(const engineer_interfaces::msg::Target &target_pose,
                              const std::array<float, 6> &target_joints,
                              PlanOptionType option) {
  auto ctx = std::make_shared<GoalContext>();
  ctx->option = option;
  ctx->target_pose = target_pose;
  ctx->target_joints = target_joints;
  ctx->error_msg.clear();

  auto fail_fast = [this, &ctx](const char *msg) {
    ctx->done.store(true);
    ctx->success.store(false);
    ctx->error_msg = msg ? msg : "";
    {
      std::scoped_lock lock(active_mutex_);
      active_ctx_ = ctx;
      active_goal_handle_.reset();
    }
  };

  if (!action_client_) {
    fail_fast("Action client not created");
    publish_error(make_error(ArmSolveClientErrc::ActionClientMissing,
                             "Action client not created"));
    return false;
  }

  // 等待 action server ready，避免立即返回失败
  if (!action_client_->wait_for_action_server(
          std::chrono::milliseconds(server_wait_ms_))) {
    RCLCPP_ERROR(node_.get_logger(),
                 "[scope=arm_solve_client][status=failed] Action server not available");
    fail_fast("Action server not available");
    publish_error(make_error(ArmSolveClientErrc::ActionServerUnavailable,
                             "Action server not available",
                             {{"action_name", action_name_}}));
    return false;
  }

  // 清空前一次 cancel 标记，准备新的 goal
  pending_cancel_.store(false, std::memory_order_relaxed); // 此时没有收到取消的请求

  {
    std::scoped_lock lock(active_mutex_);
    active_ctx_ = ctx;
    active_goal_handle_.reset();
  }

  Move::Goal goal;
  goal.option_id = static_cast<uint8_t>(option);
  goal.target_pose = target_pose;
  goal.target_joints = target_joints;

  rclcpp_action::Client<Move>::SendGoalOptions opts;

  opts.goal_response_callback = [this,
                                 ctx](std::shared_ptr<GoalHandleMove> gh) {
    if (!gh) {
      // 目标被拒绝，直接标记完成并填充错误
      ctx->done.store(true);
      ctx->success.store(false);
      ctx->error_msg = "Goal rejected by server";
      publish_error(make_error(ArmSolveClientErrc::GoalRejected, "Goal rejected by server",
                               {{"action_name", action_name_}}));
      return;
    }

    {
      std::scoped_lock lock(active_mutex_);
      if (active_ctx_ == ctx)
        active_goal_handle_ = gh;
    }

    ctx->accepted.store(true);

    // 若在等待阶段收到 cancel 请求，则在 accepted 后立即 cancel
    if (pending_cancel_.load()) {
      action_client_->async_cancel_goal(gh);
    }
  };

  opts.feedback_callback =
      [ctx](std::shared_ptr<GoalHandleMove>,
            const std::shared_ptr<const Move::Feedback> fb) {
        if (fb)
          ctx->progress.store(fb->progress);
      };

  opts.result_callback = [this,
                          ctx](const GoalHandleMove::WrappedResult &result) {
    ctx->done.store(true);
    ctx->result_code.store(static_cast<int>(result.code));

    const bool ok = result.code == rclcpp_action::ResultCode::SUCCEEDED &&
                    result.result && result.result->success;
    if (result.result) {
      ctx->error_msg = result.result->error_msg;
    } else {
      ctx->error_msg = "Empty result";
      publish_error(make_error(ArmSolveClientErrc::EmptyResult, "Empty result",
                               {{"action_name", action_name_}}));
    }
    ctx->success.store(ok);

    {
      std::scoped_lock lock(active_mutex_);
      if (active_ctx_ == ctx)
        active_goal_handle_.reset();
    }
  };

  action_client_->async_send_goal(goal, opts);
  return true;
}

// ============================================================================
//  Control
// ============================================================================

void ArmSolveClient::sendCancel() {
  // 先记录 cancel 请求，若还未 accepted 则 goal_response 中再触发
  pending_cancel_.store(true);

  std::shared_ptr<GoalHandleMove> gh;
  {
    std::scoped_lock lock(active_mutex_);
    gh = active_goal_handle_;
  }

  if (gh)
    action_client_->async_cancel_goal(gh);
}

// ============================================================================
//  Query
// ============================================================================

bool ArmSolveClient::hasActiveGoal() const {
  std::scoped_lock lock(active_mutex_);
  return active_ctx_ && !active_ctx_->done.load();
}

bool ArmSolveClient::accepted() const {
  std::scoped_lock lock(active_mutex_);
  return active_ctx_ && active_ctx_->accepted.load();
}

bool ArmSolveClient::done() const {
  std::scoped_lock lock(active_mutex_);
  return !active_ctx_ || active_ctx_->done.load();
}

bool ArmSolveClient::success() const {
  std::scoped_lock lock(active_mutex_);
  return active_ctx_ && active_ctx_->success.load();
}

float ArmSolveClient::progress() const {
  std::scoped_lock lock(active_mutex_);
  return active_ctx_ ? active_ctx_->progress.load() : 0.0f;
}

std::shared_ptr<const GoalContext> ArmSolveClient::activeContext() const {
  std::scoped_lock lock(active_mutex_);
  return active_ctx_;
}

// ============================================================================
//  Action helpers
// ============================================================================

void ArmSolveClient::resetSendOnce() { send_once_latched_.store(false); }

SendResultType
ArmSolveClient::sentArmGoal(const engineer_interfaces::msg::Target &target_pose,
                            PlanOptionType option) {

  static const std::array<float, 6> kZeroJoints{{0.f, 0.f, 0.f, 0.f, 0.f, 0.f}};
  return sendOnceAndCheck(target_pose, kZeroJoints, option);
}

SendResultType
ArmSolveClient::sentArmGoal(const std::array<float, 6> &target_joints,
                            PlanOptionType option) {

  engineer_interfaces::msg::Target empty_pose{}; // 默认零初始化
  return sendOnceAndCheck(empty_pose, target_joints, option);
}

SendResultType
ArmSolveClient::sendOnceAndCheck(const engineer_interfaces::msg::Target &target_pose,
                                 const std::array<float, 6> &target_joints,
                                 PlanOptionType option) {

  bool expected = false;
  // sendOnce 闩锁：只有首次调用成功发送，后续返回执行状态
  if (send_once_latched_.compare_exchange_strong(expected, true)) {
    if (!sendGoal(target_pose, target_joints, option)) {
      send_once_latched_.store(false);
      return SendResultType::SEND_FAILED;
    }
    return SendResultType::SENT;
  }

  if (!done())
    return SendResultType::IN_PROGRESS;
  return success() ? SendResultType::SUCCEEDED : SendResultType::FAILED;
}

} // namespace top_hfsm::executors

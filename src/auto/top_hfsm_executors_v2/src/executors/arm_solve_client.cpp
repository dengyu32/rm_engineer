// Executors
#include "top_hfsm_v2/executors/arm_solve_client.hpp"
#include "error_code_utils/app_error.hpp"

// C++
#include <chrono>

namespace top_hfsm_v2::executors {

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

bool ArmSolveClient::sendGoal(const top_hfsm_v2::ArmCommandSpec &command) {
  auto ctx = std::make_shared<GoalContext>();
  ctx->request = command;
  ctx->error_msg.clear();

  auto fail_fast = [this, &ctx](const char *msg) {
    ctx->phase.store(GoalPhase::Failed);
    ctx->error_msg = msg ? msg : "";
    {
      std::scoped_lock lock(active_mutex_);
      active_ctx_ = ctx;
      active_goal_handle_.reset();
    }
  };

  if (!action_client_) {
    fail_fast("Action client not created");
    publish_error(error_code_utils::app::make_app_error(error_code_utils::ErrorDomain::SOLVE, error_code_utils::app::SolveCode::ActionClientMissing,
                             "Action client not created"));
    return false;
  }

  // 等待 action server ready，避免立即返回失败
  if (!action_client_->wait_for_action_server(
          std::chrono::milliseconds(server_wait_ms_))) {
    RCLCPP_ERROR(node_.get_logger(),
                 "[scope=arm_solve_client][status=failed] Action server not available");
    fail_fast("Action server not available");
    publish_error(error_code_utils::app::make_app_error(error_code_utils::ErrorDomain::SOLVE, error_code_utils::app::SolveCode::ActionServerUnavailable,
                             "Action server not available",
                             {{"action_name", action_name_}}));
    return false;
  }

  {
    std::scoped_lock lock(active_mutex_);
    active_ctx_ = ctx;
    active_goal_handle_.reset();
  }

  Move::Goal goal;
  goal.option_id = static_cast<uint8_t>(command.plan_option);
  goal.target_pose = command.pose;
  goal.target_joints = command.joints;

  rclcpp_action::Client<Move>::SendGoalOptions opts;

  opts.goal_response_callback = [this,
                                 ctx](std::shared_ptr<GoalHandleMove> gh) {
    if (!gh) {
      // 目标被拒绝，直接标记完成并填充错误
      ctx->phase.store(GoalPhase::Failed);
      ctx->error_msg = "Goal rejected by server";
      publish_error(error_code_utils::app::make_app_error(error_code_utils::ErrorDomain::SOLVE, error_code_utils::app::SolveCode::GoalRejected, "Goal rejected by server",
                               {{"action_name", action_name_}}));
      return;
    }

    {
      std::scoped_lock lock(active_mutex_);
      if (active_ctx_ == ctx)
        active_goal_handle_ = gh;
    }

    ctx->phase.store(GoalPhase::Running);

    // 若在等待阶段收到 cancel 请求，则在 accepted 后立即 cancel
    if (ctx->cancel_requested.load()) {
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
    ctx->result_code.store(static_cast<int>(result.code));
    if (result.result) {
      ctx->error_msg = result.result->error_msg;
    } else {
      ctx->error_msg = "Empty result";
      publish_error(error_code_utils::app::make_app_error(error_code_utils::ErrorDomain::SOLVE, error_code_utils::app::SolveCode::EmptyResult, "Empty result",
                               {{"action_name", action_name_}}));
    }

    const bool ok = result.code == rclcpp_action::ResultCode::SUCCEEDED &&
                    result.result && result.result->success;
    if (ok) {
      ctx->phase.store(GoalPhase::Succeeded);
    } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
      ctx->phase.store(GoalPhase::Canceled);
    } else {
      ctx->phase.store(GoalPhase::Failed);
    }

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
  std::shared_ptr<GoalContext> ctx;
  std::shared_ptr<GoalHandleMove> gh;
  {
    std::scoped_lock lock(active_mutex_);
    ctx = active_ctx_;
    gh = active_goal_handle_;
  }

  if (ctx) {
    ctx->cancel_requested.store(true);
  }

  if (gh) {
    action_client_->async_cancel_goal(gh);
  }
}

// ============================================================================
//  Query
// ============================================================================

bool ArmSolveClient::hasActiveGoal() const {
  std::scoped_lock lock(active_mutex_);
  if (!active_ctx_) {
    return false;
  }

  const GoalPhase phase = active_ctx_->phase.load();
  return phase == GoalPhase::Pending || phase == GoalPhase::Running;
}

GoalPhase ArmSolveClient::goalPhase() const {
  std::scoped_lock lock(active_mutex_);
  if (!active_ctx_) {
    return GoalPhase::None;
  }

  return active_ctx_->phase.load();
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

ArmCommandStatus ArmSolveClient::execute_arm_command(
    const top_hfsm_v2::ArmCommandSpec &command) {
  return processGoalRequest(command);
}

ArmCommandStatus ArmSolveClient::processGoalRequest(
    const top_hfsm_v2::ArmCommandSpec &command) {
  std::shared_ptr<GoalContext> ctx;
  std::shared_ptr<GoalHandleMove> gh;
  {
    std::scoped_lock lock(active_mutex_);
    ctx = active_ctx_;
    gh = active_goal_handle_;
  }

  if (ctx) {
    if (matchesRequest(*ctx, command)) {
        const GoalPhase phase = ctx->phase.load();
        if (phase == GoalPhase::Pending || phase == GoalPhase::Running) {
        return ArmCommandStatus::TRACKING;
      }

      const ArmCommandStatus final_result =
          phase == GoalPhase::Succeeded ? ArmCommandStatus::SUCCEEDED
                                        : ArmCommandStatus::FAILED;
      {
        std::scoped_lock lock(active_mutex_);
        if (active_ctx_ == ctx) {
          active_ctx_.reset();
          active_goal_handle_.reset();
        }
      }
      return final_result;
    }

    ctx->cancel_requested.store(true);
    if (gh) {
      action_client_->async_cancel_goal(gh);
    }
    {
      std::scoped_lock lock(active_mutex_);
      if (active_ctx_ == ctx) {
        active_ctx_.reset();
        active_goal_handle_.reset();
      }
    }
  }

  if (!sendGoal(command)) {
    return ArmCommandStatus::START_FAILED;
  }
  return ArmCommandStatus::STARTED;
}

std::string ArmSolveClient::lastErrorMessage() const {
  std::scoped_lock lock(active_mutex_);
  return active_ctx_ ? active_ctx_->error_msg : std::string();
}

} // namespace top_hfsm_v2::executors

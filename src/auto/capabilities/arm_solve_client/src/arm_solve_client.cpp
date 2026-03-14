#include "arm_solve_client/arm_solve_client.hpp"

#include <chrono>

namespace engineer_auto::arm_solve_client {
using namespace task_step_library;

ArmSolveClient::ArmSolveClient(rclcpp::Node &node,
                               const ArmSolveClientConfig &config)
    : node_(node), logger_(node.get_logger()), config_(config) {
  RCLCPP_INFO(logger_, "\n%s", config_.summary().c_str());
  action_client_ = rclcpp_action::create_client<Move>(&node_, config_.action_name);
}

bool ArmSolveClient::sameTarget(const engineer_interfaces::msg::Pose &lhs,
                                const engineer_interfaces::msg::Pose &rhs) {
  return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z && lhs.qx == rhs.qx &&
         lhs.qy == rhs.qy && lhs.qz == rhs.qz && lhs.qw == rhs.qw;
}

bool ArmSolveClient::sameVector(const geometry_msgs::msg::Vector3 &lhs,
                                const geometry_msgs::msg::Vector3 &rhs) {
  return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z;
}

bool ArmSolveClient::sameRequest(const ArmMoveSpec &lhs, const ArmMoveSpec &rhs) {
  return lhs.plan_option == rhs.plan_option && lhs.joints == rhs.joints &&
         sameTarget(lhs.pose, rhs.pose) && sameVector(lhs.vector, rhs.vector);
}

bool ArmSolveClient::sendGoal(const ArmMoveSpec &command) {
  auto ctx = std::make_shared<GoalContext>();
  ctx->request = command;

  if (!action_client_) {
    ctx->phase.store(GoalPhase::Failed);
    ctx->error_msg = "action client not created";
    std::scoped_lock lock(mutex_);
    active_ctx_ = ctx;
    goal_handle_.reset();
    return false;
  }

  if (!action_client_->wait_for_action_server(
          std::chrono::milliseconds(config_.server_wait_ms))) {
    ctx->phase.store(GoalPhase::Failed);
    ctx->error_msg = "action server not available";
    std::scoped_lock lock(mutex_);
    active_ctx_ = ctx;
    goal_handle_.reset();
    return false;
  }

  {
    std::scoped_lock lock(mutex_);
    active_ctx_ = ctx;
    goal_handle_.reset();
  }

  Move::Goal goal;
  goal.option_id = static_cast<uint8_t>(command.plan_option);
  goal.target_pose = command.pose;
  goal.target_joints = command.joints;
  goal.target_vector = command.vector;

  rclcpp_action::Client<Move>::SendGoalOptions opts;
  opts.goal_response_callback = [this, ctx](std::shared_ptr<GoalHandleMove> gh) {
    if (!gh) {
      ctx->phase.store(GoalPhase::Failed);
      ctx->error_msg = "goal rejected";
      return;
    }
    {
      std::scoped_lock lock(mutex_);
      if (active_ctx_ == ctx) {
        goal_handle_ = gh;
      }
    }
    ctx->phase.store(GoalPhase::Running);
    if (ctx->cancel_requested.load()) {
      action_client_->async_cancel_goal(gh);
    }
  };

  opts.result_callback = [this, ctx](const GoalHandleMove::WrappedResult &result) {
    if (!result.result) {
      ctx->phase.store(GoalPhase::Failed);
      ctx->error_msg = "empty result";
    } else if (result.code == rclcpp_action::ResultCode::SUCCEEDED &&
               result.result->success) {
      ctx->phase.store(GoalPhase::Succeeded);
      ctx->error_msg.clear();
    } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
      ctx->phase.store(GoalPhase::Canceled);
      ctx->error_msg = "goal canceled";
    } else {
      ctx->phase.store(GoalPhase::Failed);
      ctx->error_msg = result.result->error_msg;
      if (ctx->error_msg.empty()) {
        ctx->error_msg = "goal failed";
      }
    }

    std::scoped_lock lock(mutex_);
    if (active_ctx_ == ctx) {
      goal_handle_.reset();
    }
  };

  action_client_->async_send_goal(goal, opts);
  return true;
}

CommandStatus ArmSolveClient::execute(const ArmMoveSpec &command) {
  std::shared_ptr<GoalContext> ctx;
  std::shared_ptr<GoalHandleMove> gh;
  {
    std::scoped_lock lock(mutex_);
    ctx = active_ctx_;
    gh = goal_handle_;
  }

  if (ctx) {
    if (sameRequest(ctx->request, command)) {
      const GoalPhase phase = ctx->phase.load();
      if (phase == GoalPhase::Pending || phase == GoalPhase::Running) {
        return CommandStatus::Tracking;
      }
      CommandStatus final_status = CommandStatus::Failed;
      if (phase == GoalPhase::Succeeded) {
        final_status = CommandStatus::Succeeded;
      }
      {
        std::scoped_lock lock(mutex_);
        if (active_ctx_ == ctx) {
          active_ctx_.reset();
          goal_handle_.reset();
        }
      }
      return final_status;
    }

    ctx->cancel_requested.store(true);
    if (gh) {
      action_client_->async_cancel_goal(gh);
    }
    std::scoped_lock lock(mutex_);
    if (active_ctx_ == ctx) {
      active_ctx_.reset();
      goal_handle_.reset();
    }
  }

  if (!sendGoal(command)) {
    return CommandStatus::StartFailed;
  }
  return CommandStatus::Started;
}

void ArmSolveClient::cancel() {
  std::shared_ptr<GoalContext> ctx;
  std::shared_ptr<GoalHandleMove> gh;
  {
    std::scoped_lock lock(mutex_);
    ctx = active_ctx_;
    gh = goal_handle_;
  }
  if (ctx) {
    ctx->cancel_requested.store(true);
  }
  if (gh) {
    action_client_->async_cancel_goal(gh);
  }
}

std::string ArmSolveClient::lastError() const {
  std::scoped_lock lock(mutex_);
  return active_ctx_ ? active_ctx_->error_msg : std::string();
}

} // namespace engineer_auto::arm_solve_client

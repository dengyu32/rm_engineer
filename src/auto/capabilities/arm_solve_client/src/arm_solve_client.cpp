#include "arm_solve_client/arm_solve_client.hpp"

#include <chrono>
#include <mutex>
#include <rclcpp_action/client.hpp>
#include <rcutils/error_handling.h>

#include "step_executor/types/command.hpp"

namespace engineer_auto::arm_solve_client {

// ============================================================================
//  匿名空间
// ============================================================================

namespace {
bool sameTarget(const engineer_interfaces::msg::Pose &lhs,
                                const engineer_interfaces::msg::Pose &rhs) {
  return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z && lhs.qx == rhs.qx &&
         lhs.qy == rhs.qy && lhs.qz == rhs.qz && lhs.qw == rhs.qw;
}

bool sameVector(const geometry_msgs::msg::Vector3 &lhs,
                                const geometry_msgs::msg::Vector3 &rhs) {
  return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z;
}

bool sameRequest(const ArmMoveSpec &lhs, const ArmMoveSpec &rhs) {
  return lhs.plan_option == rhs.plan_option && lhs.joints == rhs.joints &&
         sameTarget(lhs.pose, rhs.pose) && sameVector(lhs.vector, rhs.vector);
}
}  

// ============================================================================
//  CTOR
// ============================================================================

ArmSolveClient::ArmSolveClient(rclcpp::Node &node,
                               const ArmSolveClientConfig &config)
    : node_(node), logger_(node.get_logger()), config_(config) {
  RCLCPP_INFO(logger_, "\n%s", config_.summary().c_str());
  action_client_ = rclcpp_action::create_client<Move>(&node_, config_.action_name);
}

// ============================================================================
//  buildRequest -- 解析 Command 参数，生成 ArmMoveSpec
// ============================================================================

bool ArmSolveClient::buildRequest(const step_executor::Command &cmd,
                                  ArmMoveSpec &out,
                                  std::string &error) const {
  if (const auto *pose =
          step_executor::paramAs<std::array<double, 7>>(cmd, "target_pose")) {
    out.plan_option = PlanOption::NORMAL;
    out.pose.x = (*pose)[0];
    out.pose.y = (*pose)[1];
    out.pose.z = (*pose)[2];
    out.pose.qx = (*pose)[3];
    out.pose.qy = (*pose)[4];
    out.pose.qz = (*pose)[5];
    out.pose.qw = (*pose)[6];
    error.clear();
    return true;
  }
  if (const auto *joints =
          step_executor::paramAs<std::array<float, 6>>(cmd, "target_joints")) {
    out.plan_option = PlanOption::JOINTS;
    out.joints = *joints;
    error.clear();
    return true;
  }
  if (const auto *vec =
          step_executor::paramAs<std::array<double, 3>>(cmd, "target_vector")) {
    out.plan_option = PlanOption::CARTESIAN;
    out.vector.x = (*vec)[0];
    out.vector.y = (*vec)[1];
    out.vector.z = (*vec)[2];
    error.clear();
    return true;
  }

  error = "arm command missing target";
  return false;
}

// ============================================================================
//  EXECUTE -- 核心函数，该能力层提供的对外接口，表示执行并跟进 GOAL 状态
// ----------------------------------------------------------------------------
//  相当于轮询状态机
// ============================================================================

ExecuteResult ArmSolveClient::execute(const step_executor::Command &cmd) {
  ArmMoveSpec command{};
  std::string build_error;
  if (!buildRequest(cmd, command, build_error)) {
    return {step_executor::ExecuteStatus::Failed,
            step_executor::ErrorCode::ValidationError,
            build_error};
  }

  std::shared_ptr<GoalContext> ctx;
  std::shared_ptr<GoalHandleMove> gh;
  // 拷贝共享数据
  {
    std::scoped_lock lock(mutex_);
    ctx = active_ctx_;
    gh = goal_handle_;
  }

  // 未执行任务
  if (!ctx) {
    if (sendGoal(command)) {
      return {step_executor::ExecuteStatus::Running,
              step_executor::ErrorCode::Unknown,
              std::nullopt};
    } else {
      return {step_executor::ExecuteStatus::Failed,
              step_executor::ErrorCode::ExecutionError,
              lastError()};
    }
  }

  // 执行任务中，且同时是同一个请求
  if (sameRequest(ctx->request, command)) {
    const auto phase = ctx->phase.load();
    // TO REQUEST: None 的归属问题
    switch (phase) {
      case GoalPhase::Pending:
      case GoalPhase::Running:
        return {step_executor::ExecuteStatus::Running,
                step_executor::ErrorCode::Unknown,
                std::nullopt};

      case GoalPhase::Succeeded:
        // 清空 active_ctx_
        {
          std::scoped_lock lock(mutex_);
          if (active_ctx_ == ctx) {
            active_ctx_.reset();
            goal_handle_.reset();
          } 
        }
        return {step_executor::ExecuteStatus::Succeeded,
                step_executor::ErrorCode::Unknown,
                std::nullopt};

      case GoalPhase::Failed:
      case GoalPhase::Canceled:
        // 清空 active_ctx_
        {
          std::scoped_lock lock(mutex_);
          if (active_ctx_ == ctx) {
            active_ctx_.reset();
            goal_handle_.reset();
          } 
        }
        return {step_executor::ExecuteStatus::Failed,
                step_executor::ErrorCode::ExecutionError,
                lastError()};
      
      default:
        return {step_executor::ExecuteStatus::Failed,
                step_executor::ErrorCode::Unknown,
                "unknown phase state"};
    }
  }

  // 上述情况均不成立，表明有新请求出现，需要取消旧任务
  if (ctx) {

    ctx->cancel_requested.store(true);
    if (gh) {
      action_client_->async_cancel_goal(gh);
    }
    // 旧任务丢弃
    {
      std::scoped_lock lock(mutex_);
      if (active_ctx_ == ctx) {
        active_ctx_.reset();
        goal_handle_.reset();
      }
    }
  }

  // 启动新任务
  if (sendGoal(command)) {
    return {step_executor::ExecuteStatus::Running,
            step_executor::ErrorCode::Unknown,
            std::nullopt};
  } else {
    return {step_executor::ExecuteStatus::Failed,
            step_executor::ErrorCode::ExecutionError,
            lastError()};
  }
}

// ============================================================================
//  sendGoal -- 表示向服务端发送请求，同时修改 ctx 中的 GoalPhase
// ----------------------------------------------------------------------------
//  返回值：true 一定进入生命周期，构建 active_ctx_ , 用于 execute 状态机
//        false 一定没有 active_ctx_ , 在 execute 直接返回 StartFailed
//  回调：goal_response_callback
//       result_callback
// ============================================================================

bool ArmSolveClient::sendGoal(const ArmMoveSpec &command) {
  // 发送前，清空旧的错误记录
  {
    std::scoped_lock lock(error_mutex_);
    last_error_msg_.clear();
  }

  // 初始化
  auto ctx = std::make_shared<GoalContext>();
  ctx->request = command;
  ctx->phase.store(GoalPhase::Pending);

  // 前置检查
  if (!action_client_) {
    ctx->fail("action client not created");
    {
      std::scoped_lock lock(error_mutex_);
      last_error_msg_ = "action client not created";
    }
    return false;
  }

  if (!action_client_->wait_for_action_server(
          std::chrono::milliseconds(config_.server_wait_ms))) {
    ctx->fail("acition server not available");
    {
      std::scoped_lock lock(error_mutex_);
      last_error_msg_ = "acition server not available";
    }
    return false;
  }

  // 激活当前ctx 这样做会直接替换掉旧的 active_ctx_
  {
    std::scoped_lock lock(mutex_);
    active_ctx_ = ctx;
    goal_handle_.reset();
  }

  // 构造 goal
  Move::Goal goal;
  goal.option_id = static_cast<uint8_t>(command.plan_option);
  goal.target_pose = command.pose;
  goal.target_joints = command.joints;
  goal.target_vector = command.vector;

  // 回调
  rclcpp_action::Client<Move>::SendGoalOptions opts;

  opts.goal_response_callback = [this, ctx](std::shared_ptr<GoalHandleMove> gh) {
    if (!gh) {
      ctx->fail("goal rejected");
      {
        std::scoped_lock lock(error_mutex_);
        last_error_msg_ = "goal rejected";
      }
      return;
    }
    // 更新 goal_handle_
    {
      std::scoped_lock lock(mutex_);
      if (active_ctx_ == ctx) {
        goal_handle_ = gh;
      }
    }
    ctx->phase.store(GoalPhase::Running);
     // 检查是否取消
    if (ctx->cancel_requested.load()) {
      action_client_->async_cancel_goal(gh);
    }
  };

  opts.result_callback = [this, ctx](const GoalHandleMove::WrappedResult &result) {
    // 错误获取 lamada
    auto get_error = [&]() {
      return (result.result && !result.result->error_msg.empty())
             ? result.result->error_msg : "goal failed (unknown)";
    };

    // 先获取当前错误，并更新 last_error_msg_
    // 防止 ctx 设置为 failed 瞬间，execute 轮询到但获取不到最新 last_error_msg
    std::string current_error;
    if (result.code == rclcpp_action::ResultCode::CANCELED) {
      current_error = "goal canceled";
    } else if (result.code != rclcpp_action::ResultCode::SUCCEEDED || 
              (result.result && !result.result->success)) {
      current_error = get_error();
    }

    {
      std::scoped_lock lock(error_mutex_);
      last_error_msg_ = current_error;
    }

    // 处理 result code
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        if (result.result && result.result->success) {
          ctx->succeed();
        } else {
          ctx->fail(get_error());
        }
        break;
      case rclcpp_action::ResultCode::CANCELED:
        ctx->cancel();
        break;
      default:
        ctx->fail(current_error);
        break;
    }

    // 清除句柄
    // 这里只重置 goal_handle_ 而不重置 active_ctx_
    // 是因为之后 execute() 还需要通过 active_ctx_ 来获取最终的执行状态
    std::scoped_lock lock(mutex_);
    if (active_ctx_ == ctx) {
      goal_handle_.reset();
    }
  };

  // 异步发送goal
  action_client_->async_send_goal(goal, opts);
  return true;
}

// ============================================================================
//  外置接口
// ============================================================================

void ArmSolveClient::cancel() {
  std::shared_ptr<GoalContext> ctx;
  std::shared_ptr<GoalHandleMove> gh;
  // 拷贝共享数据
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
  std::scoped_lock lock(error_mutex_);
  return last_error_msg_;
}

} // namespace engineer_auto::arm_solve_client

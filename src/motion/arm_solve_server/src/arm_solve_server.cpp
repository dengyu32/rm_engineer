#include "arm_solve_server.hpp"

#include <chrono>
#include <exception>
#include <memory>
#include <thread>

#include <rclcpp/node_options.hpp>

namespace arm_solve {

using namespace std::chrono_literals;

namespace {

// 匿名空间

std::shared_ptr<Move::Result> make_move_result(bool success,
                                               const std::string &msg) {
  auto result = std::make_shared<Move::Result>();
  result->success = success;
  result->error_msg = msg;
  return result;
}

void finish_move_goal(const std::shared_ptr<GoalHandleMove> &gh, bool success,
                      bool canceled, const std::string &msg) {
  if (!gh) {
    return;
  }
  auto result = make_move_result(success, msg);
  if (success) {
    gh->succeed(result);
    return;
  }
  if (canceled) {
    gh->canceled(result);
    return;
  }
  gh->abort(result);
}
} // namespace

ArmSolveServer::ArmSolveServer(const rclcpp::NodeOptions &options)
    : Node("arm_solve_action_server", rclcpp::NodeOptions(options)),
      config_(ArmSolveConfig::Load(*this)),
      solve_core_config_(solve_core::SolveCoreConfig{}) {
  solve_core_config_.validate();

  solve_executor_ = std::make_unique<solve_executor::SolveExecutor>(
      *this, solve_executor::makeSolveExecutorConfig(config_),
      solve_core_config_);

  init_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(config_.late_init_delay_ms),
      [this]() {
        if (init_timer_) {
          init_timer_->cancel();
        }
        auto self = std::static_pointer_cast<rclcpp::Node>(shared_from_this());
        solve_executor_->lateInit(self);
      });

  joint_states_verbose_sub_ =
      this->create_subscription<engineer_interfaces::msg::Joints>(
          config_.joint_states_verbose_topic, rclcpp::QoS(10),
          std::bind(&ArmSolveServer::jointCallBack, this,
                    std::placeholders::_1));

  joint_cmd_pub_ = this->create_publisher<engineer_interfaces::msg::Joints>(
      config_.joint_cmd_topic, rclcpp::QoS(10));

  action_server_ = rclcpp_action::create_server<Move>(
      this, config_.arm_action_name,
      std::bind(&ArmSolveServer::handle_goal, this, std::placeholders::_1,
                std::placeholders::_2),
      std::bind(&ArmSolveServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&ArmSolveServer::handle_accepted, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "[ARM_SOLVE_SERVER] started");
  RCLCPP_INFO(this->get_logger(), "\n%s", config_.summary().c_str());
  RCLCPP_INFO(this->get_logger(), "\n%s", solve_core_config_.summary().c_str());
}

void ArmSolveServer::jointCallBack(
    const engineer_interfaces::msg::Joints::SharedPtr msg) {
  std::scoped_lock<std::mutex> lock(current_joints_mutex_);
  current_joints_ = *msg;
}

rclcpp_action::GoalResponse
ArmSolveServer::handle_goal(const rclcpp_action::GoalUUID &uuid,
                            std::shared_ptr<const Move::Goal> goal) {
  (void)uuid;
  RCLCPP_INFO(get_logger(),
              "[arm_solve_server] Received goal: option_id=%u",
              goal->option_id);

  if (!solve_executor_ || !solve_executor_->isReady()) {
    RCLCPP_WARN(get_logger(), "[arm_solve_server] SolveExecutor not ready");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
ArmSolveServer::handle_cancel(const std::shared_ptr<GoalHandleMove> gh) {
  RCLCPP_INFO(get_logger(), "[arm_solve_server] Cancel requested");
  std::scoped_lock<std::mutex> lock(active_mtx_);

  auto active = active_goal_handle_.lock();
  if (active && active.get() == gh.get() && active_ctx_) {
    active_ctx_->cancel_requested.store(true);
    if (solve_executor_) {
      solve_executor_->stop();
    }
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

void ArmSolveServer::handle_accepted(const std::shared_ptr<GoalHandleMove> gh) {
  auto goal = gh->get_goal();
  auto ctx = std::make_shared<GoalContext>();

  ctx->option = static_cast<solve_core::PlanOption>(goal->option_id);
  ctx->target_pose.header.stamp = now();
  ctx->target_pose.header.frame_id =
      solve_executor_ ? solve_executor_->planning_frame_id() : "";
  ctx->target_pose.pose.orientation.x = goal->target_pose.qx;
  ctx->target_pose.pose.orientation.y = goal->target_pose.qy;
  ctx->target_pose.pose.orientation.z = goal->target_pose.qz;
  ctx->target_pose.pose.orientation.w = goal->target_pose.qw;
  ctx->target_pose.pose.position.x = goal->target_pose.x;
  ctx->target_pose.pose.position.y = goal->target_pose.y;
  ctx->target_pose.pose.position.z = goal->target_pose.z;
  ctx->target_vector = {static_cast<double>(goal->target_vector.x),
                        static_cast<double>(goal->target_vector.y),
                        static_cast<double>(goal->target_vector.z)};
  ctx->target_joints = goal->target_joints;

  {
    std::scoped_lock<std::mutex> lock(current_joints_mutex_);
    ctx->current_joints.names.reserve(current_joints_.joints.size());
    ctx->current_joints.positions.reserve(current_joints_.joints.size());
    for (const auto &joint : current_joints_.joints) {
      ctx->current_joints.names.push_back(joint.name);
      ctx->current_joints.positions.push_back(joint.position);
    }
  }

  {
    std::scoped_lock<std::mutex> lock(active_mtx_);
    if (active_ctx_) {
      active_ctx_->cancel_requested.store(true);
      if (solve_executor_) {
        solve_executor_->stop();
      }
    }
    active_ctx_ = ctx;
    active_goal_handle_ = gh;
  }

  auto self = std::static_pointer_cast<ArmSolveServer>(shared_from_this());
  std::thread([self, gh, ctx]() { self->execute(gh, ctx); }).detach();
}

void ArmSolveServer::execute(const std::shared_ptr<GoalHandleMove> gh,
                             const std::shared_ptr<GoalContext> &ctx) {
  const auto cleanup_active_goal = [this, &gh]() {
    std::scoped_lock<std::mutex> lock(active_mtx_);
    auto active = active_goal_handle_.lock();
    if (active && active.get() == gh.get()) {
      active_ctx_.reset();
      active_goal_handle_.reset();
    }
  };

  try {
    if (isCanceled(gh, ctx)) {
      RCLCPP_WARN(get_logger(), "[arm_solve_server] Goal canceled before planning");
      finish_move_goal(gh, false, true, "Goal canceled before planning");
      cleanup_active_goal();
      return;
    }

    std::string err;
    solve_core::Trajectory traj_msg;
    if (!solve_executor_ || !solve_executor_->execute(*ctx, traj_msg, err)) {
      if (!solve_executor_ && err.empty()) {
        err = "SolveExecutor not available";
        RCLCPP_ERROR(get_logger(), "[arm_solve_server] %s", err.c_str());
      }
      if (isCanceled(gh, ctx)) {
        finish_move_goal(gh, false, true, err.empty() ? "Goal canceled" : err);
      } else {
        finish_move_goal(gh, false, false,
                         err.empty() ? "Planning failed" : err);
      }
      cleanup_active_goal();
      return;
    }

    if (isCanceled(gh, ctx)) {
      RCLCPP_WARN(get_logger(), "[arm_solve_server] Goal canceled after planning");
      finish_move_goal(gh, false, true, "Goal canceled");
      cleanup_active_goal();
      return;
    }

    ctx->traj = std::move(traj_msg);
    if (!publishTrajectoryPoints(gh, ctx)) {
      if (isCanceled(gh, ctx)) {
        finish_move_goal(gh, false, true, "Goal canceled during execution");
      } else {
        finish_move_goal(gh, false, false, "Trajectory publish failed");
      }
      cleanup_active_goal();
      return;
    }

    finish_move_goal(gh, true, false, "");
  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_logger(), "[arm_solve_server] Exception: %s", e.what());
    finish_move_goal(gh, false, false, e.what());
  } catch (...) {
    RCLCPP_ERROR(get_logger(), "[arm_solve_server] Unknown exception");
    finish_move_goal(gh, false, false, "Unknown exception");
  }
  cleanup_active_goal();
}

bool ArmSolveServer::publishTrajectoryPoints(
    const std::shared_ptr<GoalHandleMove> gh,
    const std::shared_ptr<GoalContext> &ctx) {
  const auto &traj = ctx->traj;

  if (traj.joint_names.empty() || traj.points.empty()) {
    RCLCPP_ERROR(get_logger(), "[arm_solve_server] Trajectory is empty");
    return false;
  }

  for (size_t i = 0; i < traj.points.size(); ++i) {
    if (isCanceled(gh, ctx)) {
      RCLCPP_WARN(get_logger(),
                  "[arm_solve_server] Goal canceled during trajectory publish");
      return false;
    }

    if (traj.points[i].positions.size() < traj.joint_names.size()) {
      RCLCPP_ERROR(
          get_logger(),
          "[arm_solve_server] Trajectory point %zu positions size %zu < joint_names size %zu",
          i, traj.points[i].positions.size(), traj.joint_names.size());
      return false;
    }

    engineer_interfaces::msg::Joints cmd;
    cmd.header.stamp = now();

    for (size_t j = 0; j < traj.joint_names.size(); ++j) {
      engineer_interfaces::msg::Joint joint;
      joint.header.stamp = cmd.header.stamp;
      joint.name = traj.joint_names[j];
      joint.position = traj.points[i].positions[j];
      joint.velocity = (j < traj.points[i].velocities.size())
                           ? traj.points[i].velocities[j]
                           : 0.0;
      joint.mode = "planned";
      cmd.joints.push_back(joint);
    }

    joint_cmd_pub_->publish(cmd);

    auto fb = std::make_shared<Move::Feedback>();
    fb->progress = static_cast<float>(i + 1) / traj.points.size();
    gh->publish_feedback(fb);

    if (i + 1 < traj.points.size()) {
      const double dt =
          traj.points[i + 1].time_from_start - traj.points[i].time_from_start;
      if (dt > 0.0) {
        rclcpp::sleep_for(
            std::chrono::nanoseconds(static_cast<int64_t>(dt * 1e9)));
      }
    }
  }

  return true;
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(arm_solve::ArmSolveServer)
} // namespace arm_solve

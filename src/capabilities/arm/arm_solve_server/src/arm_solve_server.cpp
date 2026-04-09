#include "arm_solve_server/arm_solve_server.hpp"
#include "executor/executor.hpp"
#include "engineer_interfaces/action/arm_move.hpp"

#include <chrono>
#include <cmath>
#include <exception>
#include <memory>
#include <rclcpp/logger.hpp>
#include <string>
#include <thread>

#include <rclcpp/node_options.hpp>

#include "log_tools/log.hpp"

namespace arm_solve
{

using namespace std::chrono_literals;

namespace
{
std::shared_ptr<GoalContext> makeGoalContext(const ArmMove::Goal& goal,
                                             const engineer_interfaces::msg::Joints& current_joints,
                                             const arm_solve::ArmSolveConfig& config)
{
  (void)config;
  std::shared_ptr<GoalContext> ctx = std::make_shared<GoalContext>();
  ctx->req.option = static_cast<solve_executor::PlanOption>(goal.option_id);
  ctx->req.target_pose.x = goal.target_pose.x;
  ctx->req.target_pose.y = goal.target_pose.y;
  ctx->req.target_pose.z = goal.target_pose.z;
  ctx->req.target_pose.qx = goal.target_pose.qx;
  ctx->req.target_pose.qy = goal.target_pose.qy;
  ctx->req.target_pose.qz = goal.target_pose.qz;
  ctx->req.target_pose.qw = goal.target_pose.qw;
  ctx->req.target_vector = { static_cast<double>(goal.target_vector.x), static_cast<double>(goal.target_vector.y),
                             static_cast<double>(goal.target_vector.z) };
  ctx->req.target_length = goal.target_length;
  ctx->req.target_joints.assign(goal.target_joints.begin(), goal.target_joints.end());
  ctx->req.current_joints.names.reserve(current_joints.joints.size());
  ctx->req.current_joints.positions.reserve(current_joints.joints.size());
  for (const auto& joint : current_joints.joints)
  {
    ctx->req.current_joints.names.push_back(joint.name);
    ctx->req.current_joints.positions.push_back(joint.position);
  }
  return ctx;
}

std::shared_ptr<ArmMove::Result> make_move_result(bool success, const std::string& msg)
{
  auto result = std::make_shared<ArmMove::Result>();
  result->success = success;
  result->error_msg = msg;
  return result;
}

// ------------------------------------------------------------------
// 任务状态反馈
// ------------------------------------------------------------------
void finish_move_goal(const std::shared_ptr<GoalHandleArmMove>& gh, bool success, bool canceled, const std::string& msg)
{
  if (!gh)
  {
    return;
  }
  auto result = make_move_result(success, msg);
  if (success)
  {
    gh->succeed(result);
    return;
  }
  if (canceled)
  {
    gh->canceled(result);
    return;
  }
  gh->abort(result);
}
}  // namespace

ArmSolveServer::ArmSolveServer(const rclcpp::NodeOptions& options)
  : Node("arm_solve_action_server", rclcpp::NodeOptions(options))
  , logger_(this->get_logger())
  , config_(ArmSolveConfig::Load(*this))
{
  log_tools::init_console_logger("core");

  solve_executor_ = std::make_unique<solve_executor::SolveExecutor>(*this);

  joint_states_verbose_sub_ = this->create_subscription<engineer_interfaces::msg::Joints>(
      config_.joint_states_verbose_topic, rclcpp::QoS(10),
      std::bind(&ArmSolveServer::jointCallBack, this, std::placeholders::_1));

  joint_cmd_pub_ = this->create_publisher<engineer_interfaces::msg::Joints>(config_.joint_cmd_topic, rclcpp::QoS(10));

  action_server_ = rclcpp_action::create_server<ArmMove>(
      this, config_.arm_action_name,
      std::bind(&ArmSolveServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&ArmSolveServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&ArmSolveServer::handle_accepted, this, std::placeholders::_1));

  LOGI("[ARM_SOLVE_SERVER] started");
  RCLCPP_INFO(this->logger_, "%s", config_.summary().c_str());
}

void ArmSolveServer::jointCallBack(const engineer_interfaces::msg::Joints::SharedPtr msg)
{
  std::scoped_lock<std::mutex> lock(current_joints_mutex_);
  current_joints_ = *msg;
}

rclcpp_action::GoalResponse ArmSolveServer::handle_goal(const rclcpp_action::GoalUUID& uuid,
                                                        std::shared_ptr<const ArmMove::Goal> goal)
{
  (void)uuid;
  LOGI("[arm_solve_server] Received goal: option_id={}", goal->option_id);

  if (!solve_executor_)
  {
    LOGW("[arm_solve_server] SolveExecutor not available");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ArmSolveServer::handle_cancel(const std::shared_ptr<GoalHandleArmMove> gh)
{
  LOGI("[arm_solve_server] Cancel requested");
  std::scoped_lock<std::mutex> lock(active_mtx_);

  auto active = active_goal_handle_.lock();
  if (active && active.get() == gh.get() && active_ctx_)
  {
    active_ctx_->cancel_requested.store(true);
    if (solve_executor_)
    {
      solve_executor_->stop();
    }
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

void ArmSolveServer::handle_accepted(const std::shared_ptr<GoalHandleArmMove> gh)
{
  auto goal = gh->get_goal();
  engineer_interfaces::msg::Joints current_joints_snapshot;
  {
    std::scoped_lock<std::mutex> lock(current_joints_mutex_);
    current_joints_snapshot = current_joints_;
  }
  auto ctx = makeGoalContext(*goal, current_joints_snapshot, config_);

  {
    std::scoped_lock<std::mutex> lock(active_mtx_);
    if (active_ctx_)
    {
      active_ctx_->cancel_requested.store(true);
      if (solve_executor_)
      {
        solve_executor_->stop();
      }
    }
    active_ctx_ = ctx;
    active_goal_handle_ = gh;
  }

  auto self = std::static_pointer_cast<ArmSolveServer>(shared_from_this());
  std::thread([self, gh, ctx]() { self->execute(gh, ctx); }).detach();
}

void ArmSolveServer::execute(const std::shared_ptr<GoalHandleArmMove> gh, const std::shared_ptr<GoalContext>& ctx)
{
  const auto cleanup_active_goal = [this, &gh]() {
    std::scoped_lock<std::mutex> lock(active_mtx_);
    auto active = active_goal_handle_.lock();
    if (active && active.get() == gh.get())
    {
      active_ctx_.reset();
      active_goal_handle_.reset();
    }
  };

  try
  {
    if (isCanceled(gh, ctx))
    {
      LOGW("[arm_solve_server] Goal canceled before planning");
      finish_move_goal(gh, false, true, "Goal canceled before planning");
      cleanup_active_goal();
      return;
    }

    std::string err;
    solve_executor::Trajectory traj_msg;
    if (!solve_executor_ || !solve_executor_->execute(ctx->req, traj_msg, err))
    {
      if (!solve_executor_ && err.empty())
      {
        err = "SolveExecutor not available";
        LOGE("[arm_solve_server] {}", err);
      }
      if (isCanceled(gh, ctx))
      {
        finish_move_goal(gh, false, true, err.empty() ? "Goal canceled" : err);
      }
      else
      {
        finish_move_goal(gh, false, false, err.empty() ? "Planning failed" : err);
      }
      cleanup_active_goal();
      return;
    }

    if (isCanceled(gh, ctx))
    {
      LOGW("[arm_solve_server] Goal canceled after planning");
      finish_move_goal(gh, false, true, "Goal canceled");
      cleanup_active_goal();
      return;
    }

    ctx->traj = std::move(traj_msg);
    if (!publishTrajectoryPoints(gh, ctx))
    {
      if (isCanceled(gh, ctx))
      {
        finish_move_goal(gh, false, true, "Goal canceled during execution");
      }
      else
      {
        finish_move_goal(gh, false, false, "Trajectory publish failed");
      }
      cleanup_active_goal();
      return;
    }

    finish_move_goal(gh, true, false, "");
  }
  catch (const std::exception& e)
  {
    LOGE("[arm_solve_server] Exception: {}", e.what());
    finish_move_goal(gh, false, false, e.what());
  }
  catch (...)
  {
    LOGE("[arm_solve_server] Unknown exception");
    finish_move_goal(gh, false, false, "Unknown exception");
  }
  cleanup_active_goal();
}

bool ArmSolveServer::publishTrajectoryPoints(const std::shared_ptr<GoalHandleArmMove> gh,
                                             const std::shared_ptr<GoalContext>& ctx)
{
  const auto& traj = ctx->traj;

  if (traj.joint_names.empty() || traj.points.empty())
  {
    LOGE("[arm_solve_server] Trajectory is empty");
    return false;
  }

  for (size_t i = 0; i < traj.points.size(); ++i)
  {
    if (isCanceled(gh, ctx))
    {
      LOGW("[arm_solve_server] Goal canceled during trajectory publish");
      return false;
    }

    if (traj.points[i].positions.size() < traj.joint_names.size())
    {
      LOGE("[arm_solve_server] Trajectory point {} positions size {} < joint_names size {}", i,
           traj.points[i].positions.size(), traj.joint_names.size());
      return false;
    }

    engineer_interfaces::msg::Joints cmd;
    cmd.header.stamp = now();

    for (size_t j = 0; j < traj.joint_names.size(); ++j)
    {
      engineer_interfaces::msg::Joint joint;
      joint.header.stamp = cmd.header.stamp;
      joint.name = traj.joint_names[j];
      joint.position = traj.points[i].positions[j];
      joint.velocity = (j < traj.points[i].velocities.size()) ? traj.points[i].velocities[j] : 0.0;
      joint.mode = "planned";
      cmd.joints.push_back(joint);
    }

    joint_cmd_pub_->publish(cmd);

    auto fb = std::make_shared<ArmMove::Feedback>();
    fb->progress = static_cast<float>(i + 1) / traj.points.size();
    gh->publish_feedback(fb);

    if (i + 1 < traj.points.size())
    {
      const double dt = traj.points[i + 1].time_from_start - traj.points[i].time_from_start;
      if (dt > 0.0)
      {
        rclcpp::sleep_for(std::chrono::nanoseconds(static_cast<int64_t>(dt * 1e9)));
      }
    }
  }

  return true;
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(arm_solve::ArmSolveServer)
}  // namespace arm_solve

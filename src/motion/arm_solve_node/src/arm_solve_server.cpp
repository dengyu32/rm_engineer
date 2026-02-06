#include "arm_solve/arm_solve_server.hpp"
#include "error_code_utils/app_error.hpp"

#include <exception>
#include <memory>
#include <chrono>
#include <optional>

// MoveIt
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/collision_detection/collision_common.h>

// ROS
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <rclcpp/node_options.hpp>

// ============================================================================
//  Internal helpers (MoveIt adapter)
// ============================================================================
namespace {

class MoveItAdapterImpl : public solve_core::MoveItAdapter {
public:
  MoveItAdapterImpl(moveit::planning_interface::MoveGroupInterface *move_group,
                    std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> psm,
                    rclcpp::Logger logger)
      : move_group_(move_group),
        psm_(std::move(psm)),
        logger_(logger) {}

  std::shared_ptr<const moveit::core::RobotModel> robot_model() const override {
    if (!move_group_) {
      return nullptr;
    }
    return move_group_->getRobotModel();
  }

  const moveit::core::JointModelGroup *joint_model_group(
      const std::string &group_name) const override {
    const auto model = robot_model();
    if (!model) {
      return nullptr;
    }
    return model->getJointModelGroup(group_name);
  }

  std::string group_name() const override {
    return move_group_ ? move_group_->getName() : std::string();
  }

  std::string planning_frame() const override {
    return move_group_ ? move_group_->getPlanningFrame() : std::string();
  }

  std::string end_effector_link() const override {
    return move_group_ ? move_group_->getEndEffectorLink() : std::string();
  }

  void set_start_state(const moveit::core::RobotState &state) override {
    if (move_group_) {
      move_group_->setStartState(state);
    }
  }

  std::optional<solve_core::Trajectory> plan_to_joint_target(
      const std::vector<std::string> &joint_names,
      const std::vector<double> &joint_values,
      const solve_core::PlannerOptions &options) override {
    if (!move_group_) {
      RCLCPP_ERROR(logger_, "[arm_solve_server] MoveGroup not ready");
      return std::nullopt;
    }

    move_group_->setPlanningTime(options.planning_time);
    move_group_->setNumPlanningAttempts(options.num_planning_attempts);
    move_group_->setMaxVelocityScalingFactor(options.max_velocity_scaling);
    move_group_->setMaxAccelerationScalingFactor(options.max_acc_scaling);
    move_group_->setGoalPositionTolerance(options.goal_position_tolerance);
    move_group_->setGoalOrientationTolerance(options.goal_orientation_tolerance);

    move_group_->setJointValueTarget(joint_names, joint_values);

    moveit::planning_interface::MoveGroupInterface::Plan plan_msg;
    const bool ok = (move_group_->plan(plan_msg) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!ok) {
      RCLCPP_ERROR(logger_, "[arm_solve_server] Planning failed");
      return std::nullopt;
    }

    solve_core::Trajectory traj;
    const auto &jt = plan_msg.trajectory_.joint_trajectory;
    traj.joint_names = jt.joint_names;
    traj.points.reserve(jt.points.size());
    for (const auto &pt : jt.points) {
      solve_core::TrajectoryPoint p;
      p.positions = pt.positions;
      p.velocities = pt.velocities;
      p.time_from_start = static_cast<double>(pt.time_from_start.sec) +
                          static_cast<double>(pt.time_from_start.nanosec) * 1e-9;
      traj.points.push_back(std::move(p));
    }
    return traj;
  }

  bool check_self_collision(
      const moveit::core::RobotState &state,
      const std::string &group_name,
      std::string &err) const override {
    if (!psm_) {
      err = "Planning scene not ready";
      return false;
    }
    planning_scene_monitor::LockedPlanningSceneRO scene(psm_);
    if (!scene) {
      err = "Planning scene unavailable";
      return false;
    }
    collision_detection::CollisionRequest req;
    collision_detection::CollisionResult res;
    req.group_name = group_name;
    req.contacts = false;
    req.max_contacts = 0;
    scene->checkSelfCollision(req, res, state);
    if (res.collision) {
      err = "Self collision detected";
      return false;
    }
    return true;
  }

private:
  moveit::planning_interface::MoveGroupInterface *move_group_{nullptr};
  std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> psm_;
  rclcpp::Logger logger_;
};

} // namespace

namespace arm_solve {

// ============================================================================
//  ctor
// ============================================================================

using namespace std::chrono_literals;

namespace {
std::shared_ptr<Move::Result> make_move_result(bool success, int error_code, const std::string &msg) {
  auto result = std::make_shared<Move::Result>();
  result->success = success;
  result->error_code = error_code;
  result->error_msg = msg;
  return result;
}

void finish_move_goal(const std::shared_ptr<GoalHandleMove> &gh,
                      bool success,
                      bool canceled,
                      const std::string &msg,
                      int error_code) {
  if (!gh) {
    return;
  }
  auto result = make_move_result(success, error_code, msg);
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

ArmSolveServer::ArmSolveServer(const rclcpp::NodeOptions& options)
    : Node("arm_solve_action_server",
           rclcpp::NodeOptions(options)),
      config_(ArmSolveConfig::Load(*this)),
      last_plan_time_(now() - rclcpp::Duration(std::chrono::milliseconds(config_.plan_min_interval_ms))) {

  // 等待构造完成后再创建 MoveGroup，防止节点还未 fully spinning 就调用
  init_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(config_.late_init_delay_ms),
      [this]() { this->lateInit(); });

  // 订阅 verbose 关节状态，记录规划起点
  joint_states_verbose_sub_ = this->create_subscription<engineer_interfaces::msg::Joints>(
      config_.joint_states_topic, rclcpp::QoS(10),
      std::bind(&ArmSolveServer::jointCallBack, this, std::placeholders::_1));

  // 规划结果拆分到关节命令话题
  joint_cmd_pub_ =
      this->create_publisher<engineer_interfaces::msg::Joints>(config_.joint_cmd_topic, rclcpp::QoS(10));

  // 创建 Move action server，绑定 goal/cancel/accept 回调
  action_server_ = rclcpp_action::create_server<Move>(
      this,
      config_.arm_action_name,
      std::bind(&ArmSolveServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&ArmSolveServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&ArmSolveServer::handle_accepted, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "[ARM_SOLVE_SERVER] started");
  RCLCPP_INFO(this->get_logger(), "\n%s", config_.summary().c_str());
}

void ArmSolveServer::set_error_bus(const std::shared_ptr<error_code_utils::ErrorBus> &bus) {
  error_bus_ = bus;
  if (solve_core_) {
    solve_core_->set_error_bus(bus);
  }
}

void ArmSolveServer::publish_error(const error_code_utils::Error &err) const {
  if (!error_bus_) {
    return;
  }
  error_bus_->publish(err);
}

// ============================================================================
//  Init / utils
// ============================================================================

void ArmSolveServer::jointCallBack(const engineer_interfaces::msg::Joints::SharedPtr msg) {
  std::scoped_lock<std::mutex> lock(current_joints_mutex_);
  current_joints_ = *msg;
}

void ArmSolveServer::lateInit() {
  if (init_timer_)
    init_timer_->cancel();

  auto node_shared = shared_from_this();
  move_group_      = std::make_unique<moveit::planning_interface::MoveGroupInterface>(node_shared, config_.group_name);

  move_group_->setPlanningTime(config_.planning_time);
  move_group_->setNumPlanningAttempts(config_.num_planning_attempts);
  move_group_->setGoalPositionTolerance(config_.goal_position_tolerance);
  move_group_->setGoalOrientationTolerance(config_.goal_orientation_tolerance);
  move_group_->setMaxVelocityScalingFactor(config_.max_velocity_scaling);
  move_group_->setMaxAccelerationScalingFactor(config_.max_acc_scaling);

  if (!psm_) {
    psm_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(shared_from_this(), "robot_description");

    psm_->startSceneMonitor();         // /planning_scene
    psm_->startWorldGeometryMonitor(); // collision objects / octomap 等
    psm_->startStateMonitor();         // /joint_states（或你 remap 的话题）
  }

  if (!moveit_adapter_) {
    moveit_adapter_ = std::make_shared<MoveItAdapterImpl>(move_group_.get(), psm_, get_logger());
  }
  if (!solve_core_) {
    solve_core_ = std::make_unique<solve_core::SolveCore>(moveit_adapter_);
    solve_core_->set_error_bus(error_bus_);
  }
}

bool ArmSolveServer::isMoveGroupReady() const {
  return move_group_ && move_group_->getRobotModel() && !move_group_->getPlanningFrame().empty() && solve_core_;
}

// ============================================================================
//  Planning
// ============================================================================

bool ArmSolveServer::planTrajectory(const std::shared_ptr<GoalContext>& ctx,
                                    solve_core::Trajectory &out_traj,
                                    std::string &err,
                                    int &err_code) {

  if (!solve_core_) {
    err = "SolveCore not ready";
    err_code = static_cast<int>(error_code_utils::app::SolveCode::SolveCoreNotReady);
    RCLCPP_ERROR(get_logger(), "[scope=arm_solve_server][status=error] %s", err.c_str());
    publish_error(error_code_utils::app::make_app_error(error_code_utils::ErrorDomain::SOLVE, error_code_utils::app::SolveCode::SolveCoreNotReady, err));
    return false;
  }

  const auto now_ts = this->now();
  const auto min_interval = rclcpp::Duration(std::chrono::milliseconds(config_.plan_min_interval_ms));
  if (config_.plan_min_interval_ms > 0 && (now_ts - last_plan_time_) < min_interval) {
    RCLCPP_WARN(get_logger(),
                "[scope=arm_solve_server][status=throttle] planning skipped: elapsed=%.2f ms < min=%d ms",
                (now_ts - last_plan_time_).nanoseconds() / 1e6,
                config_.plan_min_interval_ms);
    err = "Planning throttled";
    err_code = static_cast<int>(error_code_utils::app::SolveCode::PlanningThrottled);
    publish_error(error_code_utils::app::make_app_error(error_code_utils::ErrorDomain::SOLVE, error_code_utils::app::SolveCode::PlanningThrottled, err));
    return false;
  }
  last_plan_time_ = now_ts;

  if (isCanceled(active_goal_handle_.lock(), ctx)) {
    err = "Goal canceled before planning";
    err_code = static_cast<int>(error_code_utils::app::SolveCode::GoalCanceled);
    RCLCPP_WARN(get_logger(), "[scope=arm_solve_server][status=cancel] %s", err.c_str());
    publish_error(error_code_utils::app::make_app_error(error_code_utils::ErrorDomain::SOLVE, error_code_utils::app::SolveCode::GoalCanceled, err));
    return false;
  }

  engineer_interfaces::msg::Joints current_joints_copy;
  {
    std::scoped_lock<std::mutex> lock(current_joints_mutex_);
    current_joints_copy = current_joints_;
  }

  solve_core::SolveRequest req;
  req.option = ctx->option;
  req.target_pose = solve_core::Pose{
      ctx->target_pose.pose.position.x,
      ctx->target_pose.pose.position.y,
      ctx->target_pose.pose.position.z,
      ctx->target_pose.pose.orientation.x,
      ctx->target_pose.pose.orientation.y,
      ctx->target_pose.pose.orientation.z,
      ctx->target_pose.pose.orientation.w};
  req.target_joints.assign(ctx->target_joints.begin(), ctx->target_joints.end());
  req.current_joints.names.reserve(current_joints_copy.joints.size());
  req.current_joints.positions.reserve(current_joints_copy.joints.size());
  for (const auto &j : current_joints_copy.joints) {
    req.current_joints.names.push_back(j.name);
    req.current_joints.positions.push_back(j.position);
  }
  req.planner.goal_position_tolerance = config_.goal_position_tolerance;
  req.planner.goal_orientation_tolerance = config_.goal_orientation_tolerance;
  req.planner.planning_time = config_.planning_time;
  req.planner.num_planning_attempts = config_.num_planning_attempts;
  req.planner.max_velocity_scaling = config_.max_velocity_scaling;
  req.planner.max_acc_scaling = config_.max_acc_scaling;
  req.group_name = config_.group_name;

  auto res = solve_core_->plan(req);
  if (!res) {
    err = "Planning failed";
    err_code = static_cast<int>(error_code_utils::app::SolveCode::PlanningFailed);
    RCLCPP_ERROR(get_logger(), "[scope=arm_solve_server][status=error] %s", err.c_str());
    publish_error(error_code_utils::app::make_app_error(error_code_utils::ErrorDomain::SOLVE, error_code_utils::app::SolveCode::PlanningFailed, err));
    return false;
  }

  out_traj = std::move(res->trajectory);
  err_code = 0;
  return true;
}

// ============================================================================
//  Action callbacks
// ============================================================================

rclcpp_action::GoalResponse ArmSolveServer::handle_goal(const rclcpp_action::GoalUUID& uuid,
                                                        std::shared_ptr<const Move::Goal> goal) {
  (void)uuid;
  RCLCPP_INFO(get_logger(), "[scope=arm_solve_server][status=goal] Received goal: option_id=%u", goal->option_id);

  if (!isMoveGroupReady()) {
    RCLCPP_WARN(get_logger(), "[scope=arm_solve_server][status=not_ready] MoveGroup not ready");
    publish_error(error_code_utils::app::make_app_error(error_code_utils::ErrorDomain::SOLVE, error_code_utils::app::SolveCode::MoveGroupNotReady, "MoveGroup not ready"));
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ArmSolveServer::handle_cancel(const std::shared_ptr<GoalHandleMove> gh) {
  RCLCPP_INFO(get_logger(), "[scope=arm_solve_server][status=cancel] Cancel requested");
  std::scoped_lock<std::mutex> lock(active_mtx_);

  auto active = active_goal_handle_.lock();
  if (active && active.get() == gh.get() && active_ctx_) {
    // 只允许取消当前活跃 goal，同时停掉 MoveIt 规划/执行
    active_ctx_->cancel_requested.store(true);
    if (move_group_)
      move_group_->stop();
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

void ArmSolveServer::handle_accepted(const std::shared_ptr<GoalHandleMove> gh) {

  auto goal = gh->get_goal();
  auto ctx  = std::make_shared<GoalContext>();

  ctx->option                         = static_cast<solve_core::PlanOption>(goal->option_id);
  ctx->target_pose.header.stamp       = now();
  ctx->target_pose.header.frame_id    = move_group_->getPlanningFrame();
  ctx->target_pose.pose.orientation.x = goal->target_pose.qx;
  ctx->target_pose.pose.orientation.y = goal->target_pose.qy;
  ctx->target_pose.pose.orientation.z = goal->target_pose.qz;
  ctx->target_pose.pose.orientation.w = goal->target_pose.qw;
  ctx->target_pose.pose.position.x    = goal->target_pose.x;
  ctx->target_pose.pose.position.y    = goal->target_pose.y;
  ctx->target_pose.pose.position.z    = goal->target_pose.z;
  ctx->target_joints                  = goal->target_joints;

  {
    std::scoped_lock<std::mutex> lock(active_mtx_);
    if (active_ctx_) {
      // 新 goal 抢占旧 goal，标记取消并停掉正在执行的规划
      active_ctx_->cancel_requested.store(true);
      if (move_group_)
        move_group_->stop();
    }
    active_ctx_         = ctx;
    active_goal_handle_ = gh;
  }

  // 独立线程执行规划/拆分，避免阻塞 rclcpp action 线程
  auto self = std::static_pointer_cast<ArmSolveServer>(shared_from_this());
  std::thread([self, gh, ctx]() { self->execute(gh, ctx); }).detach();
}

// ============================================================================
//  Worker
// ============================================================================

void ArmSolveServer::execute(const std::shared_ptr<GoalHandleMove> gh, const std::shared_ptr<GoalContext>& ctx) {
  try {
    std::string err;
    int err_code = 0;
    if (isCanceled(gh, ctx)) {
      err = "Goal canceled before execution";
      err_code = static_cast<int>(error_code_utils::app::SolveCode::GoalCanceled);
      RCLCPP_WARN(this->get_logger(), "[scope=arm_solve_server][status=cancel] %s", err.c_str());
      publish_error(error_code_utils::app::make_app_error(error_code_utils::ErrorDomain::SOLVE, error_code_utils::app::SolveCode::GoalCanceled, err));
      finish_move_goal(gh, false, true, err, err_code);
      return;
    }

    solve_core::Trajectory traj_msg;

    if (!planTrajectory(ctx, traj_msg, err, err_code)) {
      const bool canceled = isCanceled(gh, ctx);
      finish_move_goal(gh, false, canceled, err, err_code);
      return;
    }

    if (isCanceled(gh, ctx)) {
      err = "Goal canceled";
      err_code = static_cast<int>(error_code_utils::app::SolveCode::GoalCanceled);
      RCLCPP_WARN(this->get_logger(), "[scope=arm_solve_server][status=cancel] %s", err.c_str());
      publish_error(error_code_utils::app::make_app_error(error_code_utils::ErrorDomain::SOLVE, error_code_utils::app::SolveCode::GoalCanceled, err));
      finish_move_goal(gh, false, true, err, err_code);
      return;
    }

    ctx->traj = std::move(traj_msg);
    if (!publishTrajectoryPoints(gh, ctx, err, err_code)) {
      const bool canceled = isCanceled(gh, ctx);
      finish_move_goal(gh, false, canceled, err, err_code);
    } else {
      finish_move_goal(gh, true, false, "", 0);
    }

    std::scoped_lock<std::mutex> lock(active_mtx_);
    auto active = active_goal_handle_.lock();
    if (active && active.get() == gh.get()) {
      // 清理当前 active goal，引导下一次接收
      active_ctx_.reset();
      active_goal_handle_.reset();
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "[scope=arm_solve_server][status=exception] %s", e.what());
    publish_error(error_code_utils::app::make_app_error(error_code_utils::ErrorDomain::SOLVE, error_code_utils::app::SolveCode::ExceptionThrown, e.what()));
    finish_move_goal(gh, false, false, e.what(),
                     static_cast<int>(error_code_utils::app::SolveCode::ExceptionThrown));
  } catch (...) {
    RCLCPP_ERROR(this->get_logger(), "[scope=arm_solve_server][status=exception] Unknown exception");
    publish_error(error_code_utils::app::make_app_error(error_code_utils::ErrorDomain::SOLVE, error_code_utils::app::SolveCode::ExceptionThrown, "Unknown exception"));
    finish_move_goal(gh, false, false, "Unknown exception",
                     static_cast<int>(error_code_utils::app::SolveCode::ExceptionThrown));
  }
}

bool ArmSolveServer::publishTrajectoryPoints(const std::shared_ptr<GoalHandleMove> gh,
                                             const std::shared_ptr<GoalContext>& ctx,
                                             std::string &err,
                                             int &err_code) {

  const auto& traj = ctx->traj;

  if (traj.joint_names.empty() || traj.points.empty()) {
    err = "Trajectory is empty";
    err_code = static_cast<int>(error_code_utils::app::SolveCode::TrajectoryEmpty);
    RCLCPP_ERROR(get_logger(), "[scope=arm_solve_server][status=error] %s", err.c_str());
    publish_error(error_code_utils::app::make_app_error(error_code_utils::ErrorDomain::SOLVE, error_code_utils::app::SolveCode::TrajectoryEmpty, err));
    return false;
  }

  // 按轨迹点逐个发布关节命令，并同步反馈 progress
  for (size_t i = 0; i < traj.points.size(); ++i) {
    if (isCanceled(gh, ctx)) {
      err = "Goal canceled during execution";
      err_code = static_cast<int>(error_code_utils::app::SolveCode::GoalCanceled);
      RCLCPP_WARN(get_logger(), "[scope=arm_solve_server][status=cancel] %s", err.c_str());
      return false;
    }

    if (traj.points[i].positions.size() < traj.joint_names.size()) {
      RCLCPP_ERROR(get_logger(),
                   "[scope=arm_solve_server][status=error] Trajectory point %zu positions size %zu < joint_names size %zu",
                   i,
                   traj.points[i].positions.size(),
                   traj.joint_names.size());
      err = "Trajectory point positions size mismatch";
      err_code = static_cast<int>(error_code_utils::app::SolveCode::TrajectoryPointMismatch);
      publish_error(error_code_utils::app::make_app_error(error_code_utils::ErrorDomain::SOLVE, error_code_utils::app::SolveCode::TrajectoryPointMismatch, err));
      return false;
    }

    engineer_interfaces::msg::Joints cmd;
    cmd.header.stamp = now();

    for (size_t j = 0; j < traj.joint_names.size(); ++j) {
      engineer_interfaces::msg::Joint joint;
      joint.header.stamp = cmd.header.stamp;
      joint.name         = traj.joint_names[j];
      joint.position     = traj.points[i].positions[j];
      joint.velocity     = (j < traj.points[i].velocities.size()) ? traj.points[i].velocities[j] : 0.0;
      joint.mode         = "planned";
      cmd.joints.push_back(joint);
    }

    joint_cmd_pub_->publish(cmd);

    auto fb      = std::make_shared<Move::Feedback>();
    fb->progress = static_cast<float>(i + 1) / traj.points.size();
    gh->publish_feedback(fb);

    if (i + 1 < traj.points.size()) {
      // 依据轨迹时间戳 sleep，确保下发节奏接近规划时间
      const double dt = traj.points[i + 1].time_from_start - traj.points[i].time_from_start;
      if (dt > 0.0) {
        rclcpp::sleep_for(std::chrono::nanoseconds(static_cast<int64_t>(dt * 1e9)));
      }
    }
  }

  err_code = 0;
  return true;
}

// ============================================================================
//  Registration
// ============================================================================

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(arm_solve::ArmSolveServer)
} // namespace arm_solve

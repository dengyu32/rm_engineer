#pragma once

// C++
#include <memory>
#include <atomic>
#include <mutex>
#include <array>

// Error utils
#include "error_code_utils/error_bus.hpp"

// ROS2
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>

// MoveIt
#include <Eigen/Geometry>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

// Msg
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include "engineer_interfaces/action/move.hpp"
#include "engineer_interfaces/msg/joint.hpp"
#include "engineer_interfaces/msg/joints.hpp"

#include "arm_solve/config.hpp"
#include "solve_core/solve_core.hpp"

namespace arm_solve {

// ============================================================================
//  GoalContext（每个 goal 独立）
// ----------------------------------------------------------------------------
//  - 输入：option + target_pose
//  - 控制：cancel_requested（cancel / preempt）
//  - 输出：traj（规划结果）
// ============================================================================

struct GoalContext {
  // request 
  solve_core::PlanOption option;

  geometry_msgs::msg::PoseStamped target_pose;
  std::array<float, 6> target_joints{{0.f, 0.f, 0.f, 0.f, 0.f, 0.f}};

  // preempt
  std::atomic<bool> cancel_requested{false};

  // target traj
  solve_core::Trajectory traj;
};

// ============================================================================
//  Action Server
// ============================================================================

using Move = engineer_interfaces::action::Move;
using GoalHandleMove = rclcpp_action::ServerGoalHandle<Move>;

// ============================================================================
//  ArmSolveServer
// ----------------------------------------------------------------------------
//  - Move.action 服务端，基于 MoveIt 规划并拆分轨迹为关节指令
//  - 支持 cancel/preempt：新 goal 抢占旧 goal
//  - 维护当前 JointState 缓存与规划参数
// ============================================================================

class ArmSolveServer : public rclcpp::Node {
public:
  explicit ArmSolveServer(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  void set_error_bus(const std::shared_ptr<error_code_utils::ErrorBus> &bus);

private:
  // Parameters
  ArmSolveConfig config_;
  rclcpp::Time last_plan_time_;

  // ROS interfaces
  rclcpp::TimerBase::SharedPtr init_timer_;
  rclcpp::Subscription<engineer_interfaces::msg::Joints>::SharedPtr joint_states_verbose_sub_;
  rclcpp::Publisher<engineer_interfaces::msg::Joints>::SharedPtr joint_cmd_pub_;
  rclcpp_action::Server<Move>::SharedPtr action_server_;

  // MoveIt interfacfes
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> psm_;
  std::mutex psm_mutex_;
  std::shared_ptr<solve_core::MoveItAdapter> moveit_adapter_;
  std::unique_ptr<solve_core::SolveCore> solve_core_;

  // Runtime state
  engineer_interfaces::msg::Joints current_joints_;
  std::mutex current_joints_mutex_;

  // 当前活跃 goal（用于 cancel / preempt）
  std::mutex active_mtx_;
  std::weak_ptr<GoalHandleMove> active_goal_handle_;
  std::shared_ptr<GoalContext> active_ctx_;

  // -----------------------------------------------------------------------
  //  Init / utils
  // -----------------------------------------------------------------------

  void lateInit();
  bool isMoveGroupReady() const;
  void jointCallBack(const engineer_interfaces::msg::Joints::SharedPtr msg);

  // -----------------------------------------------------------------------
  //  Planning
  // -----------------------------------------------------------------------

  bool planTrajectory(const std::shared_ptr<GoalContext> &ctx,
                      solve_core::Trajectory &out_traj,
                      std::string &err,
                      int &err_code);

  // -----------------------------------------------------------------------
  //  Action callbacks
  // -----------------------------------------------------------------------

  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const Move::Goal> goal);

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleMove> gh);

  void handle_accepted(const std::shared_ptr<GoalHandleMove> gh);

  // -----------------------------------------------------------------------
  //  Worker
  // -----------------------------------------------------------------------

  void execute(const std::shared_ptr<GoalHandleMove> gh,
               const std::shared_ptr<GoalContext> &ctx);

  bool publishTrajectoryPoints(const std::shared_ptr<GoalHandleMove> gh,
                               const std::shared_ptr<GoalContext> &ctx,
                               std::string &err,
                               int &err_code);

  // -----------------------------------------------------------------------
  //  Helper
  // -----------------------------------------------------------------------

  inline bool isCanceled(const std::shared_ptr<GoalHandleMove> &gh,
                         const std::shared_ptr<GoalContext> &ctx) const {
    return (gh && gh->is_canceling()) || (ctx && ctx->cancel_requested.load());
  }

  void publish_error(const error_code_utils::Error &err) const;

  std::shared_ptr<error_code_utils::ErrorBus> error_bus_;
};

} // namespace arm_solve

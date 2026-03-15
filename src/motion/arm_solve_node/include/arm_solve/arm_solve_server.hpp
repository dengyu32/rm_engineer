#pragma once

// C++
#include <array>
#include <atomic>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>

// Error utils

// ROS2
#include <rclcpp/rclcpp.hpp>
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

#include "params_utils/param_utils.hpp"
#include "solve_core/solve_core.hpp"


namespace arm_solve {

// ============================================================================
//  ArmSolveConfig
// ----------------------------------------------------------------------------
//  - MoveIt / 话题 / Action 配置
// ============================================================================

struct ArmSolveConfig : public params_utils::MoveItResetConfig,
                        public params_utils::JointResetConfig {
  // Action 服务名（通信参数）
  std::string arm_action_name{"move_arm"};

  // 延迟初始化 MoveGroup 的等待时长（毫秒）
  int late_init_delay_ms{10};
  // 连续两次规划最小间隔（毫秒）
  int plan_min_interval_ms{0};

  //  API
  static ArmSolveConfig Load(rclcpp::Node &node) {
    ArmSolveConfig cfg;

    params_utils::MoveItResetConfig::Load(node, cfg);
    params_utils::JointResetConfig::Load(node, cfg);

    using params_utils::detail::declare_get;
    using params_utils::detail::declare_get_checked;

    //  Required / validated params
    declare_get_checked(
        node, "late_init_delay_ms",
        cfg.late_init_delay_ms,
        [](int v) { return v >= 0 && v <= 5000; },
        "must be in [0, 5000]");

    declare_get_checked(
        node, "plan_min_interval_ms",
        cfg.plan_min_interval_ms,
        [](int v) { return v >= 0 && v <= 5000; },
        "must be in [0, 5000]");

    //  Usually fixed (no check)
    declare_get(node, "arm_action_name", cfg.arm_action_name);

    //  Finalize
    cfg.validate();  // keep only semantic / cross-field checks
    return cfg;
  }
  void validate() const;
  std::string summary() const;
};

inline void ArmSolveConfig::validate() const {
  params_utils::MoveItResetConfig::validate();
  params_utils::JointResetConfig::validate();
  // 当前仅保留字段范围校验，组合语义由 solve_core/config.hpp 维护。
}

inline std::string ArmSolveConfig::summary() const {
  std::ostringstream oss;
  oss << "=========\n";
  oss << " ArmSolveServer Configuration\n\n";
  oss << " Service:\n";
  oss << "   - arm_action_name        : " << arm_action_name << "\n\n";
  oss << params_utils::MoveItResetConfig::summary();
  oss << params_utils::JointResetConfig::summary();
  oss << " Planning:\n";
  oss << "   - late_init_delay_ms    : " << late_init_delay_ms << "\n";
  oss << "   - plan_min_interval_ms  : " << plan_min_interval_ms << "\n\n";
  oss << "=========\n";
  return oss.str();
}

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
  std::array<double, 3> target_vector{{0.0, 0.0, 0.0}};
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

private:
  // Parameters
  ArmSolveConfig config_;
  solve_core::SolveCoreConfig solve_core_config_;
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
  std::mutex active_mtx_; // 保护 active_goal_handle_ 和 active_ctx_
  std::weak_ptr<GoalHandleMove> active_goal_handle_;  //GoalHandleMove用在客户端向服务端发送请求
  std::shared_ptr<GoalContext> active_ctx_;   //GoalContext用在服务端处理每个goal的上下文

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

  void publish_error(int code, const char *name, const std::string &message) const;
};

} // namespace arm_solve

#pragma once

#include <array>
#include <atomic>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "engineer_interfaces/action/move.hpp"
#include "engineer_interfaces/msg/joint.hpp"
#include "engineer_interfaces/msg/joints.hpp"
#include "executor.hpp"
#include "params_utils/param_utils.hpp"
#include "solve_core/solve_core.hpp"

namespace arm_solve {

// ============================================================================
//  ArmSolveConfig
// ----------------------------------------------------------------------------
//  - 
// ============================================================================

struct ArmSolveConfig : public params_utils::MoveItResetConfig,
                        public params_utils::JointResetConfig {
  std::string arm_action_name{"move_arm"};
  int late_init_delay_ms{10};   // 延迟初始化时间
  int plan_min_interval_ms{0};  // 两次规划最短时间间隔

  static ArmSolveConfig Load(rclcpp::Node &node) {
    ArmSolveConfig cfg;

    params_utils::MoveItResetConfig::Load(node, cfg);
    params_utils::JointResetConfig::Load(node, cfg);

    using params_utils::detail::declare_get;
    using params_utils::detail::declare_get_checked;

    declare_get_checked(
        node, "late_init_delay_ms", cfg.late_init_delay_ms,
        [](int v) { return v >= 0 && v <= 5000; }, "must be in [0, 5000]");

    declare_get_checked(
        node, "plan_min_interval_ms", cfg.plan_min_interval_ms,
        [](int v) { return v >= 0 && v <= 5000; }, "must be in [0, 5000]");

    declare_get(node, "arm_action_name", cfg.arm_action_name);

    cfg.validate();
    return cfg;
  }

  void validate() const {
    params_utils::MoveItResetConfig::validate();
    params_utils::JointResetConfig::validate();
  }

  std::string summary() const {
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
};

// ============================================================================
//  GoalContext
// ----------------------------------------------------------------------------
//  - 
// ============================================================================

struct GoalContext {
  solve_core::PlanOption option{solve_core::PlanOption::NORMAL};

  // pose
  geometry_msgs::msg::PoseStamped target_pose;

  // cartesian
  std::array<double, 3> target_vector{{0.0, 0.0, 0.0}};
  double target_length{0.0};

  // joint
  std::array<float, 6> target_joints{{0.f, 0.f, 0.f, 0.f, 0.f, 0.f}};

  solve_core::JointState current_joints;
  std::atomic<bool> cancel_requested{false};
  solve_core::Trajectory traj;
};


// ============================================================================
//  ArmSolveServer
// ----------------------------------------------------------------------------
//  - 
// ============================================================================

using Move = engineer_interfaces::action::Move;
using GoalHandleMove = rclcpp_action::ServerGoalHandle<Move>;

class ArmSolveServer : public rclcpp::Node {
public:
  explicit ArmSolveServer(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  //< config
  ArmSolveConfig config_;
  solve_core::SolveCoreConfig solve_core_config_;

  //< Ros Interfaces
  rclcpp::TimerBase::SharedPtr init_timer_;

  rclcpp::Subscription<engineer_interfaces::msg::Joints>::SharedPtr
      joint_states_verbose_sub_;
  rclcpp::Publisher<engineer_interfaces::msg::Joints>::SharedPtr joint_cmd_pub_;

  rclcpp_action::Server<Move>::SharedPtr action_server_;

  //< slove_executor_
  std::unique_ptr<solve_executor::SolveExecutor> solve_executor_;

  //< 共享变量
  engineer_interfaces::msg::Joints current_joints_;
  std::mutex current_joints_mutex_;

  //< 活跃机制
  std::mutex active_mtx_;
  std::weak_ptr<GoalHandleMove> active_goal_handle_;
  std::shared_ptr<GoalContext> active_ctx_;

  // Ros Callback
  void jointCallBack(const engineer_interfaces::msg::Joints::SharedPtr msg);

  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const Move::Goal> goal);

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleMove> gh);

  void handle_accepted(const std::shared_ptr<GoalHandleMove> gh);

  void execute(const std::shared_ptr<GoalHandleMove> gh,
               const std::shared_ptr<GoalContext> &ctx);

  // Built-in Function
  bool publishTrajectoryPoints(const std::shared_ptr<GoalHandleMove> gh,
                               const std::shared_ptr<GoalContext> &ctx);

  inline bool isCanceled(const std::shared_ptr<GoalHandleMove> &gh,
                         const std::shared_ptr<GoalContext> &ctx) const {
    return (gh && gh->is_canceling()) || (ctx && ctx->cancel_requested.load());
  }
};

} // namespace arm_solve

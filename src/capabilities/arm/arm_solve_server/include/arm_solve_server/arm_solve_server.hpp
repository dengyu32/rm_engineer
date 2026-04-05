#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "engineer_interfaces/action/arm_move.hpp"
#include "engineer_interfaces/msg/joint.hpp"
#include "engineer_interfaces/msg/joints.hpp"
#include "executor/executor.hpp"
#include "types/types.hpp"
#include "robot_config/robot_config.hpp"

/*
  服务端节点
  - 接收动作通信请求
  - 处理取消逻辑
  - 订阅当前关节角
  - 发布规划后得到的关节指令
*/

namespace arm_solve
{

using namespace types;

// ============================================================================
//  ArmSolveConfig
// ----------------------------------------------------------------------------
//  - 服务端参数配置，包括：
//  - 动作通信接口名称
//  - JointResetConfig
// ============================================================================

struct ArmSolveConfig : public params_utils::JointResetConfig
{
  std::string arm_action_name{ "move_arm" };

  static ArmSolveConfig Load(rclcpp::Node& node)
  {
    ArmSolveConfig cfg;

    params_utils::JointResetConfig::Load(node, cfg);

    using params_utils::detail::declare_get;
    using params_utils::detail::declare_get_checked;

    declare_get_checked(
        node, "arm_action_name", cfg.arm_action_name, [](const std::string v) { return !v.empty(); },
        "must not be empty");

    cfg.validate();
    return cfg;
  }

  void validate() const
  {
    params_utils::JointResetConfig::validate();
  }

  std::string summary() const
  {
    std::ostringstream oss;
    oss << "=========\n";
    oss << " ArmSolveServer Configuration\n\n";
    oss << " Service:\n";
    oss << "   - arm_action_name        : " << arm_action_name << "\n\n";
    oss << params_utils::JointResetConfig::summary();
    oss << "=========\n";
    return oss.str();
  }
};

// ============================================================================
//  GoalContext
// ----------------------------------------------------------------------------
// - 动作通信上下文
// - 存放目标字段，结果字段，取消请求
// - 具有校验逻辑，检查添加的值是否有效
// ============================================================================

struct GoalContext
{
  solve_executor::SolveRequest req;
  std::atomic<bool> cancel_requested{ false };
  Trajectory traj;
};

// ============================================================================
//  ArmSolveServer
// ----------------------------------------------------------------------------
// - 功能实现类
// ============================================================================

using ArmMove = engineer_interfaces::action::ArmMove;
using GoalHandleArmMove = rclcpp_action::ServerGoalHandle<ArmMove>;

class ArmSolveServer : public rclcpp::Node
{
public:
  // 构造函数，可选配置
  explicit ArmSolveServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  // 参数
  ArmSolveConfig config_;

  // 通信指针
  rclcpp::Subscription<engineer_interfaces::msg::Joints>::SharedPtr joint_states_verbose_sub_;
  rclcpp::Publisher<engineer_interfaces::msg::Joints>::SharedPtr joint_cmd_pub_;

  rclcpp_action::Server<ArmMove>::SharedPtr action_server_;

  // 共享变量
  engineer_interfaces::msg::Joints current_joints_;
  std::mutex current_joints_mutex_;

  // 活跃机制
  std::mutex active_mtx_;
  std::weak_ptr<GoalHandleArmMove> active_goal_handle_;
  std::shared_ptr<GoalContext> active_ctx_;

  // 执行器对象
  std::unique_ptr<solve_executor::SolveExecutor> solve_executor_;

  // 回调获取当前关节角
  void jointCallBack(const engineer_interfaces::msg::Joints::SharedPtr msg);

  // 请求处理
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const ArmMove::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleArmMove> gh);
  void handle_accepted(const std::shared_ptr<GoalHandleArmMove> gh);

  // 运行服务端程序
  void execute(const std::shared_ptr<GoalHandleArmMove> gh, const std::shared_ptr<GoalContext>& ctx);

  // 发布结果
  bool publishTrajectoryPoints(const std::shared_ptr<GoalHandleArmMove> gh, const std::shared_ptr<GoalContext>& ctx);

  // 客户端请求状态判断
  inline bool isCanceled(const std::shared_ptr<GoalHandleArmMove>& gh, const std::shared_ptr<GoalContext>& ctx) const
  {
    return (gh && gh->is_canceling()) || (ctx && ctx->cancel_requested.load());
  }
};

}  // namespace arm_solve

#pragma once

#include <atomic>
#include <memory>
#include <mutex>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "engineer_interfaces/action/arm_move.hpp"
#include "engineer_interfaces/msg/joint.hpp"
#include "engineer_interfaces/msg/joints.hpp"
#include "arm_solve_server/arm_solve_config.hpp"
#include "executor/executor.hpp"
#include "types/types.hpp"

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
  rclcpp::Logger logger_;

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

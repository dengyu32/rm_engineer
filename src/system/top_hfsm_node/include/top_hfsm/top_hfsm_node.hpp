#pragma once

// ROS2
#include <rclcpp/rclcpp.hpp>

// Error utils
#include "error_code_utils/error_bus.hpp"

// C++
#include <atomic>

// ROS messages
#include <engineer_interfaces/msg/intent.hpp>

// Executors
#include "executors/arm_solve_client.hpp"
#include "top_hfsm/executors/gripper_control_node.hpp"
#include "top_hfsm/util/async_dispatcher.hpp"

// HFSM core (Context + EMachine types)
#include "top_hfsm/executors/moveit_servo_client.hpp"
#include "top_hfsm/top_hfsm_impl.hpp"
#include "top_hfsm/config.hpp"

// TODO: 有点小问题
// 测试 ros2 topic pub /intents engineer_interfaces/msg/Intent
// "{intent_id: 2}" --once 夹爪闭合异常
/*
ros2 topic pub /intents engineer_interfaces/msg/Intent "{intent_id: 2}" --once
*/

namespace top_hfsm {

// ============================================================================
//  IntentType
// ----------------------------------------------------------------------------
//  - 外部输入协议中的意图枚举，驱动 HFSM 状态切换
//  - IDLE 优先级最高，可打断其他动作
// ============================================================================
enum class IntentType : uint8_t {
  IDLE = 0,        // 优先级最高：停止/回待机
  AUTO_INIT = 1,   // 自动初始化流程
  TEST_SOLVE = 2,  // 自动求解/抓取序列
  TEST_CARTESIAN = 3, // 笛卡尔规划测试
  AUTO_GRAB = 4,  // 自动抓取能量单元
  AUTO_STORE = 5, // 自动存放能量单元
  AUTO_GET = 6,   // 自动获取能量单元
  TELEOP_SERVO = 11 // 遥操作伺服
};

// ============================================================================
//  TopHFSMNode
// ----------------------------------------------------------------------------
//  - ROS2 节点包装 HFSM2 机器，调度 ArmSolve/Gripper 执行器
//  - 订阅 Intent，周期 tick 状态机并驱动内部事件
//  - 将意图与执行器上下文写入 EMachineContext
// ============================================================================
class TopHFSMNode : public rclcpp::Node {
public:
  // -----------------------------------------------------------------------
  //  Lifecycle
  // -----------------------------------------------------------------------
  explicit TopHFSMNode(const rclcpp::NodeOptions& options);
  ~TopHFSMNode() override;

private:
  // -----------------------------------------------------------------------
  //  ROS callbacks
  // -----------------------------------------------------------------------
  void intentCallBack(engineer_interfaces::msg::Intent::ConstSharedPtr msg);
  void machine_update_timer_callback();

  // -----------------------------------------------------------------------
  //  Parameters
  // -----------------------------------------------------------------------
  TopHFSMConfig config_;
  rclcpp::Logger logger_;

  // -----------------------------------------------------------------------
  //  Executors (通信层)
  // -----------------------------------------------------------------------
  executors::ArmSolveClient arm_solve_client_;
  executors::GripperControlNode gripper_control_node_;
  executors::MoveItServoClient moveit_servo_client_;
  AsyncDispatcher dispatcher_;

  // -----------------------------------------------------------------------
  //  HFSM (核心状态机)
  // -----------------------------------------------------------------------
  EMachineContext machine_ctx_;
  EMachine::Instance machine_;

  // -----------------------------------------------------------------------
  //  Intent state
  //
  //  latest_intent_:
  //    - subscription callback 写入（异步）
  //    - timer 回调读取（周期驱动）
  //
  //  applied_intent_:
  //    - timer 内部使用：只在 intent 变化时触发一次 changeTo
  // -----------------------------------------------------------------------
  std::atomic<IntentType> latest_intent_{IntentType::IDLE};
  IntentType applied_intent_{IntentType::IDLE};

  // -----------------------------------------------------------------------
  //  ROS handles
  // -----------------------------------------------------------------------
  rclcpp::Subscription<engineer_interfaces::msg::Intent>::SharedPtr intent_sub_;
  rclcpp::TimerBase::SharedPtr machine_update_timer_;

  std::shared_ptr<error_code_utils::ErrorBus> error_bus_;
};

} // namespace top_hfsm

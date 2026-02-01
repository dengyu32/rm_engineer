#pragma once

// HFSM
#include "machine.hpp"

// Context & Executors
#include "top_hfsm/context.hpp"
#include "executors/arm_solve_client.hpp"
#include "executors/gripper_control_node.hpp"
#include "top_hfsm/executors/moveit_servo_client.hpp"

// Tasks
#include "top_hfsm/tasks/auto_testcartesian_task.hpp"
#include "top_hfsm/tasks/auto_testsolve_task.hpp"
#include "top_hfsm/tasks/sequence_task.hpp"
#include "top_hfsm/target_helper.hpp"

// ROS2
#include <rclcpp/rclcpp.hpp>

// ROS messages
#include <engineer_interfaces/msg/target.hpp>

// C++
#include <array>
#include <cstddef>

namespace top_hfsm {

using M = hfsm2::MachineT<hfsm2::Config::ContextT<EMachineContext>>;

// ============================================================================
//  EMachine
// ----------------------------------------------------------------------------
//  - HFSM2 拓扑定义：System/Safety/Mode 三路并行
//  - Mode 下包含 IDLE、AUTO、TELEOP 复合状态
//  - AUTO 内含 AUTOINIT / TESTSOLVE 顺序状态
// ============================================================================

#define S(s) struct s
using EMachine = M::PeerRoot<
    M::Orthogonal<S(System), S(SafetyFSM),
                  M::Composite<S(ModeFSM), S(IDLE),
                               M::Composite<S(AUTO), S(AUTOINIT), S(TESTSOLVE), S(TESTCARTESIAN), S(AUTOGRAB), S(AUTOSTORE), S(AUTOGET)>,
                               M::Composite<S(TELEOP), S(TELEOPSERVO)>>>>;
#undef S

// ============================================================================
//  System layer
// ============================================================================

// ============================================================================
//  System
// ----------------------------------------------------------------------------
//  - 系统层占位状态（Orthogonal 根节点之一）
//  - 用于拓扑划分，当前无自定义逻辑
// ============================================================================
struct System : EMachine::State {};

// ============================================================================
//  SafetyFSM
// ----------------------------------------------------------------------------
//  - 安全层占位状态，可扩展安全检查
// ============================================================================
struct SafetyFSM : EMachine::State {};

// ============================================================================
//  ModeFSM
// ----------------------------------------------------------------------------
//  - 模式层复合状态，子节点区分 IDLE/AUTO/TELEOP
// ============================================================================
struct ModeFSM : EMachine::State {};

// ============================================================================
//  Mode layer
// ============================================================================

// ============================================================================
//  IDLE
// ----------------------------------------------------------------------------
//  - 回待机状态，清理请求并停留空闲
//  - 进入时重置 AUTO 请求，避免重复触发
// ============================================================================
struct IDLE : EMachine::State {
  void enter(Control& control) {
    RCLCPP_INFO(control.context().logger, "[scope=IDLE][status=enter] ENTER IDLE");
  }
};

// ============================================================================
//  AUTO
// ----------------------------------------------------------------------------
//  - AUTO 复合状态，管理 AUTOINIT / TESTSOLVE 子任务
//  - enter 仅记录日志
// ============================================================================
struct AUTO : EMachine::State {
  void enter(Control& control) {
    RCLCPP_INFO(control.context().logger, "[scope=AUTO][status=enter] ENTER AUTO");
    if (control.context().servo_client && control.context().dispatcher) {
      control.context().servo_client->requestPauseServo(*control.context().dispatcher);
    }
  }

  void update(FullControl& control) {}
};

// ============================================================================
//  TELEOP
// ----------------------------------------------------------------------------
//  - 遥操作模式入口，当前仅打印进入日志
// ============================================================================
struct TELEOP : EMachine::State {
  void enter(Control& control) {
    RCLCPP_INFO(control.context().logger, "[scope=TELEOP][status=enter] ENTER TELEOP");
    if (control.context().servo_client && control.context().dispatcher) {
      control.context().servo_client->requestStartServo(*control.context().dispatcher);
    }
  }
};

// ============================================================================
//  Task layer
// ============================================================================

// ============================================================================
//  AUTOINIT
// ----------------------------------------------------------------------------
//  - 自动初始化任务：发送一次初始位姿，完成后回到 IDLE
//  - 进入时允许 ArmSolveClient send once，默认打开夹爪
// ============================================================================
struct AUTOINIT : EMachine::State {
  // -----------------------------------------------------------------------
  //  enter: 准备一次性发送
  // -----------------------------------------------------------------------
  void enter(Control& control) {
    auto& ctx = control.context();
    RCLCPP_INFO(ctx.logger, "[scope=AUTOINIT][status=enter] ENTER AUTOINIT");

    if ((!ctx.arm_client) || (!ctx.gripper_node)) {

      RCLCPP_ERROR(ctx.logger, "[scope=AUTOINIT][status=error] AUTOINIT: nullptr ERROR !");
      return;
    }

    // 进入该 task，允许本 task 触发一次发送
    ctx.arm_client->resetSendOnce();
    // init 默认打开
    ctx.gripper_node->setGripperStatus(executors::GripperStatusType::OPEN);
  }

  // -----------------------------------------------------------------------
  //  update: 发一次 + 检查
  // -----------------------------------------------------------------------
  void update(FullControl& control) {
    auto& ctx = control.context();

    if ((!ctx.arm_client) || (!ctx.gripper_node)) {
      control.changeTo<IDLE>();
      return;
    }

    // Z形位姿：待抓取位置
    // const auto init_pose = makeTarget(0.027691, -0.062626, 0.34746, -0.40657,
    // 0.40149, -0.58185, 0.57876);

    // 设置各个关节角都为0的初始位姿
    // const auto init_pose   = makeTarget(0.14293, -0.0622, 0.75049, -0.00022013, 0.00021978, -0.70767, 0.70654);
    // const auto send_result = ctx.arm_client->sentArmGoal(init_pose, executors::PlanOptionType::NORMAL); // 发完一次自动上锁

    // Z形关节值： 待抓取位置
    const std::array<float, 6> init_joints = {0, -1.042, -2.618, 0, 0, 0};

    const auto send_result =
        ctx.arm_client->sentArmGoal(
            init_joints, executors::PlanOptionType::JOINTS);

    static rclcpp::Clock steady_clock{RCL_STEADY_TIME}; // 时钟

    switch (send_result) {
    case executors::SendResultType::SENT:
    case executors::SendResultType::IN_PROGRESS:
      RCLCPP_INFO_THROTTLE(ctx.logger, steady_clock, 500, "[scope=AUTOINIT][status=running] AUTOINIT running...");
      return;

    case executors::SendResultType::SEND_FAILED:
      RCLCPP_WARN_THROTTLE(
          ctx.logger, steady_clock, 500, "[scope=AUTOINIT][status=send_failed] AUTOINIT send failed, retrying...");
      // 注意：sendOnceAndCheck 内部若 sendGoal 失败会自动释放 latch
      // 所以这里不需要 resetSendOnce()
      return;

    case executors::SendResultType::FAILED:
      RCLCPP_ERROR(ctx.logger, "[scope=AUTOINIT][status=failed] AUTOINIT failed -> IDLE");
      control.changeTo<IDLE>();
      return;

    case executors::SendResultType::SUCCEEDED:
      RCLCPP_INFO(ctx.logger, "[scope=AUTOINIT][status=success] AUTOINIT success -> IDLE");
      control.changeTo<IDLE>();
      return;
    }
  }
};

// ============================================================================
//  TESTSOLVE
// ----------------------------------------------------------------------------
//  - 自动求解任务，按预设姿态序列逐步发送目标
//  - 每步依赖 sendOnceAndCheck，成功后切换夹爪状态并进入下一步
//  - 所有步骤完成或失败后回到 IDLE
// ============================================================================
// ============================================================================
//  TESTSOLVE
// ----------------------------------------------------------------------------
//  - 4阶段：
//    (0) NORMAL    -> poses()[0]
//    (1) GRIPPER   -> CLOSE
//    (2) CARTESIAN -> poses()[1]
//    (3) JOINTS    -> init_joints
//  - 每个阶段完成后 resetSendOnce()，进入下一阶段
// ============================================================================
struct TESTSOLVE : EMachine::State {
  AutoTestSolveTask task_;

  void enter(Control &c) {
    task_.reset();
    RCLCPP_INFO(c.context().logger, "[scope=TESTSOLVE][status=enter] ENTER TESTSOLVE");
  }

  void update(FullControl &c) {
    if (!c.context().dispatcher) {
      RCLCPP_ERROR(c.context().logger, "[scope=TESTSOLVE][status=error] dispatcher null -> IDLE");
      c.changeTo<IDLE>();
      return;
    }

    static rclcpp::Clock steady_clock(RCL_STEADY_TIME);
    task_.update(c.context(), *c.context().dispatcher, steady_clock.now());

    if (!task_.finished())
      return;

    if (task_.lastStatus() == TaskStatus::Success) {
      RCLCPP_INFO(c.context().logger,
                  "[scope=TESTSOLVE][status=success] TESTSOLVE finished -> IDLE");
      c.changeTo<IDLE>();
    } else if (task_.lastStatus() == TaskStatus::Timeout) {
      RCLCPP_WARN(c.context().logger,
                  "[scope=TESTSOLVE][status=timeout] msg=%s -> IDLE",
                  task_.lastMsg().c_str());
      c.changeTo<IDLE>();
    } else {
      RCLCPP_ERROR(c.context().logger,
                   "[scope=TESTSOLVE][status=failed] status=%d msg=%s -> IDLE",
                   static_cast<int>(task_.lastStatus()),
                   task_.lastMsg().c_str());
      c.changeTo<IDLE>();
    }
  }
};

// ============================================================================
//  AUTOGRAB
// ----------------------------------------------------------------------------
//  - 自动初始化任务：
//       得到视觉返回位姿，拿到能量单元后抓住，之后笛卡尔规划拔出能量单元，完成后回到代机位置
//  - 进入时允许 ArmSolveClient send once，默认打开夹爪
// ============================================================================
struct AUTOGRAB : EMachine::State {
  void enter(Control &control) {
    RCLCPP_INFO(control.context().logger,
                "[scope=AUTOGRAB][status=enter] ENTER AUTOGRAB");
  }
  void update(FullControl &c) {}
};

// ============================================================================
//  AUTOSTORE
// ----------------------------------------------------------------------------
//  - 自动初始化任务：自动将能量单元放置到指定位置，完成或回到待机位置
//  - 进入时允许 ArmSolveClient send once，默认打开夹爪
// ============================================================================
  struct AUTOSTORE : EMachine::State {
    void enter(Control &control) {
      RCLCPP_INFO(control.context().logger,
                  "[scope=AUTOSTORE][status=enter] ENTER AUTOSTORE");
    }
    void update(FullControl &c) {}
};

// ============================================================================
//  AUTOGET
// ----------------------------------------------------------------------------
//  - 自动初始化任务：自动从指定位置获取能量单元，完成或回到待机位置
//  - 进入时允许 ArmSolveClient send once，默认打开夹爪
// ============================================================================
struct AUTOGET : EMachine::State {
  void enter(Control &control) {
    RCLCPP_INFO(control.context().logger,
                "[scope=AUTOGET][status=enter] ENTER AUTOGET");
  }
  void update(FullControl &c) {}
};
// ============================================================================
//  TESTCARTESIAN
// ----------------------------------------------------------------------------
//  - 笛卡尔规划测试,采取一次正常逆解算+一次笛卡尔，模拟抓取能量单元的过程
// ============================================================================

struct TESTCARTESIAN : EMachine::State {
  AutoTestCartesianTask task_;

  void enter(Control &c) {
    task_.reset();
    if (c.context().gripper_node) {
      c.context().gripper_node->setGripperStatus(executors::GripperStatusType::OPEN);
    }
    RCLCPP_INFO(c.context().logger, "[scope=TESTCARTESIAN][status=enter] ENTER TESTCARTESIAN");
  }

  void update(FullControl &c) {
    if (!c.context().dispatcher) {
      RCLCPP_ERROR(c.context().logger, "[scope=TESTCARTESIAN][status=error] dispatcher null -> IDLE");
      c.changeTo<IDLE>();
      return;
    }

    static rclcpp::Clock steady_clock(RCL_STEADY_TIME);
    task_.update(c.context(), *c.context().dispatcher, steady_clock.now());

    if (!task_.finished())
      return;

    if (task_.lastStatus() == TaskStatus::Success) {
      RCLCPP_INFO(c.context().logger, "[scope=TESTCARTESIAN][status=success] -> IDLE");
      c.changeTo<IDLE>();
    } else if (task_.lastStatus() == TaskStatus::Timeout) {
      RCLCPP_WARN(c.context().logger,
                  "[scope=TESTCARTESIAN][status=timeout] msg=%s -> IDLE",
                  task_.lastMsg().c_str());
      c.changeTo<IDLE>();
    } else {
      RCLCPP_ERROR(c.context().logger,
                   "[scope=TESTCARTESIAN][status=failed] status=%d msg=%s -> IDLE",
                   static_cast<int>(task_.lastStatus()),
                   task_.lastMsg().c_str());
      c.changeTo<IDLE>();
    }
  }
};

// ============================================================================
//  TELEOPSERVO
// ----------------------------------------------------------------------------
//  - 遥操作伺服状态，当前仅日志标记进入
// ============================================================================
struct TELEOPSERVO : EMachine::State {
  void enter(Control& control) { RCLCPP_INFO(control.context().logger, "[scope=TELEOPSERVO][status=enter] ENTER TELEOPSERVO"); }
};

} // namespace top_hfsm

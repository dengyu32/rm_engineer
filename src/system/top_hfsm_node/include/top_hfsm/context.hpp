// ============================================================================
// EMachineContext
// ----------------------------------------------------------------------------
// - HFSM 运行时上下文，供状态和任务访问执行器与日志
// ============================================================================
#pragma once

#include <rclcpp/rclcpp.hpp>

#include "top_hfsm/executors/arm_solve_client.hpp"
#include "top_hfsm/executors/gripper_control_node.hpp"
#include "top_hfsm/executors/moveit_servo_client.hpp"
#include "top_hfsm/util/async_dispatcher.hpp"

namespace top_hfsm {

struct EMachineContext {
  executors::ArmSolveClient *arm_client;
  executors::GripperControlNode *gripper_node;
  executors::MoveItServoClient *servo_client;
  AsyncDispatcher *dispatcher;

  rclcpp::Logger logger;

  EMachineContext()
      : arm_client(NULL),
        gripper_node(NULL),
        servo_client(NULL),
        dispatcher(NULL),
        logger(rclcpp::get_logger("top_hfsm_ctx")) {}

  EMachineContext(executors::ArmSolveClient *arm,
                  executors::GripperControlNode *gripper,
                  executors::MoveItServoClient *servo,
                  AsyncDispatcher *disp,
                  const rclcpp::Logger &log)
      : arm_client(arm),
        gripper_node(gripper),
        servo_client(servo),
        dispatcher(disp),
        logger(log) {}
};

} // namespace top_hfsm

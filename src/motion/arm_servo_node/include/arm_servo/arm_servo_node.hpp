#pragma once

// ROS2
#include <rclcpp/rclcpp.hpp>

// Error utils
#include "error_code_utils/error_bus.hpp"

// STL
#include <trajectory_msgs/msg/detail/joint_trajectory__struct.hpp>
#include <vector>

// Msg
#include <engineer_interfaces/msg/joints.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "arm_servo/config.hpp"

namespace arm_servo {

// ============================================================================
//  ArmServoNode (Adapter Only)
// ----------------------------------------------------------------------------
//  - 订阅 moveit_servo 输出的 JointTrajectory (默认:
//  /moveit_servo/joint_trajectory)
//  - 转换为 engineer_interfaces::msg::Joints 并发布到 /joint_commands
//  - 不负责启动/加载 MoveIt Servo，不创建 PlanningSceneMonitor/Servo
//    （这些由 moveit_servo/servo_node_main 完成）
// ============================================================================

class ArmServoNode : public rclcpp::Node {
public:
  // --------------------------------------------------------------------------
  //  Ctor
  // --------------------------------------------------------------------------
  explicit ArmServoNode(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  void set_error_bus(const std::shared_ptr<error_code_utils::ErrorBus> &bus);

private:
  // --------------------------------------------------------------------------
  //  Parameters
  // --------------------------------------------------------------------------
  ArmServoConfig config_;
  rclcpp::Logger logger_;

  std::vector<double> last_pos_;
  std::vector<double> last_vel_;
  rclcpp::Time last_publish_time_;
  bool has_valid_{false};

  // --------------------------------------------------------------------------
  //  ROS Interfaces
  // --------------------------------------------------------------------------
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr servo_traj_sub_;

  rclcpp::Publisher<engineer_interfaces::msg::Joints>::SharedPtr joint_cmd_pub_;

  // --------------------------------------------------------------------------
  //  Setup
  // --------------------------------------------------------------------------
  void initRosInterfaces();

  // --------------------------------------------------------------------------
  //  Callbacks
  // --------------------------------------------------------------------------
  void servoTrajectoryCallback(
      const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);

  // --------------------------------------------------------------------------
  //  Prase & Publish
  // --------------------------------------------------------------------------
  bool setTargetFromTrajectory(
      const trajectory_msgs::msg::JointTrajectory &traj,
      std::vector<double> &target_pos,
      std::vector<double> &target_vel);

  void publishJointCommands(const std::vector<double> &target_pos,
                            const std::vector<double> &target_vel);

  void publish_error(const error_code_utils::Error &err) const;

  std::shared_ptr<error_code_utils::ErrorBus> error_bus_;
};

} // namespace arm_servo

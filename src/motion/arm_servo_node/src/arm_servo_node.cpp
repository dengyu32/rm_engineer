// Hpp
#include "arm_servo/arm_servo_node.hpp"
#include "arm_servo/error_code.hpp"

// C++
#include <algorithm>
#include <string>
#include <trajectory_msgs/msg/detail/joint_trajectory__struct.hpp>
#include <vector>
#include <chrono>

// ROS2
#include <rclcpp_components/register_node_macro.hpp>

namespace arm_servo {

// ============================================================================
//  Ctor
// ============================================================================

ArmServoNode::ArmServoNode(const rclcpp::NodeOptions &options)
    : rclcpp::Node("arm_servo_node", rclcpp::NodeOptions(options)),
      config_(ArmServoConfig::Load(*this)),
      logger_(this->get_logger()) {
  // --------------------------------------------------------------------------
  //  Hold-last init
  // --------------------------------------------------------------------------
  last_pos_.assign(config_.joint_names.size(), 0.0);
  last_vel_.assign(config_.joint_names.size(), 0.0);
  has_valid_ = false;

  // --------------------------------------------------------------------------
  //  ROS interfaces
  // --------------------------------------------------------------------------
  initRosInterfaces();

  // --------------------------------------------------------------------------
  //  Log
  // --------------------------------------------------------------------------
  RCLCPP_INFO(logger_, "\n%s", config_.summary().c_str());
}

void ArmServoNode::set_error_bus(const std::shared_ptr<error_code_utils::ErrorBus> &bus) {
  error_bus_ = bus;
}

void ArmServoNode::publish_error(const error_code_utils::Error &err) const {
  if (!error_bus_) {
    return;
  }
  error_bus_->publish(err);
}

// ============================================================================
//  Setup
// ============================================================================

void ArmServoNode::initRosInterfaces() {
  // --------------------------------------------------------------------------
  //  Publisher: -> hardware adapter / controller
  // --------------------------------------------------------------------------
  joint_cmd_pub_ = create_publisher<engineer_interfaces::msg::Joints>(
      config_.output_topic, rclcpp::QoS(10));

  // --------------------------------------------------------------------------
  //  Subscriber: <- moveit_servo output trajectory
  // --------------------------------------------------------------------------
  servo_traj_sub_ = create_subscription<trajectory_msgs::msg::JointTrajectory>(
      config_.servo_out_topic, rclcpp::QoS(10),
      std::bind(&ArmServoNode::servoTrajectoryCallback, this,
                std::placeholders::_1));
}

// ============================================================================
//  Callbacks
// ============================================================================

void ArmServoNode::servoTrajectoryCallback(
    const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) {

  if(!msg) { return; }

  std::vector<double> target_pos;
  std::vector<double> target_vel;

  if (!setTargetFromTrajectory(*msg, target_pos, target_vel))
    return;

  publishJointCommands(target_pos, target_vel);
}

void ArmServoNode::publishJointCommands(
        const std::vector<double> &target_pos,
        const std::vector<double> &target_vel) {

  const std::size_t joint_count = config_.joint_names.size();
  engineer_interfaces::msg::Joints cmd;
  cmd.header.stamp = now();
  cmd.joints.reserve(joint_count);
  
  for (size_t i = 0; i < joint_count; ++i) {
    engineer_interfaces::msg::Joint joint;
    joint.header.stamp = cmd.header.stamp;
    joint.name = config_.joint_names[i];
    joint.position = target_pos[i];
    joint.velocity = (i < target_vel.size()) ? target_vel[i] : 0.0;
    cmd.joints.push_back(joint);  
  }

  joint_cmd_pub_->publish(cmd);
}
  
bool ArmServoNode::setTargetFromTrajectory(
    const trajectory_msgs::msg::JointTrajectory &traj,
    std::vector<double> &target_pos,
    std::vector<double> &target_vel){

  // --------------------------------------------------------------------------
  // Throttle (drop too-fast messages)
  // --------------------------------------------------------------------------
  const auto now_time = now();
  if (config_.publish_period_ms > 0 && last_publish_time_.nanoseconds() > 0) {
    const auto min_period = rclcpp::Duration(std::chrono::milliseconds(config_.publish_period_ms));
    if ((now_time - last_publish_time_) < min_period) {
      RCLCPP_WARN(logger_, "[arm_servo][drop][reason=throttle]");
      publish_error(make_error(ArmServoErrc::ThrottleDrop, "trajectory throttled"));
      return false;
    }
  }

  // --------------------------------------------------------------------------
  // Basic validate
  // --------------------------------------------------------------------------
  if (traj.joint_names.empty() || traj.points.empty()) {
    RCLCPP_ERROR(logger_, "[arm_servo][drop][reason=trajectory_empty]");
    publish_error(make_error(ArmServoErrc::TrajectoryEmpty, "trajectory empty"));
    return false;
  }

  const auto &point =
      config_.use_last_point_only ? traj.points.back() : traj.points.front();
  if (point.positions.empty()) {
    RCLCPP_ERROR(logger_, "[arm_servo][drop][reason=trajectory_point_empty]");
    publish_error(make_error(ArmServoErrc::TrajectoryPointEmpty, "trajectory point empty"));
    return false;
  }

  const std::size_t joint_count = config_.joint_names.size();

  target_pos.assign(joint_count, 0.0);
  target_vel.assign(joint_count, 0.0);
  std::vector<bool> has_joint(joint_count, false);

  // --------------------------------------------------------------------------
  // Fill buffers using joint_map_ (name -> fixed index)
  // --------------------------------------------------------------------------
  const size_t name_n = traj.joint_names.size();
  const size_t pos_n = point.positions.size();
  const size_t vel_n = point.velocities.size();

  const size_t n = std::min(name_n, pos_n); // 防止越界

  for (size_t i = 0; i < n; ++i) {
    const auto &name = traj.joint_names[i];

    auto it = config_.joint_index.find(name);
    if (it == config_.joint_index.end()) {
      continue; // 保险：未知 joint 直接跳过
    }

    const size_t target_i = it->second;
    if (target_i >= joint_count) {
      continue; // 保险：映射异常
    }

    target_pos[target_i] = point.positions[i];
    target_vel[target_i] = (i < vel_n) ? point.velocities[i] : 0.0;
    has_joint[target_i] = true;
  }

  // --------------------------------------------------------------------------
  // Hold-last strategy:
  // - First frame -> incomplete reject (avoid implicit zeros)
  // - Missing joints -> fill with last values
  // --------------------------------------------------------------------------
  bool incomplete = false;
  
  for (size_t i = 0; i < joint_count; ++i) {
    if (!has_joint[i]) {
      incomplete = true;
      break;
    }
  }

  if (!has_valid_) {
    if (incomplete) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "[arm_servo][drop][reason=incomplete_before_seed]");
      publish_error(make_error(ArmServoErrc::IncompleteBeforeSeed, "incomplete trajectory before seed"));
      return false;
    }
    has_valid_ = true;
  }

  if (incomplete) {
    for (size_t i = 0; i < joint_count; ++i) {
      if (!has_joint[i]) {
        target_pos[i] = last_pos_[i];
        target_vel[i] = last_vel_[i];  // 不想 hold 速度可改为 0.0
      }
    }
  }

  // --------------------------------------------------------------------------
  // Update last cache
  // (publish time放这里：确保 throttle 基于“最近一次成功构造命令”)
  // --------------------------------------------------------------------------
  last_pos_ = target_pos;
  last_vel_ = target_vel;
  last_publish_time_ = now_time;
  return true;
}

} // namespace arm_servo

// ============================================================================
//  Register component
// ============================================================================

RCLCPP_COMPONENTS_REGISTER_NODE(arm_servo::ArmServoNode)

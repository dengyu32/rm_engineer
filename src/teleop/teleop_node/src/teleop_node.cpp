#include "teleop_node/teleop_node.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace engineer_teleop {

// ============================================================================
//  ctor
// ============================================================================

TeleopNode::TeleopNode() : Node("teleop_node"), config_(TeleopConfig::load(*this)) {
  custom_topic_ = config_.joint_states_custom_topic;
  verbose_topic_ = config_.joint_states_verbose_topic;
  output_topic_ = config_.joint_cmd_topic;
  intent_cmd_topic_ = config_.intent_cmd_topic;
  group_name_ = config_.group_name;
  deadband_rad_ = config_.deadband_rad;
  teleop_intent_id_ = static_cast<uint8_t>(config_.teleop_intent_id);
  joint_names_ = config_.joint_names;

  // 状态初始化
  current_joints_.assign(joint_names_.size(), 0.0);
  has_current_joints_.assign(joint_names_.size(), false);
  last_safe_joints_.assign(joint_names_.size(), 0.0);

  for (std::size_t i = 0; i < joint_names_.size(); ++i) {
    joint_name_to_index_[joint_names_[i]] = i;
  }

  joint_cmd_pub_ =
      create_publisher<engineer_interfaces::msg::Joints>(output_topic_, rclcpp::QoS(10));
  intent_sub_ = create_subscription<engineer_interfaces::msg::Intent>(
      intent_cmd_topic_, rclcpp::QoS(10),
      std::bind(&TeleopNode::intentCallback, this, std::placeholders::_1));
  custom_sub_ = create_subscription<engineer_interfaces::msg::Joints>(
      custom_topic_, rclcpp::QoS(10),
      std::bind(&TeleopNode::customCallback, this, std::placeholders::_1));
  verbose_sub_ = create_subscription<engineer_interfaces::msg::Joints>(
      verbose_topic_, rclcpp::QoS(10),
      std::bind(&TeleopNode::verboseCallback, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "\n%s", config_.summary().c_str());
  RCLCPP_INFO(get_logger(),
              "Guarding intent=%s (teleop=%u), %s + %s -> %s (group=%s)",
              intent_cmd_topic_.c_str(),
              static_cast<unsigned>(teleop_intent_id_),
              custom_topic_.c_str(),
              verbose_topic_.c_str(),
              output_topic_.c_str(),
              group_name_.c_str());
}

// ============================================================================
//  ROS 2 Topic Callbacks
// ============================================================================

void TeleopNode::intentCallback(const engineer_interfaces::msg::Intent::SharedPtr msg) {
  std::scoped_lock<std::mutex> lock(state_mutex_);
  teleop_enabled_ = (msg->intent_id == teleop_intent_id_);
}

void TeleopNode::verboseCallback(const engineer_interfaces::msg::Joints::SharedPtr msg) {
  std::scoped_lock<std::mutex> lock(state_mutex_);
  for (const auto &joint : msg->joints) {
    const auto it = joint_name_to_index_.find(joint.name);
    if (it == joint_name_to_index_.end()) {
      continue;
    }
    current_joints_[it->second] = joint.position;
    has_current_joints_[it->second] = true;
  }

  if (hasCompleteCurrentJoints() && isStateSelfCollisionFree(current_joints_)) {
    last_safe_joints_ = current_joints_;
    has_last_safe_ = true;
  }
}

void TeleopNode::customCallback(const engineer_interfaces::msg::Joints::SharedPtr msg) {
  std::scoped_lock<std::mutex> lock(state_mutex_);
  if (!teleop_enabled_) {
    return;
  }
  if (!hasCompleteCurrentJoints()) {
    return;
  }

  std::vector<double> target_positions;
  if (!buildTargetFromCustom(msg, target_positions)) {
    return;
  }

  if (isStateSelfCollisionFree(target_positions)) {
    publishJointCommand(target_positions, "teleop_safe");
    last_safe_joints_ = std::move(target_positions);
    has_last_safe_ = true;
    return;
  }

  RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                       "Self collision predicted, fallback to last_safe_joints");
  if (has_last_safe_) {
    publishJointCommand(last_safe_joints_, "teleop_fallback");
  }
}

// ============================================================================
//  Internal Func
// ============================================================================

bool TeleopNode::buildTargetFromCustom(
    const engineer_interfaces::msg::Joints::SharedPtr &msg,
    std::vector<double> &target_positions) const {
  target_positions = current_joints_;
  bool has_effective_update = false;

  for (const auto &joint : msg->joints) {
    const auto it = joint_name_to_index_.find(joint.name);
    if (it == joint_name_to_index_.end()) {
      continue;
    }
    const std::size_t idx = it->second;
    if (std::abs(joint.position - current_joints_[idx]) <= deadband_rad_) {
      continue;
    }
    target_positions[idx] = joint.position;
    has_effective_update = true;
  }
  return has_effective_update;
}


void TeleopNode::publishJointCommand(
    const std::vector<double> &positions, const std::string &mode_tag) {
  if (positions.size() != joint_names_.size()) {
    return;
  }

  engineer_interfaces::msg::Joints cmd;
  cmd.header.stamp = now();
  cmd.header.frame_id = "base_link";
  cmd.joints.resize(joint_names_.size());
  for (std::size_t i = 0; i < joint_names_.size(); ++i) {
    auto &j = cmd.joints[i];
    j.header = cmd.header;
    j.name = joint_names_[i];
    j.position = positions[i];
    j.velocity = 0.0;
    j.effort = 0.0;
    j.mode = mode_tag;
  }
  joint_cmd_pub_->publish(cmd);
}

} // namespace engineer_teleop

// ============================================================================
//  main 全局入口
// ============================================================================

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<engineer_teleop::TeleopNode>());
  rclcpp::shutdown();
  return 0;
}

#include <algorithm>
#include <cmath>
#include "teleop_input_adapter/teleop_input_adapter.hpp"

TeleopInputAdapter::TeleopInputAdapter() : Node("teleop_input_adapter") {
  custom_topic_ =
      declare_parameter<std::string>("joint_states_custom_topic", "/joint_states_custom");
  verbose_topic_ =
      declare_parameter<std::string>("joint_states_verbose_topic", "/joint_states_verbose");
  output_topic_ =
      declare_parameter<std::string>("delta_joint_topic", "/servo_node/delta_joint_cmds");
  output_frame_id_ = declare_parameter<std::string>("output_frame_id", "base_link");
  min_dt_sec_ = declare_parameter<double>("min_dt_sec", 1e-3);
  max_dt_sec_ = declare_parameter<double>("max_dt_sec", 0.1);
  tracking_gain_ = declare_parameter<double>("tracking_gain", 3.0);
  deadband_rad_ = declare_parameter<double>("deadband_rad", 0.01);
  max_velocity_abs_ = declare_parameter<double>("max_velocity_abs", 2.0);

  joint_jog_pub_ = create_publisher<control_msgs::msg::JointJog>(output_topic_, rclcpp::QoS(10));
  custom_sub_ = create_subscription<engineer_interfaces::msg::Joints>(
      custom_topic_, rclcpp::QoS(10),
      std::bind(&TeleopInputAdapter::customCallback, this, std::placeholders::_1));
  verbose_sub_ = create_subscription<engineer_interfaces::msg::Joints>(
      verbose_topic_, rclcpp::QoS(10),
      std::bind(&TeleopInputAdapter::verboseCallback, this, std::placeholders::_1));

  RCLCPP_INFO(
      get_logger(),
      "Bridging %s + %s -> %s",
      custom_topic_.c_str(),
      verbose_topic_.c_str(),
      output_topic_.c_str());
}

void TeleopInputAdapter::verboseCallback(const engineer_interfaces::msg::Joints::SharedPtr msg) {
  for (const auto &joint : msg->joints) {
    const auto it = joint_name_to_index_.find(joint.name);
    if (it == joint_name_to_index_.end()) {
      continue;
    }
    current_joints_[it->second] = joint.position;
    has_current_joints_[it->second] = true;
  }
}

void TeleopInputAdapter::customCallback(const engineer_interfaces::msg::Joints::SharedPtr msg) {
  const rclcpp::Time now = this->now();
  if (last_custom_stamp_.nanoseconds() == 0) {
    last_custom_stamp_ = now;
    return;
  }

  const double dt = (now - last_custom_stamp_).seconds();
  last_custom_stamp_ = now;
  if (dt < min_dt_sec_ || dt > max_dt_sec_) {
    return;
  }

  control_msgs::msg::JointJog joint_jog;
  joint_jog.header.stamp = now;
  joint_jog.header.frame_id = output_frame_id_;
  joint_jog.joint_names.reserve(msg->joints.size());
  joint_jog.velocities.reserve(msg->joints.size());

  for (const auto &joint : msg->joints) {
    const auto it = joint_name_to_index_.find(joint.name);
    if (it == joint_name_to_index_.end()) {
      continue;
    }

    const std::size_t index = it->second;
    if (!has_current_joints_[index]) {
      continue;
    }

    const double delta = joint.position - current_joints_[index];
    if (std::abs(delta) < deadband_rad_) {
      continue;
    }

    double velocity = tracking_gain_ * delta;
    if (!std::isfinite(velocity)) {
      continue;
    }

    velocity = std::clamp(velocity, -max_velocity_abs_, max_velocity_abs_);
    joint_jog.joint_names.push_back(joint.name);
    joint_jog.velocities.push_back(velocity);
  }

  if (!joint_jog.joint_names.empty()) {
    joint_jog_pub_->publish(joint_jog);
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TeleopInputAdapter>());
  rclcpp::shutdown();
  return 0;
}

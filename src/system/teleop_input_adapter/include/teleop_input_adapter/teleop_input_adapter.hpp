#pragma once

#include <array>
#include <string>
#include <unordered_map>

#include <control_msgs/msg/joint_jog.hpp>
#include <engineer_interfaces/msg/joints.hpp>
#include <rclcpp/rclcpp.hpp>

class TeleopInputAdapter : public rclcpp::Node {
public:
  TeleopInputAdapter();

private:
  void verboseCallback(const engineer_interfaces::msg::Joints::SharedPtr msg);
  void customCallback(const engineer_interfaces::msg::Joints::SharedPtr msg);

  std::string custom_topic_;
  std::string verbose_topic_;
  std::string output_topic_;
  std::string output_frame_id_;
  double min_dt_sec_{1e-3};
  double max_dt_sec_{0.1};
  double tracking_gain_{3.0};
  double deadband_rad_{0.01};
  double max_velocity_abs_{2.0};

  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_jog_pub_;
  rclcpp::Subscription<engineer_interfaces::msg::Joints>::SharedPtr custom_sub_;
  rclcpp::Subscription<engineer_interfaces::msg::Joints>::SharedPtr verbose_sub_;

  const std::unordered_map<std::string, std::size_t> joint_name_to_index_{{
      {"joint1", 0}, {"joint2", 1}, {"joint3", 2},
      {"joint4", 3}, {"joint5", 4}, {"joint6", 5}}};
  std::array<double, 6> current_joints_{};
  std::array<bool, 6> has_current_joints_{};
  rclcpp::Time last_custom_stamp_{0, 0, RCL_ROS_TIME};
};


// Fake system
#include "fake_system/fake_system_node.hpp"
#include "fake_system/error_code.hpp"
#include "param_utils/param_snapshot.hpp"

// C++
#include <chrono>
#include <functional>
#include <string>

// ROS2
#include <rclcpp/node_options.hpp>

namespace fake_system {
using namespace std::chrono_literals;

// ============================================================================
//  ctor
// ============================================================================
FakeSystemNode::FakeSystemNode(const rclcpp::NodeOptions &options)
    : rclcpp::Node("fake_system", options),
      config_(FakeSystemConfig::Load(*this)),
      logger_(this->get_logger()),
      fake_joint_positions_(config_.initial_joint_positions),
      fake_gripper_position_(config_.initial_gripper_position) {

  hfsm_intent_pub_ = this->create_publisher<engineer_interfaces::msg::HFSMIntent>(
      config_.hfsm_intent_topic, rclcpp::QoS(10));
  joint_states_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
      config_.joint_states_topic, rclcpp::QoS(10));
  joint_states_custom_pub_ =
      this->create_publisher<engineer_interfaces::msg::Joints>(
          config_.joint_states_custom_topic, rclcpp::QoS(10));
  joint_states_verbose_pub_ =
      this->create_publisher<engineer_interfaces::msg::Joints>(
          config_.joint_states_verbose_topic, rclcpp::QoS(10));
  joint_cmd_sub_ = this->create_subscription<engineer_interfaces::msg::Joints>(
      config_.joint_cmd_topic, rclcpp::QoS(10),
      [this](engineer_interfaces::msg::Joints::SharedPtr msg) { this->execute(msg); });
  gripper_cmd_sub = this->create_subscription<engineer_interfaces::msg::GripperCommand>(
      config_.gripper_cmd_topic, rclcpp::QoS(10),
      [this](engineer_interfaces::msg::GripperCommand::SharedPtr msg) { this->execute(msg); });
  publish_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(config_.publish_period_ms),
      std::bind(&FakeSystemNode::publish_timer_callback, this));

  // 确保内部缓存大小合法
  if (fake_joint_positions_.empty()) {
    fake_joint_positions_.assign(6, 0.0);
  }

  param_utils::LogSnapshot(this->get_logger(), config_.params_snapshot, "[FAKE_SYSTEM][param] ");
  RCLCPP_INFO(logger_, "FAKE_SYSTEM started");
}

void FakeSystemNode::set_error_bus(const std::shared_ptr<error_code_utils::ErrorBus> &bus) {
  error_bus_ = bus;
}

void FakeSystemNode::publish_error(const error_code_utils::Error &err) const {
  if (!error_bus_) {
    return;
  }
  error_bus_->publish(err);
}

// ============================================================================
//  ROS callbacks
// ============================================================================
void FakeSystemNode::execute(engineer_interfaces::msg::Joints::SharedPtr msg) {
  std::scoped_lock<std::mutex> lock(fake_mutex_); // 直接访问msg->joints[],若消息短会直接崩溃
  const size_t expected = fake_joint_positions_.size();
  const size_t count = std::min(msg->joints.size(), expected);
  for (size_t i = 0; i < count; ++i) {
    fake_joint_positions_[i] = msg->joints[i].position;
  }
  for (size_t i = count; i < expected; ++i) {
    fake_joint_positions_[i] = 0.0;
  }
  if (msg->joints.size() < expected) {
    static auto last_warn = std::chrono::steady_clock::time_point{};
    const auto now_tp = std::chrono::steady_clock::now();
    if (last_warn.time_since_epoch().count() == 0 ||
        now_tp - last_warn >= std::chrono::milliseconds(2000)) {
      last_warn = now_tp;
      RCLCPP_WARN(logger_,
                  "[scope=fake_system_node] joint_commands size=%zu < %zu, fill defaults",
                  msg->joints.size(), expected);
      publish_error(make_error(FakeSystemErrc::JointCommandShort,
                               "joint_commands size smaller than expected",
                               {{"expected", std::to_string(expected)},
                                {"actual", std::to_string(msg->joints.size())}}));
    }
  }
}

void FakeSystemNode::execute(engineer_interfaces::msg::GripperCommand::SharedPtr msg) {
  std::scoped_lock<std::mutex> lock(fake_mutex_);
  fake_gripper_position_ = (msg->gripper_command == 1) ? 0.03 : 0.0;
}

// ============================================================================
//  Timers
// ============================================================================
void FakeSystemNode::publish_timer_callback() {
  std::vector<double> joint_positions_copy;
  double gripper_copy = 0.0;
  {
    std::scoped_lock<std::mutex> lock(fake_mutex_);
    joint_positions_copy = fake_joint_positions_;
    gripper_copy = fake_gripper_position_;
  }

  const size_t joint_count = joint_positions_copy.size();

  // 发布 JointState
  sensor_msgs::msg::JointState joint_states_msg;
  joint_states_msg.header.stamp = this->now();
  joint_states_msg.header.frame_id = "base_link";
  joint_states_msg.name.reserve(joint_count + 1);
  joint_states_msg.position.reserve(joint_count + 1);

  for (size_t i = 0; i < joint_count; ++i) {
    joint_states_msg.name.push_back("joint" + std::to_string(i + 1));
    joint_states_msg.position.push_back(joint_positions_copy[i]);
  }
  joint_states_msg.name.push_back("left_finger_joint");
  joint_states_msg.position.push_back(gripper_copy);

  joint_states_msg.velocity.resize(joint_states_msg.name.size(), 0.0);
  joint_states_msg.effort.resize(joint_states_msg.name.size(), 0.0);
  joint_states_pub_->publish(joint_states_msg);

  // 发布 JointStateVerbose
  engineer_interfaces::msg::Joints joint_states_verbose_msg;
  joint_states_verbose_msg.header.stamp = this->now();
  joint_states_verbose_msg.header.frame_id = "base_link";
  joint_states_verbose_msg.joints.resize(joint_count);
  for (size_t i = 0; i < joint_count; ++i) {
    joint_states_verbose_msg.joints[i].name = "joint" + std::to_string(i + 1);
    joint_states_verbose_msg.joints[i].position = joint_positions_copy[i];
    joint_states_verbose_msg.joints[i].velocity = 0;
    joint_states_verbose_msg.joints[i].effort = 0;
    joint_states_verbose_msg.joints[i].mode = "fake";
  }
  joint_states_verbose_pub_->publish(joint_states_verbose_msg);

  // JointStateCustom（与 verbose 相同格式，供下游兼容）
  auto joint_states_custom_msg = joint_states_verbose_msg;
  joint_states_custom_pub_->publish(joint_states_custom_msg);

  // 发布 Intent（可选）
  if (config_.publish_intent) {
    engineer_interfaces::msg::HFSMIntent hfsm_intent_msg;
    hfsm_intent_msg.stamp = this->now();
    hfsm_intent_msg.intent_id = 0; // 发送 IDLE
    hfsm_intent_pub_->publish(hfsm_intent_msg);
  }
}
} // namespace fake_system

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(fake_system::FakeSystemNode)

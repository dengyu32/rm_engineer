// Fake system
#include "fake_system/fake_system_node.hpp"
#include "error_code_utils/app_error.hpp"

// utils
#include "log_utils/log.hpp"

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

  // ros init
  initRosInterfaces();

  publish_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(config_.publish_period_ms),
      std::bind(&FakeSystemNode::publish_timer_callback, this));

  // log
  log_utils::init_console_logger("core");
  LOGI("\n{}",config_.summary());
  RCLCPP_INFO(logger_, "FAKE_SYSTEM_NODE START!!!"); // 与节点有关的日志还用ros日志
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
//  ROS interfaces
// ============================================================================
void FakeSystemNode::initRosInterfaces() {
  // Publisher
  intent_pub_ = this->create_publisher<engineer_interfaces::msg::Intent>(
      config_.intent_out_topic, rclcpp::QoS(10));
  joint_states_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
      config_.joint_states_topic, rclcpp::QoS(10));
  joint_states_custom_pub_ =
      this->create_publisher<engineer_interfaces::msg::Joints>(
          config_.joint_states_custom_topic, rclcpp::QoS(10));
  joint_states_verbose_pub_ =
      this->create_publisher<engineer_interfaces::msg::Joints>(
          config_.joint_states_verbose_topic, rclcpp::QoS(10));
  // Subscriber
  joint_cmd_sub_ = this->create_subscription<engineer_interfaces::msg::Joints>(
      config_.joint_cmd_topic, rclcpp::QoS(10),
      [this](engineer_interfaces::msg::Joints::SharedPtr msg) { this->execute(msg); });
  gripper_cmd_sub = this->create_subscription<engineer_interfaces::msg::GripperCommand>(
      config_.gripper_cmd_topic, rclcpp::QoS(10),
      [this](engineer_interfaces::msg::GripperCommand::SharedPtr msg) { this->execute(msg); });
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
        now_tp - last_warn >= std::chrono::milliseconds(2000)) {  // 限制频率 2秒最多告警一次
      last_warn = now_tp;
      LOGW("[fake_system] joint_commands size={} < {}, fill defaults",msg->joints.size(), expected);      
      publish_error(error_code_utils::app::make_app_error(
          error_code_utils::ErrorDomain::COMM,
          error_code_utils::app::CommCode::FakeJointCommandShort,
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
  sensor_msgs::msg::JointState joint_states;
  joint_states.header.stamp = this->now();
  joint_states.header.frame_id = "base_link";
  joint_states.name.reserve(joint_count + 1);
  joint_states.position.reserve(joint_count + 1);

  for (size_t i = 0; i < joint_count; ++i) {
    joint_states.name = config_.joint_names;
    joint_states.position.push_back(joint_positions_copy[i]);
  }
  joint_states.name.push_back("left_finger_joint");
  joint_states.position.push_back(gripper_copy);

  joint_states.velocity.resize(joint_states.name.size(), 0.0);
  joint_states.effort.resize(joint_states.name.size(), 0.0);
  joint_states_pub_->publish(joint_states);

  // 发布 JointStateVerbose
  engineer_interfaces::msg::Joints joint_states_verbose;
  joint_states_verbose.header.stamp = this->now();
  joint_states_verbose.header.frame_id = "base_link";
  joint_states_verbose.joints.resize(joint_count);
  for (size_t i = 0; i < joint_count; ++i) {
    joint_states_verbose.joints[i].name = config_.joint_names[i];
    joint_states_verbose.joints[i].position = joint_positions_copy[i];
    joint_states_verbose.joints[i].velocity = 0;
    joint_states_verbose.joints[i].effort = 0;
    joint_states_verbose.joints[i].mode = "fake";
  }
  joint_states_verbose_pub_->publish(joint_states_verbose);

  // JointStateCustom（与 verbose 相同格式，供下游兼容）
  auto joint_states_custom = joint_states_verbose;
  joint_states_custom_pub_->publish(joint_states_custom);

  // 发布 Intent（可选）
  if (config_.publish_intent) {
    engineer_interfaces::msg::Intent intent_msg;
    intent_msg.stamp = this->now();
    intent_msg.intent_id = 0; // 发送 IDLE
    intent_pub_->publish(intent_msg);
  }
}
} // namespace fake_system

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(fake_system::FakeSystemNode)

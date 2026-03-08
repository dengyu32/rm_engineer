// Fake system
#include "fake_system/fake_system_node.hpp"
#include "error_code_utils/app_error.hpp"

// utils
#include "log_utils/log.hpp"

// C++
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <algorithm>

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
  if (config_.initial_slot_status.size() == 2U) {
    fake_slot_status_[0] = config_.initial_slot_status[0];
    fake_slot_status_[1] = config_.initial_slot_status[1];
  }

  // 声明获取动态参数
  this->declare_parameter<int>("fake_intent_id", 0);
  int init_id = this->get_parameter("fake_intent_id").as_int();
  init_id = std::clamp(init_id, 0, 255);
  fake_intent_id_ = static_cast<uint8_t>(init_id);

  // 参数动态回调
  // dynamic param callback
  param_callback_handle_ = this->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> &params) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "";

        for (const auto &p : params) {
          if (p.get_name() != "fake_intent_id") {
            continue;
          }

          if (p.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
            result.successful = false;
            result.reason = "fake_intent_id must be integer";
            return result;
          }

          const int v = p.as_int();
          if (v < 0 || v > 255) {
            result.successful = false;
            result.reason = "fake_intent_id out of range [0,255]";
            return result;
          }

          fake_intent_id_.store(static_cast<uint8_t>(v), std::memory_order_relaxed);
          RCLCPP_INFO(logger_, "[fake_system] fake_intent_id set to %d", v);
        }
        return result;
      });


  // 声明获取动态参数
  this->declare_parameter<int>("fake_intent_id", 0);
  int init_id = this->get_parameter("fake_intent_id").as_int();
  init_id = std::clamp(init_id, 0, 255);
  fake_intent_id_ = static_cast<uint8_t>(init_id);

  // 参数动态回调
  // dynamic param callback
  param_callback_handle_ = this->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> &params) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "";

        for (const auto &p : params) {
          if (p.get_name() != "fake_intent_id") {
            continue;
          }

          if (p.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
            result.successful = false;
            result.reason = "fake_intent_id must be integer";
            return result;
          }

          const int v = p.as_int();
          if (v < 0 || v > 255) {
            result.successful = false;
            result.reason = "fake_intent_id out of range [0,255]";
            return result;
          }

          fake_intent_id_.store(static_cast<uint8_t>(v), std::memory_order_relaxed);
          RCLCPP_INFO(logger_, "[fake_system] fake_intent_id set to %d", v);
        }
        return result;
      });


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
  intent_cmd_pub_ = this->create_publisher<engineer_interfaces::msg::Intent>(
      config_.intent_cmd_topic, rclcpp::QoS(10));
  joint_states_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
      config_.joint_states_topic, rclcpp::QoS(10));
  joint_states_custom_pub_ =
      this->create_publisher<engineer_interfaces::msg::Joints>(
          config_.joint_states_custom_topic, rclcpp::QoS(10));
  joint_states_verbose_pub_ =
      this->create_publisher<engineer_interfaces::msg::Joints>(
          config_.joint_states_verbose_topic, rclcpp::QoS(10));
  slot_states_pub_ = this->create_publisher<engineer_interfaces::msg::Slots>(
      config_.slot_state_topic, rclcpp::QoS(10));

  // Subscriber
  intent_fb_sub_ = this->create_subscription<engineer_interfaces::msg::Intent>(
      config_.intent_fb_topic, rclcpp::QoS(10),
      [this](engineer_interfaces::msg::Intent::SharedPtr msg) { this->intent_feedback_callback(msg); });
  joint_cmd_sub_ = this->create_subscription<engineer_interfaces::msg::Joints>(
      config_.joint_cmd_topic, rclcpp::QoS(10),
      [this](engineer_interfaces::msg::Joints::SharedPtr msg) { this->execute(msg); });
  gripper_cmd_sub = this->create_subscription<engineer_interfaces::msg::Gripper>(
      config_.gripper_cmd_topic, rclcpp::QoS(10),
      [this](engineer_interfaces::msg::Gripper::SharedPtr msg) { this->execute(msg); });
  slot_cmd_sub_ = this->create_subscription<engineer_interfaces::msg::Slots>(
      config_.slot_cmd_topic, rclcpp::QoS(10),
      [this](engineer_interfaces::msg::Slots::SharedPtr msg) { this->execute(msg); });
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

void FakeSystemNode::execute(engineer_interfaces::msg::Gripper::SharedPtr msg) {
  std::scoped_lock<std::mutex> lock(fake_mutex_);
  fake_gripper_position_ = msg->position;
}

void FakeSystemNode::execute(engineer_interfaces::msg::Slots::SharedPtr msg) {
  if (!msg) {
    return;
  }

  bool changed = false;
  {
    std::scoped_lock<std::mutex> lock(fake_mutex_);
    const std::size_t count = std::min<std::size_t>(fake_slot_status_.size(), msg->slots.size());
    for (std::size_t i = 0; i < count; ++i) {
      const bool desired = msg->slots[i].command;
      if (desired != fake_slot_status_[i]) {
        fake_slot_status_[i] = desired;
        changed = true;
      }
    }
  }

  if (changed) {
    publish_slot_states_now();
  }
}

void FakeSystemNode::intent_feedback_callback(const engineer_interfaces::msg::Intent::SharedPtr msg) {
  if (msg->intent_finish == 0) {
    return; // 0=Running，不清零
  }
  const uint8_t current_intent_id = fake_intent_id_.load(std::memory_order_acquire);

  if (current_intent_id != 0) {
    LOGI("[fake_system] intent feedback: id={}, finish={}, reset fake_intent_id({}) -> 0",
         msg->intent_id, msg->intent_finish, current_intent_id);
    fake_intent_id_.store(0, std::memory_order_release); // 非 Running 反馈即重置为 IDLE
  }
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
  joint_states.name = config_.joint_names;
  joint_states.name.reserve(joint_count + 1);
  joint_states.position.reserve(joint_count + 1);

  for (size_t i = 0; i < joint_count; ++i) {
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

  // fake system 不需要发布 joint_states_custom (重要)

  // 发布 Intent
  engineer_interfaces::msg::Intent intent_msg;
  intent_msg.stamp = this->now();
  intent_msg.intent_id = fake_intent_id_.load(std::memory_order_relaxed);
  intent_cmd_pub_->publish(intent_msg);

  publish_slot_states_now();
}

void FakeSystemNode::publish_slot_states_now() {
  if (!slot_states_pub_) {
    return;
  }

  std::array<bool, 2> slot_copy{{false, false}};
  {
    std::scoped_lock<std::mutex> lock(fake_mutex_);
    slot_copy = fake_slot_status_;
  }

  engineer_interfaces::msg::Slots slots_msg;
  slots_msg.header.stamp = this->now();
  slots_msg.header.frame_id = "fake_system";
  slots_msg.slots.resize(slot_copy.size());
  for (std::size_t i = 0; i < slot_copy.size(); ++i) {
    slots_msg.slots[i].header.stamp = slots_msg.header.stamp;
    slots_msg.slots[i].header.frame_id = "slot_" + std::to_string(i);
    slots_msg.slots[i].status = slot_copy[i];
    slots_msg.slots[i].command = slot_copy[i];
  }
  slot_states_pub_->publish(slots_msg);
}
} // namespace fake_system

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(fake_system::FakeSystemNode)

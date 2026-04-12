// ============================================================================
//  usb_cdc_node.cpp
// ----------------------------------------------------------------------------
//  - USB CDC 设备的 ROS2 节点封装，负责收发、解析与状态发布
//  - 组合 Device，完成流式拼帧并回调处理
//  - 订阅关节/夹爪指令，发布 HFSM 意图与 JointState/Joints
//  - 独立读写线程与定时器，保障底层轮询不阻塞 ROS2 
// ============================================================================

// Project
#include "usb_cdc/usb_cdc_node.hpp"
#include "usb_cdc/packet.hpp"
#include "log_tools/log.hpp"

// C++
#include <cstddef>
#include <cstring>
#include <algorithm>
#include <exception>
#include <thread>
#include <chrono>
#include <string>
#include <sys/types.h>

// ROS2
#include <rclcpp/logging.hpp>

namespace usb_cdc {

using namespace std::chrono_literals;

// ============================================================================
//  匿名空间
// ============================================================================
namespace {

// gripper command 映射：上游发送浮点开度，USB 协议层只给下位机发送 open=0 / close=1。
uint8_t toGripperCommand(double gripper_position) {
  // 上游仍发送浮点开度，USB 协议层只给下位机发送 open=0 / close=1。
  return gripper_position > 0.0 ? 1U : 0U;
}

// 初始发送
void initialize_startup_joint_targets(usb_cdc::EngineerTxState &tx_state,
                                      const usb_cdc::UsbCdcConfig &config) {
  tx_state = {};
  const std::size_t count = std::min<std::size_t>(6, config.joint_count);
  for (std::size_t i = 0; i < count; ++i) {
    tx_state.target_joint_position[i] =
        static_cast<float>(config.startup_joint_targets[i]);
    tx_state.target_joint_velocity[i] = 0.0F;
    tx_state.target_joint_effort[i] = 0.0F;
  }
}

// 解码实现
EngineerRxState decode_receive_packet(const EngineerRxPacket &packet) {
  EngineerRxState state{};
  for (std::size_t i = 0; i < 6; ++i) {
    state.actual_joint_position[i] =
        uint_to_float(packet.data.actualJointPosition[i]);
    state.actual_joint_velocity[i] =
        uint_to_float(packet.data.actualJointVelocity[i]);
    state.custom_joint_position[i] =
        uint_to_float(packet.data.customJointPosition[i]);
  }
  state.actual_joint_position[6] =
      uint_to_float(packet.data.actualJointPosition[6], 0.0F, 0.03F);
  state.real_slot_status[0] = packet.data.realSlotStatus[0];
  state.real_slot_status[1] = packet.data.realSlotStatus[1];
  state.intent_status = packet.data.IntentStatus;
  return state;
}

// 编码实现
EngineerTxPacket encode_transmit_packet(const EngineerTxState &state) {
  EngineerTxPacket packet{};
  packet.header.id = 0x02;
  packet.header.len = sizeof(decltype(packet.data));
  packet.header.sof = HeaderFrame::SoF();
  packet.eof = HeaderFrame::EoF();

  for (std::size_t i = 0; i < 6; ++i) {
    packet.data.targetJointPosition[i] =
        float_to_uint(state.target_joint_position[i]);
    packet.data.targetJointVelocity[i] =
        float_to_uint(state.target_joint_velocity[i]);
    packet.data.targetJointEffort[i] = state.target_joint_effort[i];
  }
  packet.data.targetGripperCommand = state.target_gripper_command;
  packet.data.targetSlotStatus[0] = state.target_slot_status[0];
  packet.data.targetSlotStatus[1] = state.target_slot_status[1];
  packet.data.IntentFinish = state.intent_finish;
  return packet;
}

} // namespace

// ============================================================================
//  ctor
// ============================================================================
UsbCdcNode::UsbCdcNode(const rclcpp::NodeOptions &options)
    : rclcpp::Node("usb_cdc", options), 
      logger_(this->get_logger()),
      config_(UsbCdcConfig::Load(*this)) {

  // 动态参数回调
  debug_override_enabled_.store(
      config_.debug_override_enabled, std::memory_order_relaxed);
  debug_intent_id_.store(
      static_cast<uint8_t>(config_.debug_intent_id), std::memory_order_relaxed);

  initialize_startup_joint_targets(tx_state_, config_);
  param_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&UsbCdcNode::on_set_parameters, this, std::placeholders::_1));

  // 接收回调注册 + 打开设备，失败时进入等待重连
  this->init_device_callbacks();
  if (!this->try_open_device()) {
    RCLCPP_WARN(this->get_logger()," [FAILED] initial open FAILED, entering wait mode ");
    device_.request_reconnect();
  }

  // ros init 
  initRosInterfaces();

  // 启动独立线程拉 libusb 事件，避免阻塞 rclcpp executor
  running_ = true;
  thread_ = std::thread([this] {
    while (running_) {
      device_.handle_events();
    }
  });

  // log 
  log_tools::init_console_logger("core");
  LOGI("\n{}",config_.summary());
  RCLCPP_INFO(logger_, "\n%s", config_.summary().c_str());
  RCLCPP_INFO(logger_, "USB_CDC_NODE START!!!");
}

// ============================================================================
//  ROS interfaces initialization
// ============================================================================
void UsbCdcNode::initRosInterfaces() {
  // Publisher
  intent_pub_ = this->create_publisher<engineer_interfaces::msg::Intent>(
      config_.intent_cmd_topic, rclcpp::QoS(10));
  joint_states_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
      config_.joint_states_topic, rclcpp::QoS(10));
  joint_states_custom_pub_ = this->create_publisher<engineer_interfaces::msg::Joints>(
      config_.joint_states_custom_topic, rclcpp::QoS(10));
  joint_states_verbose_pub_ = this->create_publisher<engineer_interfaces::msg::Joints>(
      config_.joint_states_verbose_topic, rclcpp::QoS(10));
  slot_states_pub_ = this->create_publisher<engineer_interfaces::msg::Slots>(
      config_.slot_state_topic, rclcpp::QoS(10));

  // Subscriber
  intent_sub_ = this->create_subscription<engineer_interfaces::msg::Intent>(
      config_.intent_fb_topic,rclcpp::QoS(10),
      std::bind(&UsbCdcNode::IntentCallback, this, std::placeholders::_1));
  joint_cmd_sub_ = this->create_subscription<engineer_interfaces::msg::Joints>(
      config_.joint_cmd_topic, rclcpp::QoS(10),
      std::bind(&UsbCdcNode::jointCommandCallback, this, std::placeholders::_1));
  gripper_cmd_sub_ = this->create_subscription<engineer_interfaces::msg::Gripper>(
      config_.gripper_cmd_topic, rclcpp::QoS(10),
      std::bind(&UsbCdcNode::GripperCommandCallback, this, std::placeholders::_1));
  slot_cmd_sub_ = this->create_subscription<engineer_interfaces::msg::Slots>(
      config_.slot_cmd_topic, rclcpp::QoS(10),
      std::bind(&UsbCdcNode::SlotCommandCallback, this, std::placeholders::_1));

  // 定时器
  publish_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(config_.publish_period_ms),
      std::bind(&UsbCdcNode::publish_timer_callback, this));

  send_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(config_.send_period_ms),
      std::bind(&UsbCdcNode::send_timer_callback, this));
}

// ============================================================================
//  动态参数回调
// ============================================================================
rcl_interfaces::msg::SetParametersResult UsbCdcNode::on_set_parameters(
    const std::vector<rclcpp::Parameter> &params) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  // 无锁读取当前值，准备对比是否有变化
  bool new_debug_override_enabled =
      debug_override_enabled_.load(std::memory_order_relaxed);
  int new_debug_intent_id =
      static_cast<int>(debug_intent_id_.load(std::memory_order_relaxed));
  bool debug_override_changed = false;
  bool debug_intent_changed = false;

  // 遍历参数列表，处理相关参数的变化
  for (const auto &param : params) {
    if (param.get_name() == "debug_override_enabled") {
      // 只允许 bool 类型
      if (param.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
        result.successful = false;
        result.reason = "debug_override_enabled must be bool";
        return result;
      }
      // 记录新值和改变
      new_debug_override_enabled = param.as_bool();
      debug_override_changed = true;
      continue;
    }

    if (param.get_name() == "debug_intent_id") {
      // 只允许 int 类型
      if (param.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
        result.successful = false;
        result.reason = "debug_intent_id must be int";
        return result;
      }
      // 意图 ID 只能是 0-255 的 uint8
      const int value = param.as_int();
      if (value < 0 || value > 255) {
        result.successful = false;
        result.reason = "debug_intent_id out of range [0,255]";
        return result;
      }

      new_debug_intent_id = value;
      debug_intent_changed = true;
    }
  }

  // 更新值
  if (debug_override_changed) {
    debug_override_enabled_.store(
        new_debug_override_enabled, std::memory_order_relaxed);
    RCLCPP_INFO(
        logger_, "[usb_cdc] debug_override_enabled set to %s",
        new_debug_override_enabled ? "true" : "false");
  }
  if (debug_intent_changed) {
    debug_intent_id_.store(
        static_cast<uint8_t>(new_debug_intent_id), std::memory_order_relaxed);
    RCLCPP_INFO(logger_, "[usb_cdc] debug_intent_id set to %d",
                new_debug_intent_id);
  }

  return result;
}

// ============================================================================
//  Device control
// ============================================================================
bool UsbCdcNode::try_open_device() { // catch + retry 策略
  const uint16_t vid = static_cast<uint16_t>(config_.vendor_id);
  const uint16_t pid = static_cast<uint16_t>(config_.product_id);

  try {
    if (device_.open(vid, pid)) {
      RCLCPP_INFO(this->get_logger(),
                  " [SUCCESS] usb driver already ");
      return true;
    }
    RCLCPP_WARN(this->get_logger(),
                " [FAILED] usb driver open returned false ");
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(),
                 " [FAILED] usb driver open exception: %s ",
                 e.what());
  } catch (...) {
    RCLCPP_ERROR(this->get_logger(),
                 " [FAILED] usb driver open unknown exception ");
  }

  RCLCPP_ERROR(this->get_logger(),
               " [FAILED] usb driver open failed ");
  return false;
}


// ============================================================================
//  Device callbacks
// ============================================================================
void UsbCdcNode::engineer_handle_packet(const std::byte *data, size_t size) {
  if (size != sizeof(EngineerRxPacket)) {
    RCLCPP_ERROR(this->get_logger(),
                 " [ERROR] Received packet size mismatch, expected %zu, got %zu ",
                 sizeof(EngineerRxPacket), size);
    return;
  }

  EngineerRxPacket packet{};
  std::memcpy(&packet, data, sizeof(EngineerRxPacket));
  EngineerRxState state = decode_receive_packet(packet);
  static int rx_print_count = 0;
  if (++rx_print_count >= 100) {
    rx_print_count = 0;
    print_rx_packet(packet);
  }

  {
    std::scoped_lock<std::mutex> lock(rx_state_mutex_);
    rx_state_ = state;
  }
}

// ============================================================================
//  Send Timer Callback
// ============================================================================
void UsbCdcNode::send_timer_callback() {
  const bool device_open = device_.is_open();
  if (!device_open) {
    if (last_device_open_) {
      RCLCPP_WARN(this->get_logger(),
                  " [DISCONNECT] USB device disconnected, waiting to reconnect ");
    }
    {
      std::scoped_lock<std::mutex> lock(tx_state_mutex_);
      initialize_startup_joint_targets(tx_state_, config_);
    }
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         " [WAITING] waiting for USB reconnect ");
    last_device_open_ = false;
    return;
  }
  last_device_open_ = true;

  EngineerTxState tx_snapshot{};
  {
    std::scoped_lock<std::mutex> lock(tx_state_mutex_);
    tx_snapshot = tx_state_;
  }

  // 按协议填充发送帧，量化只发生在发送边界。
  EngineerTxPacket tx_data = encode_transmit_packet(tx_snapshot);
  static int tx_print_count = 0;
  if (++tx_print_count >= 100) {
    tx_print_count = 0;
    print_tx_packet(tx_data);
  }

  std::memcpy(buffer_, &tx_data, sizeof(EngineerTxPacket));
  const bool send_ok = device_.send_data(buffer_, sizeof(EngineerTxPacket));

  if (!send_ok) {
    RCLCPP_ERROR(this->get_logger(), " [FAILED] faild to send data ");
  }
}

// ============================================================================
//  Publish Timer Callback
// ============================================================================
void UsbCdcNode::publish_timer_callback() {
  EngineerRxState rx_snapshot{};
  {
    std::scoped_lock<std::mutex> lock(rx_state_mutex_);
    rx_snapshot = rx_state_;
  }
  const auto &d = rx_snapshot;
  const rclcpp::Time stamp = this->now();

  // 发布 custom 关节消息,用于自定义控制
  engineer_interfaces::msg::Joints joint_states_custom;
  joint_states_custom.joints.resize(6);
  for (size_t i = 0; i < 6; ++i) {
    joint_states_custom.joints[i].name = config_.joint_names[i];
    joint_states_custom.joints[i].position =
        d.custom_joint_position[i];
    joint_states_custom.joints[i].velocity = 0;
    joint_states_custom.joints[i].effort = 0;
    joint_states_custom.joints[i].mode = "custom";
  }
  joint_states_custom_pub_->publish(joint_states_custom);

  // 开启servo_teleop_mode后， 只发布 custom_joint_state
  if (config_.servo_teleop_mode) {
    return;
  }

  // 发布 JointState
  sensor_msgs::msg::JointState joint_states;
  joint_states.header.stamp = stamp;
  joint_states.header.frame_id = "base_link";
  joint_states.name = config_.joint_names;
  joint_states.name.push_back("left_gripper_joint");
  joint_states.position = {
      d.actual_joint_position[0],
      d.actual_joint_position[1],
      d.actual_joint_position[2],
      d.actual_joint_position[3],
      d.actual_joint_position[4],
      d.actual_joint_position[5],
      d.actual_joint_position[6]
  };
  joint_states.velocity = {
      d.actual_joint_velocity[0],
      d.actual_joint_velocity[1],
      d.actual_joint_velocity[2],
      d.actual_joint_velocity[3],
      d.actual_joint_velocity[4],
      d.actual_joint_velocity[5],
      0.0
  };
  joint_states.effort.resize(joint_states.name.size(), 0.0);
  joint_states_pub_->publish(joint_states);

  // 发布 verbose 版关节信息，带模式标记
  engineer_interfaces::msg::Joints joint_states_verbose;
  joint_states_verbose.joints.resize(6);
  for (size_t i = 0; i < 6; ++i) {
    joint_states_verbose.joints[i].name = config_.joint_names[i];
    joint_states_verbose.joints[i].position =
        d.actual_joint_position[i];
    joint_states_verbose.joints[i].velocity =
        d.actual_joint_velocity[i];
    joint_states_verbose.joints[i].effort = 0;
    joint_states_verbose.joints[i].mode = "verbose";
  }
  joint_states_verbose_pub_->publish(joint_states_verbose);

  // 发布 HFSM 意图，驱动上层状态机
  engineer_interfaces::msg::Intent intent;
  intent.stamp = this->now();
  intent.intent_id = debug_override_enabled_.load(std::memory_order_relaxed)
                         ? debug_intent_id_.load(std::memory_order_relaxed)
                         : d.intent_status;
  intent_pub_->publish(intent);

  // 发布 slot 状态（两槽）
  engineer_interfaces::msg::Slots slot_states;
  slot_states.header.stamp = stamp;
  slot_states.header.frame_id = "usb_cdc";
  slot_states.slots.resize(2);
  for (size_t i = 0; i < 2; ++i) {
    slot_states.slots[i].header.stamp = stamp;
    slot_states.slots[i].header.frame_id = "slot_" + std::to_string(i);
    slot_states.slots[i].status = d.real_slot_status[i] != 0U;
    slot_states.slots[i].command = d.real_slot_status[i] != 0U;
  }
  slot_states_pub_->publish(slot_states);
}

// ============================================================================
//  Intent Callback
// ============================================================================
void UsbCdcNode::IntentCallback(
    const engineer_interfaces::msg::Intent::SharedPtr msg) {
  const bool debug_override_enabled =
      debug_override_enabled_.load(std::memory_order_acquire);
  const uint8_t current_debug_intent_id =
      debug_intent_id_.load(std::memory_order_acquire);
  if (debug_override_enabled &&
      msg->intent_finish != 0 &&
      current_debug_intent_id != 0 &&
      msg->intent_id == current_debug_intent_id) {
    const auto set_result = this->set_parameter(rclcpp::Parameter("debug_intent_id", 0));
    if (!set_result.successful) {
      RCLCPP_WARN(logger_,
                  "[usb_cdc] failed to reset debug_intent_id to 0: %s",
                  set_result.reason.c_str());
    } else {
      RCLCPP_INFO(logger_,
                  "[usb_cdc] intent feedback: id=%u, finish=%u, reset debug_intent_id -> 0",
                  static_cast<unsigned>(msg->intent_id),
                  static_cast<unsigned>(msg->intent_finish));
    }
  }

  std::scoped_lock<std::mutex> lock(tx_state_mutex_);
  // 状态机任务完成 finish置1
  tx_state_.intent_finish = msg->intent_finish;
}

// ============================================================================
//  Joint Command Callback
// ============================================================================
void UsbCdcNode::jointCommandCallback(
    const engineer_interfaces::msg::Joints::SharedPtr msg) {
  std::scoped_lock<std::mutex> lock(tx_state_mutex_);

  // 将 JointCommand 映射到目标关节位置/速度，编码留到发送边界处理。
  for (const auto &joint : msg->joints) {
    auto it = config_.joint_index.find(joint.name);
    if (it != config_.joint_index.end()) {
      const size_t index = it->second;
      tx_state_.target_joint_position[index] =
          static_cast<float>(joint.position);
      tx_state_.target_joint_velocity[index] =
          static_cast<float>(joint.velocity);
    }
  }
}

// ============================================================================
//  Gripper Command Callback
// ============================================================================
void UsbCdcNode::GripperCommandCallback(
    const engineer_interfaces::msg::Gripper::SharedPtr msg) {
  std::scoped_lock<std::mutex> lock(tx_state_mutex_);
  // 仅改下发协议编码，反馈浮点开度仍按原样发布给上层显示。
  tx_state_.target_gripper_command = toGripperCommand(msg->position);
}

// ============================================================================
//  Slot Command Callback
// ============================================================================
void UsbCdcNode::SlotCommandCallback(
    const engineer_interfaces::msg::Slots::SharedPtr msg) {
  std::scoped_lock<std::mutex> lock(tx_state_mutex_);
  tx_state_.target_slot_status[0] = 0U;
  tx_state_.target_slot_status[1] = 0U;
  if (!msg) {
    return;
  }

  const size_t count = std::min<size_t>(2, msg->slots.size());
  for (size_t i = 0; i < count; ++i) {
    tx_state_.target_slot_status[i] = msg->slots[i].command ? 1U : 0U;
  }
}
} // namespace usb_cdc

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(usb_cdc::UsbCdcNode)

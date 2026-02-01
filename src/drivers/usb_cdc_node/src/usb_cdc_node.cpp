// USB CDC
#include "usb_cdc/usb_cdc_node.hpp"
#include "usb_cdc/error_code.hpp"
#include "usb_cdc/packet.hpp"
#include "param_utils/param_snapshot.hpp"

// C++
#include <cstddef>
#include <exception>
#include <thread>
#include <sys/types.h>
#include <chrono>
#include <string>

// ROS messages
#include <engineer_interfaces/msg/detail/hfsm_intent__struct.hpp>

// ROS2
#include <rclcpp/logging.hpp>

namespace usb_cdc {

using namespace std::chrono_literals;

// ============================================================================
//  ctor
// ============================================================================
UsbCdcNode::UsbCdcNode(const rclcpp::NodeOptions &options)
    : rclcpp::Node("usb_cdc", options), device_(parser_), config_(UsbCdcConfig::Load(*this)) {
  // 解析器注册 + 打开设备，失败时直接抛出阻止组件化加载
  this->init_parser();
  if (!this->try_open_device()) {
    throw std::runtime_error("usb_cdc_node: failed to open USB device");
  }

  // 注册 ROS 发布/订阅端口与定时器
  hfsm_intent_pub_ = this->create_publisher<engineer_interfaces::msg::HFSMIntent>(
      config_.hfsm_intent_topic, rclcpp::QoS(10));
  joint_states_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
      config_.joint_states_topic, rclcpp::QoS(10));
  joint_states_custom_pub_ = this->create_publisher<engineer_interfaces::msg::Joints>(
      config_.joint_states_custom_topic, rclcpp::QoS(10));
  joint_states_verbose_pub_ = this->create_publisher<engineer_interfaces::msg::Joints>(
      config_.joint_states_verbose_topic, rclcpp::QoS(10));
  joint_cmd_sub_ = this->create_subscription<engineer_interfaces::msg::Joints>(
      config_.joint_cmd_topic, rclcpp::QoS(10),
      std::bind(&UsbCdcNode::jointCommandCallback, this, std::placeholders::_1));
  gripper_cmd_sub_ = this->create_subscription<engineer_interfaces::msg::GripperCommand>(
      config_.gripper_cmd_topic, rclcpp::QoS(10),
      std::bind(&UsbCdcNode::GripperCommandCallback, this, std::placeholders::_1));
  publish_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(config_.publish_period_ms),
      std::bind(&UsbCdcNode::publish_timer_callback, this));
  send_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(config_.send_period_ms),
      std::bind(&UsbCdcNode::send_timer_callback, this));

  // 启动独立线程拉 libusb 事件，避免阻塞 rclcpp executor
  running_ = true;
  thread_ = std::thread([this] {
    while (running_) {
      device_.handle_events();
    }
  });

  param_utils::LogSnapshot(this->get_logger(), config_.params_snapshot, "[USB_CDC][param] ");
}

void UsbCdcNode::set_error_bus(const std::shared_ptr<error_code_utils::ErrorBus> &bus) {
  error_bus_ = bus;
}

void UsbCdcNode::publish_error(const error_code_utils::Error &err) const {
  if (!error_bus_) {
    return;
  }
  error_bus_->publish(err);
}

// ============================================================================
//  Device control
// ============================================================================
bool UsbCdcNode::try_open_device() { // catch + retry 策略
  const int kMaxAttempts = config_.open_retry_attempts;
  const auto kRetryDelay = std::chrono::milliseconds(config_.open_retry_delay_ms);
  const uint16_t vid = static_cast<uint16_t>(config_.vendor_id);
  const uint16_t pid = static_cast<uint16_t>(config_.product_id);

  for (int attempt = 1; attempt <= kMaxAttempts; ++attempt) {
    try {
      if (device_.open(vid, pid)) {
        RCLCPP_INFO(this->get_logger(),
                    "[scope=usb_cdc_node][status=success] usb driver already");
        return true;
      }
      RCLCPP_WARN(this->get_logger(),
                  "[scope=usb_cdc_node][status=failed] usb driver open returned false");
      publish_error(make_error(UsbCdcErrc::OpenFailed, "usb driver open returned false"));
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(),
                   "[scope=usb_cdc_node][status=failed] usb driver open exception: %s",
                   e.what());
      publish_error(make_error(UsbCdcErrc::OpenException, e.what()));
    } catch (...) {
      RCLCPP_ERROR(this->get_logger(),
                   "[scope=usb_cdc_node][status=failed] usb driver open unknown exception");
      publish_error(make_error(UsbCdcErrc::OpenException, "usb driver open unknown exception"));
    }

    if (attempt < kMaxAttempts) {
      // 失败后按固定间隔重试，避免阻塞构造函数
      RCLCPP_WARN(this->get_logger(),
                  "[scope=usb_cdc_node][status=retry] retrying open (%d/%d)",
                  attempt, kMaxAttempts);
      std::this_thread::sleep_for(kRetryDelay);
    }
  }

  RCLCPP_ERROR(this->get_logger(),
               "[scope=usb_cdc_node][status=failed] usb driver open failed after retries");
  publish_error(make_error(UsbCdcErrc::OpenFailed, "usb driver open failed after retries"));
  return false;
}


// ============================================================================
//  Parser callbacks
// ============================================================================
void UsbCdcNode::engineer_handle_packet(const std::byte *data, size_t size) {
  std::scoped_lock<std::mutex> lock(rx_data_mutex_);
  if (size < sizeof(EngineerReceiveData)) {
    RCLCPP_ERROR(this->get_logger(),
                 "[scope=usb_cdc_node][status=error] Received packet too small, expected at least %zu, got %zu",
                 sizeof(EngineerReceiveData), size);
    publish_error(make_error(UsbCdcErrc::PacketTooSmall, "Received packet too small",
                             {{"expected_bytes", std::to_string(sizeof(EngineerReceiveData))},
                              {"actual_bytes", std::to_string(size)}}));
    return;
  }
  std::memcpy(&rx_data_, data, sizeof(EngineerReceiveData));
  // 限频打印收到的数据，便于串口调试
  static int i = 0;
  i++;
  if (i == 50) {
    i = 0;
    engineer_print_receive_data(rx_data_);
  }
}

// ============================================================================
//  Timers & callbacks
// ============================================================================
void UsbCdcNode::send_timer_callback() {
  std::scoped_lock<std::mutex> lock(tx_data_mutex_);
  if (!send_enabled_) { // 未收到控制指令前不发串口数据
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "[scope=usb_cdc_node][status=blocked] waiting for joint_commands before sending");
    return;
  }

  // 按协议填充发送帧
  EngineerTransmitData tx_data;
  tx_data.header.id = 0x02;
  tx_data.header.len = sizeof(decltype(tx_data.data));
  tx_data.header.sof = HeaderFrame::SoF();
  tx_data.eof = HeaderFrame::EoF();
  tx_data.data = tx_data_.data;

  // 周期性打印发送帧，观测编码是否正确
  static int i = 0;
  i++;
  if (i == 50) {
    i = 0;
    engineer_print_transmit_data(tx_data);
  }

  std::memcpy(buffer_, &tx_data, sizeof(EngineerTransmitData));
  if (!device_.send_data(buffer_, sizeof(EngineerTransmitData))) {
    RCLCPP_ERROR(this->get_logger(),
                 "[scope=usb_cdc_node][status=failed] Failed to send data");
    publish_error(make_error(UsbCdcErrc::SendFailed, "Failed to send data"));
  }
}

void UsbCdcNode::publish_timer_callback() {
  std::scoped_lock<std::mutex> lock(rx_data_mutex_);
  const auto &d = rx_data_.data;
  const rclcpp::Time stamp = this->now();

  // 发布 JointState
  sensor_msgs::msg::JointState joint_states_msg;
  joint_states_msg.header.stamp = stamp;
  joint_states_msg.header.frame_id = "base_link";
  joint_states_msg.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "left_finger_joint"};
  joint_states_msg.position = {
      d.actualJointPosition[0],
      d.actualJointPosition[1],
      d.actualJointPosition[2],
      d.actualJointPosition[3],
      d.actualJointPosition[4],
      d.actualJointPosition[5],
      d.actualJointPosition[6]
  };
  joint_states_msg.velocity.resize(joint_states_msg.name.size(), 0.0);
  joint_states_msg.effort.resize(joint_states_msg.name.size(), 0.0);
  joint_states_pub_->publish(joint_states_msg);

  // 发布 verbose 版关节信息，带模式标记
  engineer_interfaces::msg::Joints joint_states_verbose_msg;
  joint_states_verbose_msg.joints.resize(6);
  for (size_t i = 0; i < 6; ++i) {
    joint_states_verbose_msg.joints[i].name = "joint" + std::to_string(i + 1);
    joint_states_verbose_msg.joints[i].position = d.actualJointPosition[i];
    joint_states_verbose_msg.joints[i].velocity = 0;
    joint_states_verbose_msg.joints[i].effort = 0;
    joint_states_verbose_msg.joints[i].mode = "verbose";
  }
  joint_states_verbose_pub_->publish(joint_states_verbose_msg);

  // 发布 custom 关节消息,用于自定义控制
  engineer_interfaces::msg::Joints joint_states_custom_msg;
  joint_states_custom_msg.joints.resize(6);
  for (size_t i = 0; i < 6; ++i) {
    joint_states_custom_msg.joints[i].name = "joint" + std::to_string(i + 1);
    joint_states_custom_msg.joints[i].position = d.customJointPosition[i];
    joint_states_custom_msg.joints[i].velocity = 0;
    joint_states_custom_msg.joints[i].effort = 0;
    joint_states_custom_msg.joints[i].mode = "custom";
  }
  joint_states_custom_pub_->publish(joint_states_custom_msg);

  // 发布 HFSM 意图，驱动上层状态机
  engineer_interfaces::msg::HFSMIntent hfsm_intent_msg;
  hfsm_intent_msg.stamp = this->now();
  hfsm_intent_msg.intent_id = d.currentIntent;
  hfsm_intent_pub_->publish(hfsm_intent_msg);
}

// ============================================================================
//  ROS subscriptions
// ============================================================================
void UsbCdcNode::jointCommandCallback(
    const engineer_interfaces::msg::Joints::SharedPtr msg) {
  std::scoped_lock<std::mutex> lock(tx_data_mutex_);
  send_enabled_ = true; // 接受到控制命令，send_enabled 允许发送

  // 将 JointCommand 映射到目标关节位置/速度，超出部分清零
  for (const auto &joint : msg->joints) {
    auto it = joint_map_.find(joint.name);
    if (it != joint_map_.end()) {
      const size_t index = it->second;
      tx_data_.data.targetJointPosition[index] = joint.position;
      tx_data_.data.targetJointVelocity[index] = joint.velocity;
    }
  }
}

void UsbCdcNode::GripperCommandCallback(
    const engineer_interfaces::msg::GripperCommand::SharedPtr msg) {
  std::scoped_lock<std::mutex> lock(tx_data_mutex_);
  // 覆盖夹爪指令，随发送定时器一起输出
  tx_data_.data.gripperCommand = msg->gripper_command;
}
}; // namespace usb_cdc

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(usb_cdc::UsbCdcNode)

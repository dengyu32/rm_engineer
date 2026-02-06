// USB CDC
#include "usb_cdc/usb_cdc_node.hpp"
#include "error_code_utils/app_error.hpp"
#include "usb_cdc/packet.hpp"

// uitls
#include "log_utils/log.hpp"

// C++
#include <cstddef>
#include <exception>
#include <thread>
#include <sys/types.h>
#include <chrono>
#include <string>

// ROS messages
#include <engineer_interfaces/msg/detail/intent__struct.hpp>

// ROS2
#include <rclcpp/logging.hpp>

namespace usb_cdc {

using namespace std::chrono_literals;

// ============================================================================
//  ctor
// ============================================================================
UsbCdcNode::UsbCdcNode(const rclcpp::NodeOptions &options)
    : rclcpp::Node("usb_cdc", options), 
      device_(parser_), 
      logger_(this->get_logger()),
      config_(UsbCdcConfig::Load(*this)) {

  // 解析器注册 + 打开设备，失败时直接抛出阻止组件化加载
  this->init_parser();
  if (!this->try_open_device()) {
    RCLCPP_WARN(this->get_logger(),
                " [FAILED] initial open FAILED, entering wait mode ");
    device_.request_reconnect();
  }

  // ros init 
  initRosInterfaces();

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

  // log 
  log_utils::init_console_logger("core");
  LOGI("\n{}",config_.summary());
  RCLCPP_INFO(logger_, "USB_CDC_NODE START!!!");
}

void UsbCdcNode::initRosInterfaces() {
  // Publisher
  intent_pub_ = this->create_publisher<engineer_interfaces::msg::Intent>(
      config_.intent_out_topic, rclcpp::QoS(10));
  joint_states_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
      config_.joint_states_topic, rclcpp::QoS(10));
  joint_states_custom_pub_ = this->create_publisher<engineer_interfaces::msg::Joints>(
      config_.joint_states_custom_topic, rclcpp::QoS(10));
  joint_states_verbose_pub_ = this->create_publisher<engineer_interfaces::msg::Joints>(
      config_.joint_states_verbose_topic, rclcpp::QoS(10));
  // Subscriber
  intent_sub_ = this->create_subscription<engineer_interfaces::msg::Intent>(
      config_.intent_in_topic,rclcpp::QoS(10),
      std::bind(&UsbCdcNode::IntentCallback, this, std::placeholders::_1));
  joint_cmd_sub_ = this->create_subscription<engineer_interfaces::msg::Joints>(
      config_.joint_cmd_topic, rclcpp::QoS(10),
      std::bind(&UsbCdcNode::jointCommandCallback, this, std::placeholders::_1));
  gripper_cmd_sub_ = this->create_subscription<engineer_interfaces::msg::GripperCommand>(
      config_.gripper_cmd_topic, rclcpp::QoS(10),
      std::bind(&UsbCdcNode::GripperCommandCallback, this, std::placeholders::_1));
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
    publish_error(error_code_utils::app::make_app_error(
        error_code_utils::ErrorDomain::COMM,
        error_code_utils::app::CommCode::UsbOpenFailed,
        "usb driver open returned false"));
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(),
                 " [FAILED] usb driver open exception: %s ",
                 e.what());
    publish_error(error_code_utils::app::make_app_error(
        error_code_utils::ErrorDomain::COMM,
        error_code_utils::app::CommCode::UsbOpenException,
        e.what()));
  } catch (...) {
    RCLCPP_ERROR(this->get_logger(),
                 " [FAILED] usb driver open unknown exception ");
    publish_error(error_code_utils::app::make_app_error(
        error_code_utils::ErrorDomain::COMM,
        error_code_utils::app::CommCode::UsbOpenException,
        "usb driver open unknown exception"));
  }

  RCLCPP_ERROR(this->get_logger(),
               " [FAILED] usb driver open failed ");
  publish_error(error_code_utils::app::make_app_error(
      error_code_utils::ErrorDomain::COMM,
      error_code_utils::app::CommCode::UsbOpenFailed,
      "usb driver open FAILED"));
  return false;
}


// ============================================================================
//  Parser callbacks
// ============================================================================
void UsbCdcNode::engineer_handle_packet(const std::byte *data, size_t size) {
  std::scoped_lock<std::mutex> lock(rx_data_mutex_);
  if (size < sizeof(EngineerReceiveData)) {
    RCLCPP_ERROR(this->get_logger(),
                 " [ERROR] Received packet too small, expected at least %zu, got %zu ",
                 sizeof(EngineerReceiveData), size);
    publish_error(error_code_utils::app::make_app_error(
        error_code_utils::ErrorDomain::COMM,
        error_code_utils::app::CommCode::UsbPacketTooSmall,
        "Received packet too small",
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
  const bool device_open = device_.is_open();
  if (!device_open) {
    if (last_device_open_) {
      RCLCPP_WARN(this->get_logger(),
                  " [DISCONNECT] USB device disconnected, waiting to reconnect ");
      send_enabled_ = false;
    }
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         " [WAITING] waiting for USB reconnect ");
    last_device_open_ = false;
    return;
  }
  last_device_open_ = true;
  // TODO: 取消注释
  // if (!send_enabled_) { // 未收到控制指令前不发串口数据
  //   RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
  //                        " [blocked] waiting for joint_commands before sending");
  //   return;
  // }

  // 按协议填充发送帧
  EngineerTransmitData tx_data;
  tx_data.header.id = 0x02;
  tx_data.header.len = sizeof(decltype(tx_data.data));
  tx_data.header.sof = HeaderFrame::SoF();
  tx_data.eof = HeaderFrame::EoF();
  tx_data.data = tx_data_.data;
  tx_data.header.crc = calc_crc_len_id_payload(
      tx_data.header.len, tx_data.header.id, &tx_data.data);

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
                 " [FAILED] faild to send data ");
    publish_error(error_code_utils::app::make_app_error(
        error_code_utils::ErrorDomain::COMM,
        error_code_utils::app::CommCode::UsbSendFailed,
        "FAILED to send data"));
  }
}

void UsbCdcNode::publish_timer_callback() {
  std::scoped_lock<std::mutex> lock(rx_data_mutex_);
  const auto &d = rx_data_.data;
  const rclcpp::Time stamp = this->now();

  // 发布 JointState
  sensor_msgs::msg::JointState joint_states;
  joint_states.header.stamp = stamp;
  joint_states.header.frame_id = "base_link";
  joint_states.name = config_.joint_names;
  joint_states.name.push_back("left_finger_joint");
  joint_states.position = {
      d.actualJointPosition[0],
      d.actualJointPosition[1],
      d.actualJointPosition[2],
      d.actualJointPosition[3],
      d.actualJointPosition[4],
      d.actualJointPosition[5],
      d.actualJointPosition[6]
  };
  joint_states.velocity.resize(joint_states.name.size(), 0.0);
  joint_states.effort.resize(joint_states.name.size(), 0.0);
  joint_states_pub_->publish(joint_states);

  // 发布 verbose 版关节信息，带模式标记
  engineer_interfaces::msg::Joints joint_states_verbose;
  joint_states_verbose.joints.resize(6);
  for (size_t i = 0; i < 6; ++i) {
    joint_states_verbose.joints[i].name = config_.joint_names[i];
    joint_states_verbose.joints[i].position = d.actualJointPosition[i];
    joint_states_verbose.joints[i].velocity = 0;
    joint_states_verbose.joints[i].effort = 0;
    joint_states_verbose.joints[i].mode = "verbose";
  }
  joint_states_verbose_pub_->publish(joint_states_verbose);

  // 发布 custom 关节消息,用于自定义控制
  engineer_interfaces::msg::Joints joint_states_custom;
  joint_states_custom.joints.resize(6);
  for (size_t i = 0; i < 6; ++i) {
    joint_states_custom.joints[i].name = config_.joint_names[i];
    joint_states_custom.joints[i].position = d.customJointPosition[i];
    joint_states_custom.joints[i].velocity = 0;
    joint_states_custom.joints[i].effort = 0;
    joint_states_custom.joints[i].mode = "custom";
  }
  joint_states_custom_pub_->publish(joint_states_custom);

  // 发布 HFSM 意图，驱动上层状态机
  engineer_interfaces::msg::Intent intent;
  intent.stamp = this->now();
  intent.intent_id = d.IntentStatus;
  intent.intent_ack = d.IntentAck;
  intent_pub_->publish(intent);
}

// ============================================================================
//  ROS subscriptions
// ============================================================================
void UsbCdcNode::IntentCallback(
    const engineer_interfaces::msg::Intent::SharedPtr msg) {
  std::scoped_lock<std::mutex> lock(tx_data_mutex_);
  // 状态机任务完成 finish置1
  tx_data_.data.IntentFinish = msg->intent_finish;
}

void UsbCdcNode::jointCommandCallback(
    const engineer_interfaces::msg::Joints::SharedPtr msg) {
  std::scoped_lock<std::mutex> lock(tx_data_mutex_);
  send_enabled_ = true; // 接受到控制命令，send_enabled 允许发送

  // 将 JointCommand 映射到目标关节位置/速度，超出部分清零
  for (const auto &joint : msg->joints) {
    auto it = config_.joint_index.find(joint.name);
    if (it != config_.joint_index.end()) {
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

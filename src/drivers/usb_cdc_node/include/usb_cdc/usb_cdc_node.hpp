#pragma once

// USB CDC
#include "usb_cdc/packet.hpp"
#include "usb_cdc/usb_cdc_driver.hpp"
#include "usb_cdc/config.hpp"
#include "error_code_utils/error_bus.hpp"

// ROS messages
#include <engineer_interfaces/msg/gripper_command.hpp>
#include <engineer_interfaces/msg/intent.hpp>
#include <engineer_interfaces/msg/joints.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

// ROS2
#include <rclcpp/callback_group.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

// TF2
#include <tf2/LinearMath/Transform.hpp>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/buffer_interface.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

// C++
#include <atomic>
#include <cstdint>

namespace usb_cdc {

// ============================================================================
//  UsbCdcNode
// ----------------------------------------------------------------------------
//  - USB CDC 设备的 ROS2 节点封装，负责收发、解析与状态发布
//  - 组合 Device / DeviceParser，绑定自定义包 ID → 回调处理
//  - 订阅关节/夹爪指令，发布 HFSM 意图与 JointState/Joints
//  - 独立读写线程与定时器，保障底层轮询不阻塞 ROS2 executor
// ============================================================================
class UsbCdcNode : public rclcpp::Node {
public:
  // -----------------------------------------------------------------------
  //  Lifecycle
  // -----------------------------------------------------------------------
  UsbCdcNode(const rclcpp::NodeOptions &options);

  ~UsbCdcNode() {
    if (thread_.joinable()) {
      running_ = false;
      thread_.join();
    }
  }

  // -----------------------------------------------------------------------
  //  Device control
  // -----------------------------------------------------------------------
  bool try_open_device();
  void set_error_bus(const std::shared_ptr<error_code_utils::ErrorBus> &bus);

private:
  // -----------------------------------------------------------------------
  //  Parser setup
  // -----------------------------------------------------------------------
  void init_parser() {
    parser_.register_parser(0x01, std::bind(&UsbCdcNode::engineer_handle_packet,
                                            this, std::placeholders::_1,
                                            std::placeholders::_2));
  }
  void initRosInterfaces();

  void engineer_handle_packet(const std::byte *data, size_t size);

  // -----------------------------------------------------------------------
  //  Timers & callbacks
  // -----------------------------------------------------------------------
  void publish_timer_callback();
  void IntentCallback(const engineer_interfaces::msg::Intent::SharedPtr msg);
  void jointCommandCallback(
      const engineer_interfaces::msg::Joints::SharedPtr msg);
  void GripperCommandCallback(
      const engineer_interfaces::msg::GripperCommand::SharedPtr msg);
  void send_timer_callback();
  void publish_error(const error_code_utils::Error &err) const;

  // Protocol
  DeviceParser parser_;
  Device device_;
  uint8_t buffer_[256]; // USB 读缓冲区（256 字节原始数据）

  // ROS interfaces
  rclcpp::Publisher<engineer_interfaces::msg::Intent>::SharedPtr intent_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;
  rclcpp::Publisher<engineer_interfaces::msg::Joints>::SharedPtr joint_states_custom_pub_;
  rclcpp::Publisher<engineer_interfaces::msg::Joints>::SharedPtr joint_states_verbose_pub_;

  rclcpp::Subscription<engineer_interfaces::msg::Intent>::SharedPtr intent_sub_;
  rclcpp::Subscription<engineer_interfaces::msg::Joints>::SharedPtr joint_cmd_sub_;
  rclcpp::Subscription<engineer_interfaces::msg::GripperCommand>::SharedPtr gripper_cmd_sub_;

  rclcpp::TimerBase::SharedPtr send_timer_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  rclcpp::Logger logger_;

  // Buffers
  EngineerTransmitData tx_data_{};
  EngineerReceiveData rx_data_{};
  std::mutex tx_data_mutex_; // 保护发送缓冲
  std::mutex rx_data_mutex_; // 保护接收缓冲

  // Runtime state
  std::atomic_bool running_;
  bool send_enabled_{false};
  std::atomic_bool last_device_open_{false};
  std::thread thread_; // 底层读写循环线程

  // Config
  UsbCdcConfig config_;

  std::shared_ptr<error_code_utils::ErrorBus> error_bus_;
};

} // namespace usb_cdc

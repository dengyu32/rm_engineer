// ============================================================================
//  usb_cdc_node.hpp
// ----------------------------------------------------------------------------
//  - 定义 UsbCdcNode 类，封装 USB CDC 设备的 ROS2 节点功能
//  - 组合 Device，完成数据收发、解析与状态发布
//  - 订阅关节/夹爪指令，发布 HFSM 意图与 JointState/Joints
//  - 独立读写线程与定时器，保障底层轮询不阻塞 ROS2
// ============================================================================
#pragma once

// USB CDC
#include "usb_cdc/packet.hpp"
#include "usb_cdc/usb_cdc_config.hpp"
#include "usb_cdc/device.hpp"

// ROS messages
#include <engineer_interfaces/msg/gripper.hpp>
#include <engineer_interfaces/msg/intent.hpp>
#include <engineer_interfaces/msg/joints.hpp>
#include <engineer_interfaces/msg/slots.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

// ROS2
#include <rclcpp/callback_group.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/parameter.hpp>
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
#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <thread>
#include <vector>

namespace usb_cdc {

// ============================================================================
//  EngineerRxState & EngineerTxState
// ============================================================================
struct EngineerRxState {
  std::array<float, 7> actual_joint_position{};
  std::array<float, 6> actual_joint_velocity{};
  std::array<float, 6> custom_joint_position{};
  std::array<uint8_t, 2> real_slot_status{};
  uint8_t intent_status{0};
};

struct EngineerTxState {
  std::array<float, 6> target_joint_position{};
  std::array<float, 6> target_joint_velocity{};
  std::array<float, 6> target_joint_effort{};
  uint8_t target_gripper_command{0};
  std::array<uint8_t, 2> target_slot_status{};
  uint8_t intent_finish{0};
};

// ============================================================================
//  UsbCdcNode
// ----------------------------------------------------------------------------
//  - USB CDC 设备的 ROS2 节点封装，负责收发、解析与状态发布
//  - 组合 Device，完成流式拼帧并回调处理
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

private:
  // -----------------------------------------------------------------------
  //  Device callback setup
  // -----------------------------------------------------------------------
  void init_device_callbacks() {
    device_.set_packet_callback(0x01,
                                std::bind(&UsbCdcNode::engineer_handle_packet,
                                          this, std::placeholders::_1,
                                          std::placeholders::_2));
  }
  void initRosInterfaces();

  void engineer_handle_packet(const std::byte *data, size_t size);

  // -----------------------------------------------------------------------
  //  Timers & callbacks
  // -----------------------------------------------------------------------
  void publish_timer_callback();
  void send_timer_callback();
  rcl_interfaces::msg::SetParametersResult on_set_parameters(
      const std::vector<rclcpp::Parameter> &params);

  void IntentCallback(const engineer_interfaces::msg::Intent::SharedPtr msg);
  void jointCommandCallback(const engineer_interfaces::msg::Joints::SharedPtr msg);
  void GripperCommandCallback(const engineer_interfaces::msg::Gripper::SharedPtr msg);
  void SlotCommandCallback(const engineer_interfaces::msg::Slots::SharedPtr msg);

  // Protocol
  Device device_;
  uint8_t buffer_[256]; // USB 读缓冲区（256 字节原始数据）

  // ROS interfaces
  rclcpp::Publisher<engineer_interfaces::msg::Intent>::SharedPtr intent_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;
  rclcpp::Publisher<engineer_interfaces::msg::Joints>::SharedPtr joint_states_custom_pub_;
  rclcpp::Publisher<engineer_interfaces::msg::Joints>::SharedPtr joint_states_verbose_pub_;
  rclcpp::Publisher<engineer_interfaces::msg::Slots>::SharedPtr slot_states_pub_;

  rclcpp::Subscription<engineer_interfaces::msg::Intent>::SharedPtr intent_sub_;
  rclcpp::Subscription<engineer_interfaces::msg::Joints>::SharedPtr joint_cmd_sub_;
  rclcpp::Subscription<engineer_interfaces::msg::Gripper>::SharedPtr gripper_cmd_sub_;
  rclcpp::Subscription<engineer_interfaces::msg::Slots>::SharedPtr slot_cmd_sub_;

  rclcpp::TimerBase::SharedPtr send_timer_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  rclcpp::Logger logger_;

  // Node state. Packet quantization is only applied at the USB protocol boundary.
  EngineerTxState tx_state_{};
  EngineerRxState rx_state_{};
  std::mutex tx_state_mutex_; // 保护发送状态
  std::mutex rx_state_mutex_; // 保护接收状态

  // Runtime state
  std::atomic_bool running_;
  std::atomic_bool last_device_open_{false};
  std::atomic_bool debug_override_enabled_{false};
  std::atomic<uint8_t> debug_intent_id_{0};
  std::thread thread_; // 底层读写循环线程

  // Config
  UsbCdcConfig config_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      param_callback_handle_;
};

} // namespace usb_cdc

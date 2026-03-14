#pragma once

// USB CDC
#include "usb_cdc/packet.hpp"
#include "usb_cdc/usb_cdc_driver.hpp"
#include "params_utils/param_utils.hpp"

// ROS messages
#include <engineer_interfaces/msg/gripper.hpp>
#include <engineer_interfaces/msg/intent.hpp>
#include <engineer_interfaces/msg/joints.hpp>
#include <engineer_interfaces/msg/slots.hpp>
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
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>

namespace usb_cdc {

// ============================================================================
//  UsbCdcConfig
// ----------------------------------------------------------------------------
//  - USB 设备 + 话题 + 关节布局
// ============================================================================

struct UsbCdcConfig : public params_utils::JointResetConfig,
                      public params_utils::IntentResetConfig,
                      public params_utils::GripperResetConfig {
  //  Device
  int vendor_id{0x0483};
  int product_id{0x5740};

  //  Timing
  int publish_period_ms{33};
  int send_period_ms{20};

  //  Mode
  bool servo_teleop_mode{false};
  std::string slot_state_topic{"/slot_states"};
  std::string slot_cmd_topic{"/slot_cmds"};

  //  API
  static UsbCdcConfig Load(rclcpp::Node &node) {
    using params_utils::detail::declare_get;
    using params_utils::detail::declare_get_checked;
    using params_utils::detail::in_range;

    UsbCdcConfig cfg;

    //  Base layout + topics
    params_utils::JointResetConfig::Load(node, cfg);
    params_utils::IntentResetConfig::Load(node, cfg);
    params_utils::GripperResetConfig::Load(node, cfg);

    //  Device
    declare_get_checked(
        node, "vendor_id", cfg.vendor_id,
        in_range(1, 0xFFFF),
        "must be in [1, 65535]");
    declare_get_checked(
        node, "product_id", cfg.product_id,
        in_range(1, 0xFFFF),
        "must be in [1, 65535]");

    //  Timing
    declare_get_checked(
        node, "publish_period_ms", cfg.publish_period_ms,
        in_range(1, 1000),
        "must be in [1, 1000]");
    declare_get_checked(
        node, "send_period_ms", cfg.send_period_ms,
        in_range(1, 1000),
        "must be in [1, 1000]");

    //  Mode
    declare_get(node, "servo_teleop_mode", cfg.servo_teleop_mode);
    declare_get(node, "slot_state_topic", cfg.slot_state_topic);
    declare_get(node, "slot_cmd_topic", cfg.slot_cmd_topic);

    //  Finalize
    if (cfg.joint_count < 1 || cfg.joint_count > 6) {
      throw std::runtime_error("UsbCdcConfig: joint_count must be in [1, 6]");
    }
    cfg.validate();
    return cfg;
  }
  void validate() const;
  std::string summary() const;
};

inline void UsbCdcConfig::validate() const {
  params_utils::JointResetConfig::validate();
  params_utils::IntentResetConfig::validate();
  params_utils::GripperResetConfig::validate();
}

inline std::string UsbCdcConfig::summary() const {
  std::ostringstream oss;
  oss << "=============================================================================\n";
  oss << " USB CDC Configuration\n\n";

  oss << " Device:\n";
  oss << "   - vendor_id           : " << vendor_id << "\n";
  oss << "   - product_id          : " << product_id << "\n";
  oss << "\n";

  oss << " Timing:\n";
  oss << "   - publish_period_ms   : " << publish_period_ms << "\n";
  oss << "   - send_period_ms      : " << send_period_ms << "\n\n";

  oss << " Mode:\n";
  oss << "   - servo_teleop_mode   : " << (servo_teleop_mode ? "true" : "false") << "\n\n";
  oss << " Slot:\n";
  oss << "   - slot_state_topic    : " << slot_state_topic << "\n";
  oss << "   - slot_cmd_topic      : " << slot_cmd_topic << "\n\n";

  oss << params_utils::JointResetConfig::summary();
  oss << params_utils::IntentResetConfig::summary();
  oss << params_utils::GripperResetConfig::summary();
  oss << "=============================================================================\n";
  return oss.str();
}

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
  void send_timer_callback();

  void IntentCallback(const engineer_interfaces::msg::Intent::SharedPtr msg);
  void jointCommandCallback(const engineer_interfaces::msg::Joints::SharedPtr msg);
  void GripperCommandCallback(const engineer_interfaces::msg::Gripper::SharedPtr msg);
  void SlotCommandCallback(const engineer_interfaces::msg::Slots::SharedPtr msg);
  void publish_error(int code, const char *name, const std::string &message) const;

  // Protocol
  DeviceParser parser_;
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

  // Buffers
  EngineerTransmitData tx_data_{};
  EngineerReceiveData rx_data_{};
  std::mutex tx_data_mutex_; // 保护发送缓冲
  std::mutex rx_data_mutex_; // 保护接收缓冲

  // Runtime state
  std::atomic_bool running_;
  std::atomic_bool last_device_open_{false};
  std::thread thread_; // 底层读写循环线程

  // Config
  UsbCdcConfig config_;
};

} // namespace usb_cdc

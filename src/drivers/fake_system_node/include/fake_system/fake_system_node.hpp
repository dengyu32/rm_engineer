#pragma once

// ROS2
#include <rclcpp/rclcpp.hpp>

// C++
#include <array>
#include <atomic>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

// ROS messages
#include <engineer_interfaces/msg/intent.hpp>
#include <engineer_interfaces/msg/joint.hpp>
#include <engineer_interfaces/msg/joints.hpp>
#include <engineer_interfaces/msg/gripper.hpp>
#include <engineer_interfaces/msg/slots.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "params_utils/param_utils.hpp"

namespace fake_system {

// ============================================================================
//  FakeSystemConfig
// ----------------------------------------------------------------------------
//  - 关节布局 + 话题 + 初始化参数
// ============================================================================

struct FakeSystemConfig : public params_utils::JointResetConfig,
                          public params_utils::IntentResetConfig,
                          public params_utils::GripperResetConfig {
  //  Timing
  int publish_period_ms{33};

  //  Initialization
  std::vector<double> initial_joint_positions{0, 0, 0, 0, 0, 0};
  double initial_gripper_position{0.0};
  std::string slot_cmd_topic{"/slot_cmds"};
  std::string slot_state_topic{"/slot_states"};
  std::vector<bool> initial_slot_status{false, false};

  //  API
  static FakeSystemConfig Load(rclcpp::Node &node) {
    FakeSystemConfig cfg;

    //  Load base layout + topics
    params_utils::JointResetConfig::Load(node, cfg);
    params_utils::IntentResetConfig::Load(node, cfg);
    params_utils::GripperResetConfig::Load(node, cfg);

    using params_utils::detail::declare_get;
    using params_utils::detail::declare_get_checked;

    //  Timing
    declare_get_checked(
        node, "publish_period_ms", cfg.publish_period_ms,
        [](int v) { return v > 0; }, "must be > 0");

    //  Initialization
    cfg.initial_joint_positions.assign(static_cast<std::size_t>(cfg.joint_count), 0.0);

    declare_get(node, "initial_joint_positions", cfg.initial_joint_positions);
    declare_get_checked(
        node, "initial_gripper_position", cfg.initial_gripper_position,
        [](double v) { return v >= 0.0 && v <= 0.03; }, "must be in [0, 0.03]");
    declare_get(node, "slot_cmd_topic", cfg.slot_cmd_topic);
    declare_get(node, "slot_state_topic", cfg.slot_state_topic);
    declare_get(node, "initial_slot_status", cfg.initial_slot_status);
    if (cfg.initial_slot_status.size() != 2U) {
      std::ostringstream oss;
      oss << "FakeSystemConfig: initial_slot_status must have 2 elements, got "
          << cfg.initial_slot_status.size();
      throw std::runtime_error(oss.str());
    }

    //  Finalize
    cfg.validate();
    return cfg;
  }
  void validate() const;
  std::string summary() const;
};

inline void FakeSystemConfig::validate() const {
  params_utils::JointResetConfig::validate();
  params_utils::IntentResetConfig::validate();
  params_utils::GripperResetConfig::validate();

  if (initial_joint_positions.size() != static_cast<std::size_t>(joint_count)) {
    std::ostringstream oss;
    oss << "FakeSystemConfig: initial_joint_positions must have " << joint_count
        << " elements, got "
        << initial_joint_positions.size();
    throw std::runtime_error(oss.str());
  }
}

inline std::string FakeSystemConfig::summary() const {
  std::ostringstream oss;

  oss << "=============================================================================\n";
  oss << " FakeSystemNode Configuration\n";

  oss << " Timing:\n";
  oss << "   - publish_period_ms           : " << publish_period_ms << "\n";

  oss << params_utils::JointResetConfig::summary();
  oss << params_utils::IntentResetConfig::summary();
  oss << params_utils::GripperResetConfig::summary();

  oss << " Initialization:\n";
  oss << "   - initial_joint_positions     : [";
  for (std::size_t i = 0; i < initial_joint_positions.size(); ++i) {
    oss << initial_joint_positions[i];
    if (i + 1 < initial_joint_positions.size()) {
      oss << ", ";
    }
  }
  oss << "]\n";
  oss << "   - initial_gripper_position    : " << initial_gripper_position << "\n\n";
  oss << " Slot:\n";
  oss << "   - slot_cmd_topic              : " << slot_cmd_topic << "\n";
  oss << "   - slot_state_topic            : " << slot_state_topic << "\n";
  oss << "   - initial_slot_status         : ["
      << (initial_slot_status[0] ? 1 : 0) << ", "
      << (initial_slot_status[1] ? 1 : 0) << "]\n\n";
  oss << "=============================================================================\n\n";

  return oss.str();
}

// ============================================================================
//  FakeSystemNode
// ----------------------------------------------------------------------------
//  - 提供伪造关节/夹爪状态的 ROS2 节点，便于上层联调
//  - 依赖 rclcpp 定时器与发布/订阅接口
//  - 订阅关节与夹爪指令，生成 JointState / Joints 输出
//  - 人为终端发布 IntentStatus，模拟调度状态机意图
//  - 维护简易内存态，替代真实硬件反馈
// ============================================================================

class FakeSystemNode : public rclcpp::Node {
public:
  // -----------------------------------------------------------------------
  //  Lifecycle
  // -----------------------------------------------------------------------
  explicit FakeSystemNode(const rclcpp::NodeOptions &options);

private:
  // -----------------------------------------------------------------------
  //  ROS interfaces
  // -----------------------------------------------------------------------
  void initRosInterfaces();

  // -----------------------------------------------------------------------
  //  ROS callbacks
  // -----------------------------------------------------------------------
  void execute(const engineer_interfaces::msg::Joints::SharedPtr msg);
  void execute(const engineer_interfaces::msg::Gripper::SharedPtr msg);
  void execute(const engineer_interfaces::msg::Slots::SharedPtr msg);
  void intent_feedback_callback(const engineer_interfaces::msg::Intent::SharedPtr msg);
  void publish_timer_callback();
  void publish_slot_states_now();
  void publish_error(int code, const char *name, const std::string &message) const;

  // Parameters / states
  FakeSystemConfig config_;
  rclcpp::Logger logger_;

  std::vector<double> fake_joint_positions_;
  double fake_gripper_position_{0.0};
  std::array<bool, 2> fake_slot_status_{{false, false}};
  std::mutex fake_mutex_; // 保护仿真关节/夹爪状态

  // 动态配置 runtime_config_
  std::atomic<uint8_t> fake_intent_id_{0};

  // 动态回调句柄
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  // ROS interfaces
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;
  rclcpp::Publisher<engineer_interfaces::msg::Joints>::SharedPtr joint_states_custom_pub_;
  rclcpp::Publisher<engineer_interfaces::msg::Joints>::SharedPtr joint_states_verbose_pub_;
  rclcpp::Publisher<engineer_interfaces::msg::Intent>::SharedPtr intent_cmd_pub_; // 人为调度（schedule）状态机意图
  rclcpp::Publisher<engineer_interfaces::msg::Slots>::SharedPtr slot_states_pub_;

  rclcpp::Subscription<engineer_interfaces::msg::Intent>::SharedPtr intent_fb_sub_; // 状态机反馈
  rclcpp::Subscription<engineer_interfaces::msg::Joints>::SharedPtr joint_cmd_sub_;
  rclcpp::Subscription<engineer_interfaces::msg::Gripper>::SharedPtr gripper_cmd_sub;
  rclcpp::Subscription<engineer_interfaces::msg::Slots>::SharedPtr slot_cmd_sub_;
};
} // namespace fake_system

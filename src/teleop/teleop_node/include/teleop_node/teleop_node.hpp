#pragma once

// Rely

//< C++
#include <algorithm>
#include <cstdint>
#include <mutex>
#include <sstream>
#include <string>
#include <unordered_map>
#include <memory>
#include <vector>

//< ROS 2
#include <rclcpp/rclcpp.hpp>

//< Engineer Interfaces
#include <engineer_interfaces/msg/intent.hpp>
#include <engineer_interfaces/msg/joints.hpp>

//< Internal hpp
#include "teleop_node/collision_checker.hpp"
#include "params_utils/param_utils.hpp"

namespace engineer_teleop {

// ============================================================================
//  TeleopConfig
// ----------------------------------------------------------------------------
//  - 话题 / MoveIt / Teleop 参数
// ============================================================================

struct TeleopConfig : public params_utils::IntentResetConfig,
                      public params_utils::JointResetConfig,
                      public params_utils::MoveItResetConfig {
  double deadband_rad{1e-4};
  int teleop_intent_id{11};

  static TeleopConfig load(rclcpp::Node &node) {
    TeleopConfig cfg;

    params_utils::IntentResetConfig::Load(node, cfg);
    params_utils::JointResetConfig::Load(node, cfg);
    params_utils::MoveItResetConfig::Load(node, cfg);

    params_utils::detail::declare_get(node, "command_deadband_rad", cfg.deadband_rad);
    params_utils::detail::declare_get_checked(
        node, "teleop_intent_id", cfg.teleop_intent_id,
        [](int v) { return v >= 0 && v <= 255; },
        "must be in [0, 255]");

    cfg.validate();
    return cfg;
  }
  void validate() const {
    params_utils::IntentResetConfig::validate();
    params_utils::JointResetConfig::validate();
    params_utils::MoveItResetConfig::validate();
  }

  std::string summary() const {
    std::ostringstream oss;
    oss << "=============================================================================\n";
    oss << " Teleop Configuration\n\n";
    oss << " Teleop:\n";
    oss << "   - command_deadband_rad : " << deadband_rad << "\n";
    oss << "   - teleop_intent_id     : " << teleop_intent_id << "\n\n";
    oss << params_utils::IntentResetConfig::summary();
    oss << params_utils::JointResetConfig::summary();
    oss << params_utils::MoveItResetConfig::summary();
    oss << "=============================================================================\n";
    return oss.str();
  }
};

// ============================================================================
//   TeleopNode
// ----------------------------------------------------------------------------
//   - ROS2 订阅与发布 
//      - 关节状态 Verbose/Custom 及命令 Teleop Intent
//      - 发布 Cmd 控制命令
//   - joint 状态缓存与处理
//   - 动态安全位置 last_safe_joints_ 机制
//   - 通过 CollisionChecker 进行安全检查
// ============================================================================

class TeleopNode : public rclcpp::Node {
public:
  TeleopNode();

private:
  //< ROS 2 Topic Callbacks
  void verboseCallback(const engineer_interfaces::msg::Joints::SharedPtr msg);
  void customCallback(const engineer_interfaces::msg::Joints::SharedPtr msg);
  void intentCallback(const engineer_interfaces::msg::Intent::SharedPtr msg);
  
  //< Internal Func
  bool buildTargetFromCustom(
      const engineer_interfaces::msg::Joints::SharedPtr &msg,
      std::vector<double> &target_positions) const;
  void publishJointCommand(const std::vector<double> &positions, const std::string &mode_tag);

  bool hasCompleteCurrentJoints() const {
    return std::all_of(has_current_joints_.begin(), has_current_joints_.end(),
                     [](bool v) { return v; });
  }

  bool isStateSelfCollisionFree(const std::vector<double> &positions) {
    if (!collision_checker_) {
      collision_checker_ = std::make_unique<CollisionChecker>(shared_from_this(), group_name_, joint_names_);
    }
    return collision_checker_->isSelfCollisionFree(positions);
  }

  //< Topic Params
  std::string custom_topic_;
  std::string verbose_topic_;
  std::string output_topic_;
  std::string intent_cmd_topic_;

  //< Robotic Arm Params
  std::string group_name_;
  std::vector<std::string> joint_names_;
  std::unordered_map<std::string, std::size_t> joint_name_to_index_;
  std::vector<double> current_joints_;
  std::vector<bool> has_current_joints_;

  std::vector<double> last_safe_joints_;
  bool has_last_safe_{false};

  //< Other Params
  double deadband_rad_{1e-4};
  uint8_t teleop_intent_id_{11};
  bool teleop_enabled_{false};
  mutable std::mutex state_mutex_; // 维护共享状态 current_joints_ / has_current_joints_ / last_safe_joints_ / has_last_safe_

  TeleopConfig config_;

  //< ROS 2 Publishers & Subscribers
  rclcpp::Publisher<engineer_interfaces::msg::Joints>::SharedPtr joint_cmd_pub_;
  rclcpp::Subscription<engineer_interfaces::msg::Intent>::SharedPtr intent_sub_;
  rclcpp::Subscription<engineer_interfaces::msg::Joints>::SharedPtr custom_sub_;
  rclcpp::Subscription<engineer_interfaces::msg::Joints>::SharedPtr verbose_sub_;

  //< Collision Checker
  std::unique_ptr<CollisionChecker> collision_checker_;
};

} // namespace engineer_teleop

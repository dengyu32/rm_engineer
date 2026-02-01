// Strongly-typed configuration for ArmServoNode with validation and summary dump.
#pragma once

#include <rclcpp/rclcpp.hpp>

#include <param_utils/param_snapshot.hpp>

#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace arm_servo {

struct ArmServoConfig {
  std::string servo_out_topic{"/moveit_servo/joint_trajectory"};
  std::string output_topic{"/joint_commands"};
  bool use_last_point_only{true};
  int publish_period_ms{30};   // 0 = no throttle

  int joint_count{6};
  std::vector<std::string> joint_names{};
  std::unordered_map<std::string, std::size_t> joint_index{};

  std::vector<param_utils::ParamKV> params_snapshot{};

  static ArmServoConfig Load(rclcpp::Node &node);

private:
  // 返回前指定位vector容器
  static std::vector<std::string> default_joint_names(int count) {
    std::vector<std::string> names;
    if (count <= 0) {
      return names;
    }
    names.reserve(static_cast<std::size_t>(count));
    for (int i = 1; i <= count; ++i) {
      // emplace_back 向末尾添加新元素
      names.emplace_back("joint" + std::to_string(i));
    }
    return names;
  }

  static void require_true(bool cond, const std::string &name,
                           const std::string &msg) {
    if (cond) {
      return;
    }
    throw std::runtime_error("param '" + name + "' " + msg);
  }

  // to confirm // 确定参数范围 // 添加约束
  void validate() const {
    if (servo_out_topic.empty()) {
      throw std::runtime_error("param 'servo_out_topic' must not be empty");
    }
    if (output_topic.empty()) {
      throw std::runtime_error("param 'output_topic' must not be empty");
    }
    if (publish_period_ms < 0 || publish_period_ms > 1000) {
      throw std::runtime_error("param 'publish_period_ms' must be in [0, 1000]");
    }
    if (joint_count < 1 || joint_count > 12) {
      throw std::runtime_error("param 'joint_count' must be in [1, 12]");
    }
    require_true(static_cast<int>(joint_names.size()) == joint_count,
                 "joint_names", "size must match joint_count");
    require_true(joint_index.size() == joint_names.size(),
                 "joint_index", "index map size mismatch");
    for (const auto &name : joint_names) {
      if (name.empty()) {
        throw std::runtime_error("param 'joint_names' must not contain empty strings");
      }
    }
  }
};

// 重载函数
inline ArmServoConfig ArmServoConfig::Load(rclcpp::Node &node) {
  ArmServoConfig cfg;
  auto declare_get = [&](const std::string& name, auto& value) {
    node.declare_parameter(name, value);
    node.get_parameter(name, value);
  };

  declare_get("servo_out_topic", cfg.servo_out_topic);
  declare_get("output_topic", cfg.output_topic);
  declare_get("publish_period_ms", cfg.publish_period_ms);
  declare_get("use_last_point_only", cfg.use_last_point_only);
  declare_get("joint_count", cfg.joint_count);

  cfg.joint_names = default_joint_names(cfg.joint_count);
  declare_get("joint_names", cfg.joint_names);
  if (cfg.joint_names.empty()) {
    cfg.joint_names = default_joint_names(cfg.joint_count);
  }

  cfg.joint_index.clear();
  cfg.joint_index.reserve(cfg.joint_names.size());
  for (std::size_t i = 0; i < cfg.joint_names.size(); ++i) {
    cfg.joint_index.emplace(cfg.joint_names[i], i);
  }

  cfg.validate();
  cfg.params_snapshot = param_utils::CollectAllParams(node);
  return cfg;
}

} // namespace arm_servo

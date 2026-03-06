#pragma once

// ROS2
#include <geometry_msgs/msg/detail/twist_stamped__struct.hpp>
#include <rclcpp/rclcpp.hpp>

// C++
#include <chrono>
#include <string>
#include <array>
#include <mutex>

// Msgs
#include <engineer_interfaces/msg/joints.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <unordered_map>

#include "top_hfsm/config.hpp"
#include "top_hfsm/util/async_dispatcher.hpp"

namespace top_hfsm::executors {

// ============================================================================
//  MoveItServoClient
// ----------------------------------------------------------------------------
//  - MoveItServo 客户端封装，管理其开启/关闭
//  - 订阅 custom 消息，SERVO 时发布 /detla_cmds
//  - 暴露进度与上下文查询接口给 HFSM
// ============================================================================
class MoveItServoClient {
public:
  // -----------------------------------------------------------------------
  //  Lifecycle
  // -----------------------------------------------------------------------
  MoveItServoClient(rclcpp::Node &node, const MoveItServoClientConfig &config);

  // -----------------------------------------------------------------------
  //  Async requests（非阻塞）
  // -----------------------------------------------------------------------
  void requestStartServo(top_hfsm::AsyncDispatcher &dispatcher);
  void requestPauseServo(top_hfsm::AsyncDispatcher &dispatcher);
  bool servoPending() const;
  top_hfsm::TaskStatus lastServoStatus() const;
  std::string lastServoMsg() const;
  void applyAsyncResult(const top_hfsm::AsyncDispatcher::Result &r);

private:
  // -----------------------------------------------------------------------
  //  helpers (同步实现，给后台线程用)
  // -----------------------------------------------------------------------
  top_hfsm::AsyncDispatcher::Result runStartServoJob();
  top_hfsm::AsyncDispatcher::Result runPauseServoJob();
  bool startServoSync();
  bool pauseServoSync();
  bool unpauseServoSync();
  bool stopServoSync();
  bool resetServoStatusSync();

private:
  // -----------------------------------------------------------------------
  //  Setup
  // -----------------------------------------------------------------------
  void initParameters();
  void initRosInterfaces();
  void initRosServices();
  // -----------------------------------------------------------------------
  //  call
  // -----------------------------------------------------------------------
  struct CallResult {
    top_hfsm::TaskStatus status;
    std::string message;
  };

  static CallResult call_trigger_service(
      rclcpp::Node &node,
      const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr &client,
      const std::string &srv_name,
      std::chrono::milliseconds wait_timeout,
      std::chrono::milliseconds call_timeout);
  static CallResult call_empty_service(
      rclcpp::Node &node,
      const rclcpp::Client<std_srvs::srv::Empty>::SharedPtr &client,
      const std::string &srv_name,
      std::chrono::milliseconds wait_timeout,
      std::chrono::milliseconds call_timeout);

  // -----------------------------------------------------------------------
  //  delta cmds publish
  // -----------------------------------------------------------------------
  void customJointCallback(engineer_interfaces::msg::Joints::SharedPtr msg);
  void verboseJointCallback(engineer_interfaces::msg::Joints::SharedPtr msg);

private:
  // Node config
  rclcpp::Node &node_;
  MoveItServoClientConfig config_;
  rclcpp::Logger logger_;

  // Clients
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_servo_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr unpause_servo_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr pause_servo_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_servo_client_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr reset_status_client_;

  // Services
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pause_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr unpause_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_status_srv_;

  // Publishers & Subscribers
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr delta_twist_pub_;
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_jog_pub_;
  rclcpp::Subscription<engineer_interfaces::msg::Joints>::SharedPtr joint_states_custom_sub_;
  rclcpp::Subscription<engineer_interfaces::msg::Joints>::SharedPtr joint_states_verbose_sub_;

  // Others
  const std::unordered_map<std::string, size_t> joint_name_to_index_map_{
      {"joint1", 0}, {"joint2", 1}, {"joint3", 2},
      {"joint4", 3}, {"joint5", 4}, {"joint6", 5}};
  std::array<double, 6> current_joints_{};
  std::mutex current_joint_mutex_;

  // Async state
  mutable std::mutex servo_mutex_;
  bool servo_pending_{false};
  top_hfsm::TaskStatus last_servo_status_{top_hfsm::TaskStatus::Success};
  std::string last_servo_msg_;
};

} // top_hfsm::executors

// Executors
#include "top_hfsm/executors/moveit_servo_client.hpp"
#include "param_utils/param_snapshot.hpp"

// C++
#include <memory>
#include <future>

namespace top_hfsm::executors {

// ============================================================================
// ctor
// ============================================================================

MoveItServoClient::MoveItServoClient(rclcpp::Node &node, const MoveItServoClientConfig &config)
    : node_(node), config_(config), logger_(node.get_logger()) {
  // --------------------------------------------------------------------------
  //  ROS interfaces
  // --------------------------------------------------------------------------
  initRosInterfaces();

  // --------------------------------------------------------------------------
  //  ROS services (output)
  // --------------------------------------------------------------------------
  initRosServices();

  param_utils::LogSnapshot(node_.get_logger(), config_.params_snapshot,
                           "[MOVEIT_SERVO_CLIENT][param] ");
  RCLCPP_INFO(logger_, "[MOVEIT_SERVO_CLIENT] started");
}

// ============================================================================
//  Setup
// ============================================================================
void MoveItServoClient::initRosInterfaces() {
  // Clients
  start_servo_client_ = node_.create_client<std_srvs::srv::Trigger>(config_.servo_start_service);
  stop_servo_client_ = node_.create_client<std_srvs::srv::Trigger>(config_.servo_stop_service);
  pause_servo_client_ = node_.create_client<std_srvs::srv::Trigger>(config_.servo_pause_service);
  unpause_servo_client_ = node_.create_client<std_srvs::srv::Trigger>(config_.servo_unpause_service);
  reset_status_client_ = node_.create_client<std_srvs::srv::Empty>(config_.servo_reset_status_service);
  
  // Others
  delta_twist_pub_ = node_.create_publisher<geometry_msgs::msg::TwistStamped>(
      config_.delta_twist_topic, rclcpp::QoS(10));
  joint_jog_pub_ = node_.create_publisher<control_msgs::msg::JointJog>(
      config_.delta_joint_topic, rclcpp::QoS(10));

  joint_states_custom_sub_ = node_.create_subscription<engineer_interfaces::msg::Joints>(
      config_.joint_states_custom_topic, rclcpp::QoS(10),
      std::bind(&MoveItServoClient::customJointCallback, this, std::placeholders::_1));
  joint_states_verbose_sub_ = node_.create_subscription<engineer_interfaces::msg::Joints>(
      config_.joint_states_verbose_topic, rclcpp::QoS(10),
      std::bind(&MoveItServoClient::verboseJointCallback, this, std::placeholders::_1));
}

void MoveItServoClient::initRosServices() {
  start_srv_ = node_.create_service<std_srvs::srv::Trigger>(
      "moveit_servo_client/start",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        (void)req;
        bool ok = this->startServoSync();
        res->success = ok;
        res->message =
            ok ? "Started MoveIt Servo." : "Failed to start MoveIt Servo.";
      });
  stop_srv_ = node_.create_service<std_srvs::srv::Trigger>(
      "moveit_servo_client/stop",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        (void)req;
        bool ok = this->stopServoSync();
        res->success = ok;
        res->message =
            ok ? "Stopped MoveIt Servo." : "Failed to stop MoveIt Servo.";
      });
  pause_srv_ = node_.create_service<std_srvs::srv::Trigger>(
      "moveit_servo_client/pause",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        (void)req;
        bool ok = this->pauseServoSync();
        res->success = ok;
        res->message =
            ok ? "Paused MoveIt Servo." : "Failed to pause MoveIt Servo.";
      });
  unpause_srv_ = node_.create_service<std_srvs::srv::Trigger>(
      "moveit_servo_client/unpause",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        (void)req;
        bool ok = this->unpauseServoSync();
        res->success = ok;
        res->message =
            ok ? "Unpaused MoveIt Servo." : "Failed to unpause MoveIt Servo.";
      });
  reset_status_srv_ = node_.create_service<std_srvs::srv::Trigger>(
      "moveit_servo_client/reset_status",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        (void)req;
        bool ok = this->resetServoStatusSync();
        res->success = ok;
        res->message = ok ? "Reset MoveIt Servo status."
                          : "Failed to reset MoveIt Servo status.";
      });
}

// ============================================================================
//  call (无阻塞日志，返回状态)
// ============================================================================

MoveItServoClient::CallResult MoveItServoClient::call_trigger_service(
    rclcpp::Node &node,
    const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr &client,
    const std::string &srv_name, std::chrono::milliseconds wait_timeout,
    std::chrono::milliseconds call_timeout) {
  CallResult r;
  r.status = top_hfsm::TaskStatus::Failure;
  r.message = srv_name;

  if (!client) {
    r.status = top_hfsm::TaskStatus::Failure;
    r.message = "null client";
    return r;
  }

  if (!client->wait_for_service(wait_timeout)) {
    r.status = top_hfsm::TaskStatus::Failure;
    r.message = "service not available";
    return r;
  }

  std::shared_ptr<std_srvs::srv::Trigger::Response> resp;
  try {
    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto fut = client->async_send_request(req);
    if (fut.wait_for(call_timeout) != std::future_status::ready) {
      r.status = top_hfsm::TaskStatus::Timeout;
      r.message = "call timeout";
      return r;
    }
    resp = fut.get();
  } catch (const std::exception &e) {
    r.status = top_hfsm::TaskStatus::Failure;
    r.message = e.what();
    return r;
  } catch (...) {
    r.status = top_hfsm::TaskStatus::Failure;
    r.message = "unknown exception";
    return r;
  }

  if (!resp) {
    r.status = top_hfsm::TaskStatus::Failure;
    r.message = "null response";
    return r;
  }
  if (!resp->success) {
    r.status = top_hfsm::TaskStatus::Failure;
    r.message = resp->message;
    return r;
  }

  r.status = top_hfsm::TaskStatus::Success;
  r.message = resp->message;
  return r;
}

MoveItServoClient::CallResult MoveItServoClient::call_empty_service(
    rclcpp::Node &node,
    const rclcpp::Client<std_srvs::srv::Empty>::SharedPtr &client,
    const std::string &srv_name, std::chrono::milliseconds wait_timeout,
    std::chrono::milliseconds call_timeout) {
  CallResult r;
  r.status = top_hfsm::TaskStatus::Failure;
  r.message = srv_name;

  if (!client) {
    r.status = top_hfsm::TaskStatus::Failure;
    r.message = "null client";
    return r;
  }

  if (!client->wait_for_service(wait_timeout)) {
    r.status = top_hfsm::TaskStatus::Failure;
    r.message = "service not available";
    return r;
  }

  try {
    auto req = std::make_shared<std_srvs::srv::Empty::Request>();
    auto fut = client->async_send_request(req);
    if (fut.wait_for(call_timeout) != std::future_status::ready) {
      r.status = top_hfsm::TaskStatus::Timeout;
      r.message = "call timeout";
      return r;
    }
    (void)fut.get();
  } catch (const std::exception &e) {
    r.status = top_hfsm::TaskStatus::Failure;
    r.message = e.what();
    return r;
  } catch (...) {
    r.status = top_hfsm::TaskStatus::Failure;
    r.message = "unknown exception";
    return r;
  }

  r.status = top_hfsm::TaskStatus::Success;
  r.message = "ok";
  return r;
}

// ============================================================================
//  async request入口
// ============================================================================

void MoveItServoClient::requestStartServo(top_hfsm::AsyncDispatcher &dispatcher) {
  std::lock_guard<std::mutex> lk(servo_mutex_);
  if (servo_pending_)
    return;
  servo_pending_ = true;
  dispatcher.enqueue("servo_start", [this]() { return this->runStartServoJob(); });
}

void MoveItServoClient::requestPauseServo(top_hfsm::AsyncDispatcher &dispatcher) {
  std::lock_guard<std::mutex> lk(servo_mutex_);
  if (servo_pending_)
    return;
  servo_pending_ = true;
  dispatcher.enqueue("servo_pause", [this]() { return this->runPauseServoJob(); });
}

bool MoveItServoClient::servoPending() const {
  std::lock_guard<std::mutex> lk(servo_mutex_);
  return servo_pending_;
}

top_hfsm::TaskStatus MoveItServoClient::lastServoStatus() const {
  std::lock_guard<std::mutex> lk(servo_mutex_);
  return last_servo_status_;
}

std::string MoveItServoClient::lastServoMsg() const {
  std::lock_guard<std::mutex> lk(servo_mutex_);
  return last_servo_msg_;
}

void MoveItServoClient::applyAsyncResult(const top_hfsm::AsyncDispatcher::Result &r) {
  if (r.tag != "servo_start" && r.tag != "servo_pause")
    return;

  {
    std::lock_guard<std::mutex> lk(servo_mutex_);
    servo_pending_ = false;
    last_servo_status_ = r.status;
    last_servo_msg_ = r.message;
  }

  if (r.status == top_hfsm::TaskStatus::Success) {
    RCLCPP_INFO(node_.get_logger(), "[ASYNC][%s][ok] %s", r.tag.c_str(),
                r.message.c_str());
  } else {
    RCLCPP_WARN(node_.get_logger(), "[ASYNC][%s][status=%d] %s", r.tag.c_str(),
                static_cast<int>(r.status), r.message.c_str());
  }
}

// ============================================================================
//  helpers (同步，供后台线程调用)
// ============================================================================

bool MoveItServoClient::startServoSync() {
  CallResult r1 = call_trigger_service(node_, start_servo_client_,
                                       config_.servo_start_service,
                                       std::chrono::milliseconds(4000),
                                       std::chrono::milliseconds(8000));
  if (r1.status != top_hfsm::TaskStatus::Success)
    return false;

  CallResult r2 = call_empty_service(node_, reset_status_client_,
                                     config_.servo_reset_status_service,
                                     std::chrono::milliseconds(4000),
                                     std::chrono::milliseconds(8000));
  if (r2.status != top_hfsm::TaskStatus::Success)
    return false;

  CallResult r3 = call_trigger_service(node_, unpause_servo_client_,
                                       config_.servo_unpause_service,
                                       std::chrono::milliseconds(4000),
                                       std::chrono::milliseconds(8000));
  return r3.status == top_hfsm::TaskStatus::Success;
}

bool MoveItServoClient::pauseServoSync() {
  CallResult r1 = call_trigger_service(node_, pause_servo_client_,
                                       config_.servo_pause_service,
                                       std::chrono::milliseconds(4000),
                                       std::chrono::milliseconds(8000));
  return r1.status == top_hfsm::TaskStatus::Success;
}

bool MoveItServoClient::unpauseServoSync() {
  CallResult r1 = call_trigger_service(node_, unpause_servo_client_,
                                       config_.servo_unpause_service,
                                       std::chrono::milliseconds(4000),
                                       std::chrono::milliseconds(8000));
  return r1.status == top_hfsm::TaskStatus::Success;
}

bool MoveItServoClient::stopServoSync() {
  CallResult r1 = call_trigger_service(node_, pause_servo_client_,
                                       config_.servo_pause_service,
                                       std::chrono::milliseconds(4000),
                                       std::chrono::milliseconds(8000));
  if (r1.status != top_hfsm::TaskStatus::Success)
    return false;
  CallResult r2 = call_trigger_service(node_, stop_servo_client_,
                                       config_.servo_stop_service,
                                       std::chrono::milliseconds(4000),
                                       std::chrono::milliseconds(8000));
  return r2.status == top_hfsm::TaskStatus::Success;
}

bool MoveItServoClient::resetServoStatusSync() {
  CallResult r1 = call_empty_service(node_, reset_status_client_,
                                     config_.servo_reset_status_service,
                                     std::chrono::milliseconds(4000),
                                     std::chrono::milliseconds(8000));
  return r1.status == top_hfsm::TaskStatus::Success;
}

top_hfsm::AsyncDispatcher::Result MoveItServoClient::runStartServoJob() {
  top_hfsm::AsyncDispatcher::Result r;
  r.tag = "servo_start";
  r.status = top_hfsm::TaskStatus::Failure;
  r.message = "start failed";

  CallResult r1 = call_trigger_service(node_, start_servo_client_,
                                       config_.servo_start_service,
                                       std::chrono::milliseconds(4000),
                                       std::chrono::milliseconds(8000));
  if (r1.status != top_hfsm::TaskStatus::Success) {
    r.status = r1.status;
    r.message = r1.message;
    return r;
  }

  CallResult r2 = call_empty_service(node_, reset_status_client_,
                                     config_.servo_reset_status_service,
                                     std::chrono::milliseconds(4000),
                                     std::chrono::milliseconds(8000));
  if (r2.status != top_hfsm::TaskStatus::Success) {
    r.status = r2.status;
    r.message = r2.message;
    return r;
  }

  CallResult r3 = call_trigger_service(node_, unpause_servo_client_,
                                       config_.servo_unpause_service,
                                       std::chrono::milliseconds(4000),
                                       std::chrono::milliseconds(8000));
  r.status = r3.status;
  r.message = r3.message;

  return r;
}

top_hfsm::AsyncDispatcher::Result MoveItServoClient::runPauseServoJob() {
  top_hfsm::AsyncDispatcher::Result r;
  r.tag = "servo_pause";
  r.status = top_hfsm::TaskStatus::Failure;
  r.message = "pause failed";

  CallResult r1 = call_trigger_service(node_, pause_servo_client_,
                                       config_.servo_pause_service,
                                       std::chrono::milliseconds(4000),
                                       std::chrono::milliseconds(8000));
  r.status = r1.status;
  r.message = r1.message;

  return r;
}

// ============================================================================
//  Solution
// ============================================================================

void MoveItServoClient::verboseJointCallback(
    engineer_interfaces::msg::Joints::SharedPtr msg) {
  std::scoped_lock<std::mutex> lock(current_joint_mutex_);
  for (const auto &j : msg->joints) {
    auto it = joint_name_to_index_map_.find(j.name);
    if (it != joint_name_to_index_map_.end()) {
      current_joints_[it->second] = j.position;
    }
  }
}
  // TODO: 实现 键 -> delta_twist 的映射

void MoveItServoClient::customJointCallback(
    engineer_interfaces::msg::Joints::SharedPtr msg) {

  // --------------------------------------------------------------------------
  // dt: 两次进入该回调的时间差
  // --------------------------------------------------------------------------
  static rclcpp::Time last_time = node_.now();
  const rclcpp::Time now = node_.now();
  double dt = (now - last_time).seconds();
  last_time = now;

  // 限幅（避免 dt 异常导致速度炸掉）
  dt = std::clamp(dt, 1e-3, 0.1);

  // --------------------------------------------------------------------------
  // current_joints validity check
  // --------------------------------------------------------------------------
  std::array<double, 6> current_joints_copy;
  {
    std::scoped_lock<std::mutex> lock(current_joint_mutex_);
    current_joints_copy = current_joints_;
  }

  // --------------------------------------------------------------------------
  // JointJog build
  // --------------------------------------------------------------------------
  const double vmax = 0.8;
  const double deadband = 1e-4; // rad，按你系统噪声调：1e-4~1e-3 常见

  control_msgs::msg::JointJog joint_jog_msg;
  joint_jog_msg.header.stamp = now;
  joint_jog_msg.header.frame_id = "joint";

  // duration 与 dt 一致（并给个上限，防止 dt 大时过期/滞后感）
  joint_jog_msg.duration = std::clamp(dt, 0.02, 0.10);
  joint_jog_msg.joint_names.reserve(msg->joints.size());
  joint_jog_msg.velocities.reserve(msg->joints.size());

  for (const auto &j : msg->joints) {
    auto it = joint_name_to_index_map_.find(j.name);
    if (it == joint_name_to_index_map_.end()) {
      RCLCPP_WARN_THROTTLE(
          node_.get_logger(), *node_.get_clock(), 2000,
          "[scope=moveit_servo_client][skip=unknown_joint] name='%s'",
          j.name.c_str());
      continue;
    }

    const size_t idx = it->second;
    if (idx >= current_joints_copy.size()) {
      RCLCPP_WARN_THROTTLE(
          node_.get_logger(), *node_.get_clock(), 2000,
          "[scope=moveit_servo_client][skip=bad_index] name='%s' idx=%zu",
          j.name.c_str(), idx);
      continue;
    }

    const double at_pos = current_joints_copy[idx];
    const double to_pos = j.position;

    double err = to_pos - at_pos;
    if (std::fabs(err) < deadband) {
      err = 0.0;
    }

    double v = err / dt;
    v = std::clamp(v, -vmax, vmax);

    joint_jog_msg.joint_names.push_back(j.name);
    joint_jog_msg.velocities.push_back(v);
  }

  // 如果全被过滤了，就不发（避免空消息干扰）
  if (joint_jog_msg.joint_names.empty()) {
    return;
  }

  joint_jog_pub_->publish(joint_jog_msg);
}

} // top_hfsm::executors

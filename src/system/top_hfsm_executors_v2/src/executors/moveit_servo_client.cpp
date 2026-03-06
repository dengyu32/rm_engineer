#include "top_hfsm_v2/executors/moveit_servo_client.hpp"

#include <future>
#include <memory>

namespace top_hfsm_v2::executors {

namespace {

TaskStatus mapTaskStatus(ServoCommandStatus status) {
  switch (status) {
  case ServoCommandStatus::Success:
    return TaskStatus::Success;
  case ServoCommandStatus::Timeout:
    return TaskStatus::Timeout;
  case ServoCommandStatus::Failure:
  case ServoCommandStatus::Started:
  case ServoCommandStatus::InProgress:
  default:
    return TaskStatus::Failure;
  }
}

} // namespace

MoveItServoClient::MoveItServoClient(rclcpp::Node &node,
                                     const MoveItServoClientConfig &config)
    : node_(node), config_(config), logger_(node.get_logger()) {
  initRosInterfaces();
  initRosServices();
  async_dispatcher_.start();
  async_poll_timer_ = node_.create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&MoveItServoClient::pollAsyncResults, this));
  RCLCPP_INFO(logger_, "[MOVEIT_SERVO_CLIENT_V2] started");
}

MoveItServoClient::~MoveItServoClient() { async_dispatcher_.stop(); }

void MoveItServoClient::initRosInterfaces() {
  start_servo_client_ =
      node_.create_client<std_srvs::srv::Trigger>(config_.servo_start_service);
  stop_servo_client_ =
      node_.create_client<std_srvs::srv::Trigger>(config_.servo_stop_service);
  pause_servo_client_ =
      node_.create_client<std_srvs::srv::Trigger>(config_.servo_pause_service);
  unpause_servo_client_ =
      node_.create_client<std_srvs::srv::Trigger>(config_.servo_unpause_service);
  reset_status_client_ =
      node_.create_client<std_srvs::srv::Empty>(config_.servo_reset_status_service);
}

void MoveItServoClient::initRosServices() {
  start_srv_ = node_.create_service<std_srvs::srv::Trigger>(
      "moveit_servo_client_v2/start",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        (void)req;
        const auto result = this->doStartServo();
        res->success = result.status == ServoCommandStatus::Success;
        res->message = result.message;
      });
  stop_srv_ = node_.create_service<std_srvs::srv::Trigger>(
      "moveit_servo_client_v2/stop",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        (void)req;
        const bool ok = this->stopServoSync();
        res->success = ok;
        res->message = ok ? "Stopped MoveIt Servo." : "Failed to stop MoveIt Servo.";
      });
  pause_srv_ = node_.create_service<std_srvs::srv::Trigger>(
      "moveit_servo_client_v2/pause",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        (void)req;
        const auto result = this->doPauseServo();
        res->success = result.status == ServoCommandStatus::Success;
        res->message = result.message;
      });
  unpause_srv_ = node_.create_service<std_srvs::srv::Trigger>(
      "moveit_servo_client_v2/unpause",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        (void)req;
        const bool ok = this->unpauseServoSync();
        res->success = ok;
        res->message = ok ? "Unpaused MoveIt Servo." : "Failed to unpause MoveIt Servo.";
      });
  reset_status_srv_ = node_.create_service<std_srvs::srv::Trigger>(
      "moveit_servo_client_v2/reset_status",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        (void)req;
        const bool ok = this->resetServoStatusSync();
        res->success = ok;
        res->message = ok ? "Reset MoveIt Servo status." : "Failed to reset MoveIt Servo status.";
      });
}

ServoCommandResult MoveItServoClient::startServo() {
  return executeServoCommand(ServoCommandType::Start);
}

ServoCommandResult MoveItServoClient::pauseServo() {
  return executeServoCommand(ServoCommandType::Pause);
}

ServoCommandResult MoveItServoClient::executeServoCommand(ServoCommandType command) {
  const bool target_active = command == ServoCommandType::Start;

  {
    std::lock_guard<std::mutex> lock(servo_mutex_);

    if (terminal_result_ready_ && terminal_result_command_ == command) {
      const ServoCommandResult result = terminal_result_;
      terminal_result_ready_ = false;
      return result;
    }

    if (servo_pending_) {
      ServoCommandResult result;
      result.status = ServoCommandStatus::InProgress;
      result.message =
          (in_flight_command_ == command) ? "servo command in progress"
                                          : "another servo command in progress";
      return result;
    }

    if (servo_active_ == target_active) {
      ServoCommandResult result;
      result.status = ServoCommandStatus::Success;
      result.message = target_active ? "servo already started"
                                     : "servo already paused";
      return result;
    }

    servo_pending_ = true;
    in_flight_command_ = command;
    terminal_result_ready_ = false;
  }

  async_dispatcher_.enqueue(
      commandTag(command), [this, command]() {
        AsyncDispatcher::Result result;
        result.tag = commandTag(command);
        const auto servo_result =
            (command == ServoCommandType::Start) ? doStartServo() : doPauseServo();
        result.status = mapTaskStatus(servo_result.status);
        result.message = servo_result.message;
        return result;
      });

  ServoCommandResult result;
  result.status = ServoCommandStatus::Started;
  result.message = command == ServoCommandType::Start ? "servo start requested"
                                                      : "servo pause requested";
  return result;
}

ServoCommandResult MoveItServoClient::doStartServo() {
  ServoCommandResult result;
  auto r1 = callTriggerService(node_, start_servo_client_,
                               config_.servo_start_service,
                               std::chrono::milliseconds(4000),
                               std::chrono::milliseconds(8000));
  if (r1.status != ServoCommandStatus::Success) {
    result.status = r1.status;
    result.message = r1.message;
    return result;
  }

  auto r2 = callEmptyService(node_, reset_status_client_, config_.servo_reset_status_service,
                             std::chrono::milliseconds(4000),
                             std::chrono::milliseconds(8000));
  if (r2.status != ServoCommandStatus::Success) {
    result.status = r2.status;
    result.message = r2.message;
    return result;
  }

  auto r3 = callTriggerService(node_, unpause_servo_client_, config_.servo_unpause_service,
                               std::chrono::milliseconds(4000),
                               std::chrono::milliseconds(8000));
  result.status = r3.status;
  result.message = r3.message;
  return result;
}

ServoCommandResult MoveItServoClient::doPauseServo() {
  ServoCommandResult result;
  auto r1 = callTriggerService(node_, pause_servo_client_,
                               config_.servo_pause_service,
                               std::chrono::milliseconds(4000),
                               std::chrono::milliseconds(8000));
  result.status = r1.status;
  result.message = r1.message;
  return result;
}

void MoveItServoClient::pollAsyncResults() {
  std::vector<AsyncDispatcher::Result> results;
  async_dispatcher_.poll(results);
  for (const auto &result : results) {
    const ServoCommandType command = commandFromTag(result.tag);
    const ServoCommandResult servo_result = toServoResult(result);

    std::lock_guard<std::mutex> lock(servo_mutex_);
    servo_pending_ = false;
    terminal_result_ready_ = true;
    terminal_result_command_ = command;
    terminal_result_ = servo_result;
    if (servo_result.status == ServoCommandStatus::Success) {
      servo_active_ = command == ServoCommandType::Start;
    }
  }
}

const char *MoveItServoClient::commandTag(ServoCommandType command) {
  return command == ServoCommandType::Start ? "servo_start_v2"
                                            : "servo_pause_v2";
}

ServoCommandType MoveItServoClient::commandFromTag(const std::string &tag) {
  return tag == "servo_start_v2" ? ServoCommandType::Start
                                 : ServoCommandType::Pause;
}

ServoCommandResult
MoveItServoClient::toServoResult(const AsyncDispatcher::Result &result) {
  ServoCommandResult servo_result;
  switch (result.status) {
  case TaskStatus::Success:
    servo_result.status = ServoCommandStatus::Success;
    break;
  case TaskStatus::Timeout:
    servo_result.status = ServoCommandStatus::Timeout;
    break;
  case TaskStatus::Failure:
  default:
    servo_result.status = ServoCommandStatus::Failure;
    break;
  }
  servo_result.message = result.message;
  return servo_result;
}

MoveItServoClient::CallResult MoveItServoClient::callTriggerService(
    rclcpp::Node &node,
    const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr &client,
    const std::string &service_name, std::chrono::milliseconds wait_timeout,
    std::chrono::milliseconds call_timeout) {
  CallResult result;
  result.status = ServoCommandStatus::Failure;
  result.message = service_name;

  if (!client) {
    result.message = "null client";
    return result;
  }
  if (!client->wait_for_service(wait_timeout)) {
    result.message = "service not available";
    return result;
  }

  std::shared_ptr<std_srvs::srv::Trigger::Response> response;
  try {
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = client->async_send_request(request);
    if (future.wait_for(call_timeout) != std::future_status::ready) {
      result.status = ServoCommandStatus::Timeout;
      result.message = "call timeout";
      return result;
    }
    response = future.get();
  } catch (const std::exception &e) {
    result.message = e.what();
    return result;
  } catch (...) {
    result.message = "unknown exception";
    return result;
  }

  if (!response) {
    result.message = "null response";
    return result;
  }
  if (!response->success) {
    result.message = response->message;
    return result;
  }

  result.status = ServoCommandStatus::Success;
  result.message = response->message;
  return result;
}

MoveItServoClient::CallResult MoveItServoClient::callEmptyService(
    rclcpp::Node &node,
    const rclcpp::Client<std_srvs::srv::Empty>::SharedPtr &client,
    const std::string &service_name, std::chrono::milliseconds wait_timeout,
    std::chrono::milliseconds call_timeout) {
  CallResult result;
  result.status = ServoCommandStatus::Failure;
  result.message = service_name;

  if (!client) {
    result.message = "null client";
    return result;
  }
  if (!client->wait_for_service(wait_timeout)) {
    result.message = "service not available";
    return result;
  }

  try {
    auto request = std::make_shared<std_srvs::srv::Empty::Request>();
    auto future = client->async_send_request(request);
    if (future.wait_for(call_timeout) != std::future_status::ready) {
      result.status = ServoCommandStatus::Timeout;
      result.message = "call timeout";
      return result;
    }
    (void)future.get();
  } catch (const std::exception &e) {
    result.message = e.what();
    return result;
  } catch (...) {
    result.message = "unknown exception";
    return result;
  }

  result.status = ServoCommandStatus::Success;
  result.message = "ok";
  return result;
}

bool MoveItServoClient::startServoSync() {
  return doStartServo().status == ServoCommandStatus::Success;
}

bool MoveItServoClient::pauseServoSync() {
  return doPauseServo().status == ServoCommandStatus::Success;
}

bool MoveItServoClient::unpauseServoSync() {
  const auto r = callTriggerService(node_, unpause_servo_client_,
                                    config_.servo_unpause_service,
                                    std::chrono::milliseconds(4000),
                                    std::chrono::milliseconds(8000));
  return r.status == ServoCommandStatus::Success;
}

bool MoveItServoClient::stopServoSync() {
  const auto r1 = callTriggerService(node_, pause_servo_client_,
                                     config_.servo_pause_service,
                                     std::chrono::milliseconds(4000),
                                     std::chrono::milliseconds(8000));
  if (r1.status != ServoCommandStatus::Success) {
    return false;
  }
  const auto r2 = callTriggerService(node_, stop_servo_client_,
                                     config_.servo_stop_service,
                                     std::chrono::milliseconds(4000),
                                     std::chrono::milliseconds(8000));
  return r2.status == ServoCommandStatus::Success;
}

bool MoveItServoClient::resetServoStatusSync() {
  const auto r = callEmptyService(node_, reset_status_client_,
                                  config_.servo_reset_status_service,
                                  std::chrono::milliseconds(4000),
                                  std::chrono::milliseconds(8000));
  return r.status == ServoCommandStatus::Success;
}

} // namespace top_hfsm_v2::executors

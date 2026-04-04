#include "vision_detect_client/vision_detect_client.hpp"

#include <chrono>
#include <exception>

#include "task_orchestrator/protocol.hpp"

namespace engineer_auto::vision_detect_client {

using core::Command;
using core::ExecuteResult;
using core::makeFailed;
using core::makeRunning;
using core::makeSucceeded;

// -----------------------------------------------------------------------------
// CTOR
// -----------------------------------------------------------------------------
VisionDetectClient::VisionDetectClient(rclcpp::Node &node)
    : node_(node), logger_(node.get_logger()), config_(VisionDetectClientConfig::load(node)) {
  RCLCPP_INFO(logger_, "\n%s", config_.summary().c_str());
  detect_client_ =
      node_.create_client<engineer_interfaces::srv::DetectEnergyUnit>(config_.detect_service_name);
  RCLCPP_INFO(logger_, "[VISION_CLIENT] service=%s wait_ms=%d",
              config_.detect_service_name.c_str(), config_.service_wait_ms);
}

ExecuteResult VisionDetectClient::execute(const Command &cmd) {
  std::optional<PendingRequest> ready_request;
  {
    std::scoped_lock lock(mutex_);
    if (pending_request_) {
      if (pending_request_->future.wait_for(std::chrono::seconds(0)) !=
          std::future_status::ready) {
        return makeRunning();
      }
      ready_request = std::move(pending_request_);
      pending_request_.reset();
    }
  }

  if (ready_request) {
    return consumePendingRequest(std::move(*ready_request));
  }

  bool enable = true;
  std::string error;
  if (!buildRequest(cmd, enable, error)) {
    return makeFailed(error, false);
  }

  if (sendRequest(enable)) {
    return makeRunning();
  }

  const std::string last_error = getLastError();
  return makeFailed(last_error.empty() ? "vision request send failed" : last_error, true);
}

bool VisionDetectClient::buildRequest(const Command &cmd, bool &enable,
                                      std::string &error) const {
  enable = true;
  if (const auto *request_enable = core::paramAs<bool>(cmd, "enable")) {
    enable = *request_enable;
  }
  error.clear();
  return true;
}

bool VisionDetectClient::sendRequest(bool enable) {
  setLastError("");

  if (!detect_client_) {
    setLastError("vision service client not created");
    return false;
  }

  if (!detect_client_->wait_for_service(std::chrono::milliseconds(config_.service_wait_ms))) {
    setLastError("vision service not available: " + config_.detect_service_name);
    return false;
  }

  auto request = std::make_shared<engineer_interfaces::srv::DetectEnergyUnit::Request>();
  request->enable = enable;

  try {
    PendingRequest pending = detect_client_->async_send_request(request);
    std::scoped_lock lock(mutex_);
    pending_request_.emplace(std::move(pending));
  } catch (const std::exception &ex) {
    setLastError(std::string("failed to send vision request: ") + ex.what());
    return false;
  }

  return true;
}

ExecuteResult VisionDetectClient::consumePendingRequest(PendingRequest request) {
  try {
    const auto response = request.future.get();
    if (!response) {
      setLastError("vision service returned null response");
      return makeFailed("vision service returned null response", true);
    }

    if (!response->success) {
      const std::string message =
          response->message.empty() ? "vision detection failed" : response->message;
      setLastError(message);
      return makeFailed(message, true);
    }

    VisionSpec detection{};
    detection.pose = response->target.target_pose;
    detection.vector = response->target.target_vector;

    auto result = makeSucceeded();
    result.outputs[task_orchestrator::protocol::kVisionPose] = std::vector<double>{
        detection.pose.x,
        detection.pose.y,
        detection.pose.z,
        detection.pose.qx,
        detection.pose.qy,
        detection.pose.qz,
        detection.pose.qw,
    };
    result.outputs[task_orchestrator::protocol::kVisionVector] = std::vector<double>{
        detection.vector.x,
        detection.vector.y,
        detection.vector.z,
    };
    setLastError("");
    return result;
  } catch (const std::exception &ex) {
    const std::string message = std::string("vision service exception: ") + ex.what();
    setLastError(message);
    return makeFailed(message, true);
  }
}

void VisionDetectClient::cancel() {
  {
    std::scoped_lock lock(mutex_);
    if (detect_client_ && pending_request_) {
      detect_client_->remove_pending_request(*pending_request_);
      pending_request_.reset();
    }
  }
  setLastError("vision request canceled");
}

void VisionDetectClient::setLastError(const std::string &error) {
  std::scoped_lock lock(mutex_);
  last_error_ = error;
}

std::string VisionDetectClient::getLastError() const {
  std::scoped_lock lock(mutex_);
  return last_error_;
}

} // namespace engineer_auto::vision_detect_client

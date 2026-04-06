#include "vision_detect_client/vision_detect_client.hpp"

#include <chrono>
#include <exception>

#include "auto_library/context_keys.hpp"

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
  // 创建客户端
  detect_client_ =
      node_.create_client<engineer_interfaces::srv::DetectEnergyUnit>(config_.detect_service_name);
  RCLCPP_INFO(logger_, "[VISION_CLIENT] service=%s wait_ms=%d",
              config_.detect_service_name.c_str(), config_.service_wait_ms);
}

// -----------------------------------------------------------------------------
// Execute
// -----------------------------------------------------------------------------

ExecuteResult VisionDetectClient::execute(const Command &cmd) {
  std::optional<PendingRequest> ready_request;
  {
    std::scoped_lock lock(mutex_);
    // 是否存在正在进行的请求
    if (pending_request_) {
      // 检查任务是否完成 ( 等待0秒,瞬间查询状态 )
      if (pending_request_->future.wait_for(std::chrono::seconds(0)) !=
          std::future_status::ready) {
        // 任务仍在进行中，继续等待
        return makeRunning();
      }
      // 任务完成,将结果取出并准备处理
      ready_request = std::move(pending_request_);
      pending_request_.reset();
    }
  }

  // 如果有处理完成的结果
  if (ready_request) {
    return consumePendingRequest(std::move(*ready_request));
  }

  // 如果没有正在进行的请求，则构建并发送新的请求
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

// -----------------------------------------------------------------------------
// BUILD-IN FUNC
// -----------------------------------------------------------------------------

bool VisionDetectClient::buildRequest(const Command &cmd, bool &enable,
                                      std::string &error) const {
  enable = true;
  // 从 cmd 中查找键为 "enable" 的参数, 返回该值指针,不存在或者不是 bool 类型则返回 nullptr
  // *request_eanble 是解引用指针获取值
  if (const auto *request_enable = core::paramAs<bool>(cmd, "enable")) {
    enable = *request_enable;
  }
  error.clear();
  return true;
}

bool VisionDetectClient::sendRequest(bool enable) {
  setLastError("");
  // 检查客户端是否创建成功
  if (!detect_client_) {
    setLastError("vision service client not created");
    return false;
  }
  // 等待服务可用, 超时则设置错误信息并返回 false
  if (!detect_client_->wait_for_service(std::chrono::milliseconds(config_.service_wait_ms))) {
    setLastError("vision service not available: " + config_.detect_service_name);
    return false;
  }

  // 创建请求
  auto request = std::make_shared<engineer_interfaces::srv::DetectEnergyUnit::Request>();
  request->enable = enable;

  try {
    // 异步发送请求,返回一个 future 对象和请求 ID 的结构体
    PendingRequest pending = detect_client_->async_send_request(request);
    std::scoped_lock lock(mutex_);
    // 将 pending 请求对象存储在成员变量中,以便后续检查和处理结果
    pending_request_.emplace(std::move(pending));
  } catch (const std::exception &ex) {
    setLastError(std::string("failed to send vision request: ") + ex.what());
    return false;
  }

  return true;
}

// 有处理完的结果
ExecuteResult VisionDetectClient::consumePendingRequest(PendingRequest request) {
  try {
    // 获取结果
    const auto response = request.future.get();
    // 检查响应是否有效
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
    // 从响应中提取结果
    VisionSpec detection{};
    detection.pose = response->target.target_pose;
    detection.vector = response->target.target_vector;

    // 构建成功的 ExecuteResult
    auto result = makeSucceeded();
    result.outputs[core::keys::kVisionPose] = std::vector<double>{
        detection.pose.x,
        detection.pose.y,
        detection.pose.z,
        detection.pose.qx,
        detection.pose.qy,
        detection.pose.qz,
        detection.pose.qw,
    };
    result.outputs[core::keys::kVisionVector] = std::vector<double>{
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
      // 丢弃未完成的请求, 这里没有真正的取消机制, 只是丢弃结果不处理
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

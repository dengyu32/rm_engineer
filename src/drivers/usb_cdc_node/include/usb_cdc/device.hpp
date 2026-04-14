// ============================================================================
//  device.hpp
// ----------------------------------------------------------------------------
//  - 提供 USB CDC 设备的 PIMPL 封装，负责打开、发送与事件轮询
//  - 依赖 libusb + rclcpp logging，对外暴露线程安全的同步接口
//  - 内置流式拼帧逻辑并上报完整数据帧
//  - RAII 工具 FinalAction 协助 transfer / 热插拔清理
// ============================================================================

#pragma once

// C++
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <unordered_map>
#include <vector>

// Libusb
#include <libusb-1.0/libusb.h>

// ROS2
#include <rclcpp/logging.hpp>

namespace usb_cdc {

// ============================================================================
//  Device
// ----------------------------------------------------------------------------
//  - USB CDC 设备生命周期管理（open / send / event loop）
//  - 使用 PIMPL 隐藏 libusb 细节，面向上层提供同步 API
//  - 内部完成异步数据的流式拼帧与完整帧回调
// ============================================================================
class Device {
public:
  using PacketCallback = std::function<void(const std::byte *, size_t)>;

  // -----------------------------------------------------------------------
  //  Lifecycle
  // -----------------------------------------------------------------------
  Device();
  ~Device();

  // -----------------------------------------------------------------------
  //  Operations
  // -----------------------------------------------------------------------
  bool open(uint16_t vid, uint16_t pid = 0);
  bool send_data(uint8_t *data, std::size_t size);
  void handle_events();
  bool is_open() const;
  void request_reconnect();
  void set_packet_callback(uint8_t id, PacketCallback callback);

private:
  struct TransmitBuffer;
  class Impl;

  std::unique_ptr<Impl> impl_;
};

// ============================================================================
//  FinalAction
// ----------------------------------------------------------------------------
//  - 作用域退出自动执行清理动作的 RAII 包装
//  - 适合 transfer 释放、句柄注销等收尾逻辑
// ============================================================================
template <class F> class FinalAction {
public:
  explicit FinalAction(F f) : f_(f), enabled_(true) {}

  ~FinalAction() {
    if (enabled_)
      f_();
  }

  void disable() { enabled_ = false; }

  FinalAction(const FinalAction &) = delete;
  FinalAction &operator=(const FinalAction &) = delete;
  FinalAction(FinalAction &&) = delete;
  FinalAction &operator=(FinalAction &&) = delete;

private:
  F f_;          // 清理动作
  bool enabled_; // 是否启用
};

// ============================================================================
//  Device::Impl
// ----------------------------------------------------------------------------
//  - libusb 全流程实现：初始化、枚举、热插拔与事件循环
//  - 管理异步 transfer 提交与回调，并直接完成流式拼帧
//  - 处理断开后的自动重连，维持通信可用性
//  - 提供 sync_send/process_once 供上层周期调用
// ============================================================================
class Device::Impl {
public:
  // -----------------------------------------------------------------------
  //  USB constants
  // -----------------------------------------------------------------------
  static constexpr uint16_t VID = 0x0483;     // STM32 默认 Vendor ID
  static constexpr uint8_t EP_OUT = 0x01;     // 主机 → 设备 OUT 端点
  static constexpr uint8_t EP_IN = 0x81;      // 设备 → 主机 IN 端点
  static constexpr uint8_t INTERFACE_NUM = 1; // USB 接口号

  struct error : std::runtime_error {
    explicit error(int code)
        : std::runtime_error(libusb_error_name(code)), code(code) {}
    int code; // libusb 错误码
  };

  // -----------------------------------------------------------------------
  //  Lifecycle
  // -----------------------------------------------------------------------
  Impl() { handling_events_ = true; }

  ~Impl() {
    handling_events_ = false;
    cleanup();
    if (ctx_)
      libusb_exit(ctx_);
  }

  // -----------------------------------------------------------------------
  //  Operations
  // -----------------------------------------------------------------------
  bool open(uint16_t vid, uint16_t pid = 0);
  void process_once();
  bool sync_send(uint8_t *data, std::size_t size, unsigned tout_ms = 5);
  bool is_open() const;
  void request_reconnect();
  void set_packet_callback(uint8_t id, PacketCallback callback);

  // -----------------------------------------------------------------------
  //  Hot-plug callbacks
  // -----------------------------------------------------------------------
  void on_hotplug(libusb_hotplug_event ev);

  // -----------------------------------------------------------------------
  //  Internal helpers
  // -----------------------------------------------------------------------
  uint16_t find_device(uint16_t vid);
  void alloc_transfer();
  void submit_transfer();
  void cleanup();
  void cleanup_unlocked();
  bool try_reopen();
  void parse(const std::byte *data, size_t size);

  // -----------------------------------------------------------------------
  //  State
  // -----------------------------------------------------------------------
  std::unordered_map<uint8_t, PacketCallback> packet_callbacks_;
  std::vector<std::byte> stream_buffer_;

  libusb_context *ctx_ = nullptr;          // libusb 上下文
  libusb_device_handle *handle_ = nullptr; // USB 设备句柄
  libusb_transfer *rx_transfer_ = nullptr; // 异步接收 Transfer
  libusb_hotplug_callback_handle hp_handle_{};
  bool hotplug_registered_{false};

  std::byte rx_buf_[64]{}; // USB 单次接收 chunk 缓冲区

  std::atomic_bool handling_events_{false};
  std::atomic_bool disconnected_{false};
  std::atomic_bool rx_transfer_done_{false};
  uint16_t reconnect_vid_{VID};
  uint16_t reconnect_pid_{0};
  mutable std::mutex io_mutex_;
};

} // namespace usb_cdc

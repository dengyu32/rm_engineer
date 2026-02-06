/**
 * @file usb_cdc_driver.cpp
 * @brief USB CDC 设备驱动实现文件（libusb 异步收发 + 热插拔处理）
 *
 * 本文件实现 usb_cdc::Device::Impl 的底层 USB CDC 通信逻辑，包含：
 * - 设备打开/关闭、接口声明
 * - libusb 异步 bulk 传输分配与回调
 * - 热插拔（Hot-plug）检测与自动重连
 * - 同步发送接口 sync_send()
 * - 事件循环处理 process_once()
 *
 * 设计说明：
 * - 使用 libusb 的异步传输机制进行数据接收（bulk IN）。
 * - 使用同步 bulk 传输进行发送（bulk OUT）。
 * - 热插拔使用 libusb_hotplug 机制自动检测断开与重新连接。
 * - 通过 DeviceParser 回调将接收到的完整数据包交给上层解析。
 */

// USB CDC
#include "usb_cdc/usb_cdc_driver.hpp"
#include "usb_cdc/packet.hpp"

// ROS2
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>

namespace usb_cdc {

// ============================================================================
//  Operations
// ============================================================================
/**
 * @brief 打开 USB CDC 设备，初始化 libusb，声明接口并启动异步接收。
 *
 * @param vid USB Vendor ID（厂商 ID）
 * @param pid USB Product ID（产品 ID），若为 0 则自动搜索匹配 VID 的 PID
 * @return true 打开成功
 * @throw error 设备找不到 / 接口占用 / 其他 libusb 错误
 *
 * 执行流程：
 * 1. 初始化 libusb
 * 2. 若 pid=0，调用 find_device() 查找该 VID 对应的唯一 PID
 * 3. 打开设备句柄 libusb_open_device_with_vid_pid
 * 4. 若内核已绑定驱动，尝试脱离（CDC 在 Linux 下常被 cdc_acm 占用）
 * 5. 声明 USB 接口 libusb_claim_interface
 * 6. 分配 bulk IN 异步接收传输 alloc_transfer()
 * 7. 注册 Hot-plug 回调（若系统支持）
 * 8. 提交异步接收 submit_transfer()，开始持续收包
 */
bool Device::Impl::open(uint16_t vid, uint16_t pid) {
  if (!ctx_) { // 防止重复初始化，只有在第一次调用open()时执行
    if (libusb_init(&ctx_) != 0)
      throw error(LIBUSB_ERROR_OTHER);
  }

  if (pid == 0)
    pid = find_device(vid);

  // 打开设备句柄
  handle_ = libusb_open_device_with_vid_pid(ctx_, vid, pid);
  if (!handle_)
    throw error(LIBUSB_ERROR_NO_DEVICE);

  // 若内核驱动占用 CDC 接口，则先脱离
  if (libusb_kernel_driver_active(handle_, INTERFACE_NUM)) {
    int detach_result = libusb_detach_kernel_driver(handle_, INTERFACE_NUM);
    if (detach_result != LIBUSB_SUCCESS)
      throw error(detach_result);
  }

  // 声明接口
  if (libusb_claim_interface(handle_, INTERFACE_NUM))
    throw error(LIBUSB_ERROR_BUSY);

  // 分配异步接收传输
  alloc_transfer();

  // 注册热插拔回调：检测 USB 插入 / 拔出
  if (libusb_has_capability(LIBUSB_CAP_HAS_HOTPLUG)) {
    int rc = libusb_hotplug_register_callback(
        ctx_,
        static_cast<libusb_hotplug_event>(LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT |
                                          LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED),
        LIBUSB_HOTPLUG_NO_FLAGS, vid, pid, LIBUSB_HOTPLUG_MATCH_ANY,
        [](libusb_context * /*ctx*/, libusb_device * /*dev*/,
           libusb_hotplug_event event, void *user) -> int {  
          static_cast<Impl *>(user)->on_hotplug(event);
          return 0;
        },
        this, &hp_handle_);
    if (rc != LIBUSB_SUCCESS)
      RCLCPP_INFO(rclcpp::get_logger("usb_cdc"),
                  " [FAILED] Hot-plug callback reg failed: {%s} ",
                  libusb_error_name(rc));
    else
      hotplug_registered_ = true;
  }

  // 启动异步接收
  submit_transfer();
  handling_events_ = true;
  disconnected_ = false;
  hotplug_arrived_ = false;
  rx_transfer_done_ = false;
  return true;
}

// ============================================================================
//  Internal helpers
// ============================================================================
/**
 * @brief 根据 VID 自动查找唯一的 PID（适用于只有 VID 的情况下）
 *
 * @param vid USB Vendor ID
 * @return uint16_t 对应的唯一 PID
 * @throw error 找不到设备 / PID 数量大于 1（冲突）
 */
uint16_t Device::Impl::find_device(uint16_t vid) {
  libusb_device **list;
  ssize_t cnt = libusb_get_device_list(ctx_, &list);
  if (cnt < 0)
    throw error(static_cast<int>(cnt));
  FinalAction free{[&]() { libusb_free_device_list(list, 1); }};

  std::vector<uint16_t> pids;
  for (ssize_t i = 0; i < cnt; ++i) {
    libusb_device_descriptor d{};
    if (libusb_get_device_descriptor(list[i], &d) == 0 && d.idVendor == vid)
      pids.push_back(d.idProduct);
  }
  if (pids.empty())
    throw error(LIBUSB_ERROR_NOT_FOUND);
  if (pids.size() > 1)
    throw error(LIBUSB_ERROR_OVERFLOW);
  return pids[0];
}

// ============================================================================
// alloc_transfer() — 分配异步 bulk IN 传输及其回调函数
// ============================================================================
/**
 * @brief 分配异步 bulk IN 传输结构，并设置回调函数。
 *
 * libusb_fill_bulk_transfer() 会绑定：
 * - 接口句柄
 * - 端点号 EP_IN
 * - 接收缓冲区 rx_buf_
 * - 回调函数，用来处理每个到来的 USB 数据包
 *
 * 回调逻辑：
 * 1. 传输状态检查（若失败则认为设备断开）
 * 2. 若收到有效数据，则调用上层 device_parser_.parse()
 * 3. 自动重新提交下一次传输 libusb_submit_transfer
 */
void Device::Impl::alloc_transfer() {
  rx_transfer_ = libusb_alloc_transfer(0);
  if (!rx_transfer_)
    throw error(LIBUSB_ERROR_NO_MEM);

  libusb_fill_bulk_transfer(
      rx_transfer_, handle_, EP_IN, reinterpret_cast<uint8_t *>(rx_buf_),
      sizeof(rx_buf_),
      [](libusb_transfer *tr) {
        auto self = static_cast<Impl *>(tr->user_data);
        if (tr->status == LIBUSB_TRANSFER_CANCELLED) {
          self->rx_transfer_done_ = true;
          return;
        }
        if (!self->handling_events_) {
          self->rx_transfer_done_ = true;
          return;
        }

        // USB 中断 / 拔出
        if (tr->status != LIBUSB_TRANSFER_COMPLETED) {
          self->disconnected_ = true;
          self->rx_transfer_done_ = true;
          return;
        }

        // 有效数据
        int len = tr->actual_length;
        if (len > 0) {
          if (self->first_rx_) {
            // 第一次接收通常用于丢弃握手包
            self->first_rx_ = false;
          } else {
            self->device_parser_.parse(
                reinterpret_cast<std::byte *>(tr->buffer),
                static_cast<size_t>(len));
          }
        }

        // 继续提交下一次接收
        int rc = libusb_submit_transfer(tr);
        if (rc != 0) {
          self->disconnected_ = true;
          self->rx_transfer_done_ = true;
        }
      },
      this, 0);
}

// ============================================================================
// submit_transfer() — 提交异步传输
// ============================================================================
/**
 * @brief 将分配好的传输提交给 libusb，开始异步接收。
 * @throw error 提交失败（I/O 错误）
 */
void Device::Impl::submit_transfer() {
  if (libusb_submit_transfer(rx_transfer_) != 0)
    throw error(LIBUSB_ERROR_IO);
}

// ============================================================================
// cleanup() — 释放 USB 资源
// ============================================================================
/**
 * @brief 清理并释放所有 USB 资源，包含：
 * - 取消异步传输
 * - 释放传输结构
 * - 释放接口
 * - 关闭设备句柄
 */
void Device::Impl::cleanup() {
  if (hotplug_registered_ && ctx_) {
    libusb_hotplug_deregister_callback(ctx_, hp_handle_);
    hotplug_registered_ = false;
  }
  if (rx_transfer_) {
    libusb_cancel_transfer(rx_transfer_);
    rx_transfer_done_ = false;
    if (ctx_) {
      timeval tv{0, 1000}; // 1ms
      for (int i = 0; i < 200 && !rx_transfer_done_; ++i) {
        libusb_handle_events_timeout_completed(ctx_, &tv, nullptr);
      }
    }
    libusb_free_transfer(rx_transfer_);
    rx_transfer_ = nullptr;
  }
  if (handle_) {
    libusb_release_interface(handle_, INTERFACE_NUM);
    libusb_close(handle_);
    handle_ = nullptr;
  }
}

// ============================================================================
//  Operations (event loop)
// ============================================================================
/**
 * @brief 处理一次 libusb 事件循环，并检测是否需要重连。
 *
 * 注意：必须在用户的主循环中周期性调用：
 * @code
 * device.handle_events();
 * @endcode
 *
 * 若设备断开（disconnected_ = true），则会自动进入 try_reopen().
 */
void Device::Impl::process_once() {
  if (!ctx_) {
    return;
  }
  timeval tv{0, 1000}; // 1ms to avoid busy loop
  libusb_handle_events_timeout_completed(ctx_, &tv, nullptr);

  if (disconnected_ || hotplug_arrived_) {
    hotplug_arrived_ = false;
    cleanup();
    try_reopen();
  }
}
// timeval tv{0, 0};

// ============================================================================
//  Hot-plug callbacks
// ============================================================================
/**
 * @brief libusb hot-plug 回调函数。用于检测设备插入/拔出。
 *
 * @param ev 事件类型（到达/离开）
 */
void Device::Impl::on_hotplug(libusb_hotplug_event ev) {
  if (ev == LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT) {
    disconnected_ = true;
  } else if (ev == LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED) {
    hotplug_arrived_ = true;
  }
}

// ============================================================================
//  Internal helpers (reconnect)
// ============================================================================
/**
 * @brief 在设备掉线后尝试自动重连。
 *
 * @return true 重连成功
 * @return false 若 handling_events_ 已停止则不再重连
 *
 * 重连策略：
 * - 每 500 ms 尝试一次 open(VID)
 * - 成功后设置 disconnected_ = false
 */
bool Device::Impl::try_reopen() {
  while (handling_events_) {
    try {
      if (open(VID)) {
        disconnected_ = false;
        RCLCPP_INFO(rclcpp::get_logger("usb_cdc"),
                    " [SUCCESS] USB re-connected ");
        return true;
      }
    } catch (...) {
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  return false;
}

void Device::Impl::request_reconnect() {
  disconnected_ = true;
}

// ============================================================================
//  Operations (send)
// ============================================================================
/**
 * @brief 使用同步 bulk 传输发送数据。
 *
 * @param data 发送数据指针
 * @param size 数据长度
 * @param tout_ms 超时时间（毫秒）
 * @return true 成功发送所有数据
 *
 * 说明：
 * - 使用同步阻塞方式，适合上层在单独线程中调用
 * - 对于高频发送，可扩展为异步发送队列
 */
bool Device::Impl::sync_send(uint8_t *data, std::size_t size,
                             unsigned tout_ms) {
  if (!handle_)
    return false;
  int actual = 0;
  int rc = libusb_bulk_transfer(handle_, EP_OUT, data, static_cast<int>(size),
                                &actual, tout_ms);
  return rc == 0 && actual == static_cast<int>(size);
}

// ============================================================================
//  Facade
// ============================================================================

// ============================================================================
//  ctor
// ============================================================================
Device::Device(DeviceParser &p) : impl_(std::make_unique<Impl>(p)) {}
Device::~Device() = default;

bool Device::open(uint16_t vid, uint16_t pid) { return impl_->open(vid, pid); }
bool Device::send_data(uint8_t *d, std::size_t s) {
  return impl_->sync_send(d, s);
}
void Device::handle_events() { impl_->process_once(); }
bool Device::is_open() const { return impl_->is_open(); }
void Device::request_reconnect() { impl_->request_reconnect(); }

// ============================================================================
//  Parser
// ============================================================================

void DeviceParser::parse(const std::byte *data, size_t size) {
  static auto clock = rclcpp::Clock::make_shared();
  if (size < sizeof(HeaderFrame) + sizeof(uint8_t)) { // 检验最小长度 HeaderFrame + 1(eof)
    RCLCPP_WARN_THROTTLE(rclcpp::get_logger("usb_cdc"),
                         *clock, 2000,
                         " [WARN] Frame size too small, expected 0x%X, got 0x%X ",
                         static_cast<unsigned int>(sizeof(HeaderFrame) + sizeof(uint8_t)),
                         static_cast<unsigned int>(size));
    return;
  }

  HeaderFrame header_frame;
  std::memcpy(&header_frame, data, sizeof(HeaderFrame));
  if (header_frame.sof != HeaderFrame::SoF()) { // 校验包头 SOF
    RCLCPP_WARN_THROTTLE(rclcpp::get_logger("usb_cdc"),
                         *clock, 2000,
                         " [WARN] Frame header invalid, expected 0x%X, got 0x%X ",
                         static_cast<unsigned int>(HeaderFrame::SoF()),
                         static_cast<unsigned int>(header_frame.sof));
    return;
  }

  const size_t expected_size = sizeof(HeaderFrame) + header_frame.len + sizeof(uint8_t);
  if (size != expected_size) { // 检验总长  HF + len + 1(eof)
    RCLCPP_WARN_THROTTLE(rclcpp::get_logger("usb_cdc"),
                         *clock, 2000,
                         " [WARN] Frame length mismatch, expected 0x%X, got 0x%X ",
                         static_cast<unsigned int>(expected_size),
                         static_cast<unsigned int>(size));
    return;
  }

  const uint16_t computed_crc =
      calc_crc_len_id_payload(header_frame.len, header_frame.id,
                              data + sizeof(HeaderFrame));
  if (computed_crc != header_frame.crc) { // 校验 CRC
    RCLCPP_WARN_THROTTLE(rclcpp::get_logger("usb_cdc"),
                         *clock, 2000,
                         " [WARN] Frame crc invalid, expected 0x%X, got 0x%X ",
                         static_cast<unsigned int>(computed_crc),
                         static_cast<unsigned int>(header_frame.crc));
    return;
  }

  const uint8_t eof = static_cast<uint8_t>(data[expected_size - 1]);
  if (eof != HeaderFrame::EoF()) { // 检验包尾 eof
    RCLCPP_WARN_THROTTLE(rclcpp::get_logger("usb_cdc"),
                         *clock, 2000,
                         " [WARN] Frame eof invalid, expected 0x%X, got 0x%X ",
                         static_cast<unsigned int>(HeaderFrame::EoF()),
                         static_cast<unsigned int>(eof));
    return;
  }

  auto it = parser_table_.find(header_frame.id); // 通过 id 寻找相应析构函数
  if (it != parser_table_.end()) {
    it->second(data, size);
  } else {
    RCLCPP_WARN_THROTTLE(rclcpp::get_logger("usb_cdc"),
                         *clock, 2000,
                         " [WARN] Unknown header 0x%X ",
                         header_frame.id);
    return;
  }
}

} // namespace usb_cdc

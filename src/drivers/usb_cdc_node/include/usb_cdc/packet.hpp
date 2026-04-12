// ============================================================================
//  packet.hpp
// ----------------------------------------------------------------------------
//  - 定义 USB CDC 收发数据帧的公共头与工程报文结构
//  - 固定 SoF/EoF 边界，方便 Device 做帧同步
//  - 提供收发数据打印工具，便于串口调试
//  - 使用 #pragma pack(1) 保证与下位机一致的字节布局
// ============================================================================
#pragma once

// C++
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <iostream>

namespace usb_cdc {

// ============================================================================
//  Quantization
// ----------------------------------------------------------------------------
//  USB 协议层用 uint16_t 压缩关节浮点量，ROS 节点边界再恢复为 float。
// ============================================================================
inline uint16_t float_to_uint(float x_float, float x_min = -3.1415926F,
                              float x_max = 3.1415926F, int bits = 16) {
  if (bits <= 0 || bits > 16 || !(x_max > x_min)) {
    return 0U;
  }

  const float value = std::clamp(x_float, x_min, x_max);
  const float span = x_max - x_min;
  const uint16_t max_int = static_cast<uint16_t>((1U << bits) - 1U);
  const auto quantized =
      std::lround((value - x_min) * static_cast<float>(max_int) / span);
  return static_cast<uint16_t>(
      std::clamp<long>(quantized, 0L, static_cast<long>(max_int)));
}

inline float uint_to_float(uint16_t x_int, float x_min = -3.1415926F,
                           float x_max = 3.1415926F, int bits = 16) {
  if (bits <= 0 || bits > 16 || !(x_max > x_min)) {
    return x_min;
  }

  const float span = x_max - x_min;
  const uint16_t max_int = static_cast<uint16_t>((1U << bits) - 1U);
  const uint16_t value = std::min(x_int, max_int);
  return (static_cast<float>(value) * span / static_cast<float>(max_int)) + x_min;
}

#pragma pack(1) // 数据包按字节对齐

// ============================================================================
//  HeaderFrame
// ----------------------------------------------------------------------------
//  - 所有数据帧公共头，定义 SoF/EoF 与包 ID
//  - len 表示有效负载长度，便于校验和解析
//  - 提供 SoF/EoF 静态函数方便校验常量
// ============================================================================
struct HeaderFrame {
  static constexpr uint8_t SoF() { return 0x5A; } // 数据包开始标识
  static constexpr uint8_t EoF() { return 0xA5; } // 数据包结束标识

  uint8_t sof; // 0x5A
  uint8_t len; // 数据区长度（不含 header & eof）
  uint8_t id;  // 数据包 ID（区分不同功能包）
};

// ============================================================================
//  EngineerRxPacket
// ============================================================================
struct EngineerRxPacket {
  HeaderFrame header;
  struct {
    uint16_t actualJointPosition[7]; // 当前关节位置
    uint16_t actualJointVelocity[6]; // 当前关节速度
    uint16_t customJointPosition[6]; // 自定义关节角度
    uint8_t realSlotStatus[2];
    uint8_t IntentStatus; ///< 当前意图
  } data;
  uint8_t eof; ///< 0xA5
};

// ============================================================================
//  EngineerTxPacket
// ============================================================================
struct EngineerTxPacket {
  HeaderFrame header;
  struct {
    uint16_t targetJointPosition[6];  ///< 目标关节位置
    uint16_t targetJointVelocity[6];  ///< ,目标关节速度
    float targetJointEffort[6];
    uint8_t targetGripperCommand;  ///< 夹爪开合命令: open=0, close=1
    uint8_t targetSlotStatus[2];
    uint8_t IntentFinish;          ///< 完成请求并返回Finish
  } data;
  uint8_t eof; ///< 0xA5
};

#pragma pack() // 取消字节对齐

// ============================================================================
//  Print Receive Data
// ---------------------------------------------------------------------------
//  打印解码后的数据包内容，便于调试和验证协议正确性
// ============================================================================
inline void print_rx_packet(const EngineerRxPacket &rx_packet) {
  std::cout << "\n=================== RECEIVE PACKET ===================\n";
  // Header
  std::cout << "Header:\n";
  std::cout << "  SoF    : 0x" << std::hex << std::setw(2) << std::setfill('0')
            << static_cast<int>(rx_packet.header.sof) << std::dec << '\n';
  std::cout << "  Length : " << static_cast<int>(rx_packet.header.len) << '\n';
  std::cout << "  ID     : 0x" << std::hex << std::setw(2) << std::setfill('0')
            << static_cast<int>(rx_packet.header.id) << std::dec << '\n';

  // Data
  std::cout << "Data:\n";

  std::cout << "  Actual Joint Position:\n";
  for (size_t i = 0; i < 7; ++i) {
    const float position = i == 6
                               ? uint_to_float(rx_packet.data.actualJointPosition[i],
                                               0.0F, 0.03F)
                               : uint_to_float(rx_packet.data.actualJointPosition[i]);
    std::cout << "    Joint[" << i << "] : " << std::fixed
              << std::setprecision(6) << position << '\n';
  }

  std::cout << "  Actual Joint Velocity:\n";
  for (size_t i = 0; i < 6; ++i) {
    std::cout << "    Joint[" << i << "] : " << std::fixed
              << std::setprecision(6)
              << uint_to_float(rx_packet.data.actualJointVelocity[i]) << '\n';
  }

  std::cout << "  Custom Joint Position:\n";
  for (size_t i = 0; i < 6; ++i) {
    std::cout << "    Joint[" << i << "] : " << std::fixed
              << std::setprecision(6)
              << uint_to_float(rx_packet.data.customJointPosition[i]) << '\n';
  }

  std::cout << "  Slot Status:\n";
  for (size_t i = 0; i < 2; ++i) {
    std::cout << "    Slot[" << i << "] : "
              << static_cast<unsigned>(rx_packet.data.realSlotStatus[i]) << '\n';
  }

  // IntentStatus (enum)
  std::cout << "  Current Intent (Status):\n";
  uint8_t intent = rx_packet.data.IntentStatus;
  std::cout << "    ID      : " << static_cast<unsigned>(intent) << " (0x"
            << std::hex << std::setw(2) << std::setfill('0')
            << static_cast<unsigned>(intent) << std::dec << ")\n";
  std::cout << "    Meaning : " << intent << '\n';

  // End
  std::cout << "EoF    : 0x" << std::hex << std::setw(2) << std::setfill('0')
            << static_cast<int>(rx_packet.eof) << std::dec << '\n';
  std::cout << "=====================================================\n";
}

// ============================================================================
//  Print Transmit Data
// ---------------------------------------------------------------------------
//  打印加密前的发送数据包内容，便于调试和验证协议正确性
// ============================================================================
inline void print_tx_packet(const EngineerTxPacket &tx_packet) {
  std::cout << "\n=================== TRANSMIT PACKET ===================\n";
  // Header
  std::cout << "Header:\n";
  std::cout << "  SoF    : 0x" << std::hex << std::setw(2) << std::setfill('0')
            << static_cast<int>(tx_packet.header.sof) << std::dec << '\n';
  std::cout << "  Length : " << static_cast<int>(tx_packet.header.len) << '\n';
  std::cout << "  ID     : 0x" << std::hex << std::setw(2) << std::setfill('0')
            << static_cast<int>(tx_packet.header.id) << std::dec << '\n';

  // Data
  std::cout << "Data:\n";

  std::cout << "  Target Joint Position:\n";
  for (size_t i = 0; i < 6; ++i) {
    std::cout << "    Joint[" << i << "] : " << std::fixed
              << std::setprecision(6)
              << uint_to_float(tx_packet.data.targetJointPosition[i]) << '\n';
  }

  std::cout << "  Target Joint Velocity:\n";
  for (size_t i = 0; i < 6; ++i) {
    std::cout << "    Joint[" << i << "] : " << std::fixed
              << std::setprecision(6)
              << uint_to_float(tx_packet.data.targetJointVelocity[i]) << '\n';
  }

  std::cout << "  Target Joint Effort:\n";
  for (size_t i = 0; i < 6; ++i) {
    std::cout << "    Joint[" << i << "] : " << std::fixed
              << std::setprecision(6)
              << tx_packet.data.targetJointEffort[i] << '\n';
  }

  std::cout << "  Target Gripper Command:\n";
  std::cout << "    Value   : "
            << static_cast<unsigned>(tx_packet.data.targetGripperCommand) << '\n';

  std::cout << "  Target Slot Status:\n";
  for (size_t i = 0; i < 2; ++i) {
    std::cout << "    Slot[" << i << "] : "
              << static_cast<unsigned>(tx_packet.data.targetSlotStatus[i]) << '\n';
  }

  // IntentFinish (bool flag: 0/1)
  std::cout << "  Intent Finish (0=running, 1=fin):\n";
  uint8_t fin = tx_packet.data.IntentFinish;
  std::cout << "    Value   : " << static_cast<unsigned>(fin) << " (0x"
            << std::hex << std::setw(2) << std::setfill('0')
            << static_cast<unsigned>(fin) << std::dec << ")\n";
  std::cout << "    State   : " << (fin ? "FIN" : "RUNNING") << '\n';

  // End
  std::cout << "EoF    : 0x" << std::hex << std::setw(2) << std::setfill('0')
            << static_cast<int>(tx_packet.eof) << std::dec << '\n';
  std::cout << "=====================================================\n";
}

} // namespace usb_cdc

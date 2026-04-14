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
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <iostream>

namespace usb_cdc {

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
  // uint8_t reserved; // 保留对齐字节：暂不启用；启用后发送端填 0，接收端忽略
};

// STM32H7 (下位机) Rx Packet
struct H7RxPacket { // id = 1
  HeaderFrame header;
  struct {
    float actualJointPosition[7]; // 当前关节位置
    float actualJointVelocity[6]; // 当前关节速度
    uint8_t realSlotStatus[2];
    uint8_t IntentStatus; ///< 当前意图
  } data;
  uint8_t eof; ///< 0xA5
};

// Custom Controller Rx Packet
struct CCRxPacket { // id = 2
  HeaderFrame header;
  struct {
    float customJointPosition[6]; // 自定义关节角度
  } data;
  uint8_t eof; ///< 0xA5
};

// 控制 arm 速度环位置环 tx packet
struct MotionTxPacket { // id = 1
  HeaderFrame header;
  struct {
    float targetJointPosition[6]; ///< 目标关节位置
    float targetJointVelocity[6]; ///< 目标关节速度
  } data;
  uint8_t eof; ///< 0xA5
};

// 辅助功能 tx packet
struct AuxTxPacket { // id = 2
  HeaderFrame header;
  struct {
    float targetJointEffort[6];   ///< 目标关节力矩
    uint8_t targetGripperCommand; ///< 夹爪开合命令: open=0, close=1
    uint8_t targetSlotStatus[2];
    uint8_t IntentFinish; ///< 完成请求并返回Finish
  } data;
  uint8_t eof; ///< 0xA5
};

#pragma pack() // 取消字节对齐

static_assert(sizeof(HeaderFrame) == 3, "HeaderFrame must be 3 bytes");
static_assert(sizeof(H7RxPacket) == 59, "H7RxPacket must be 59 bytes");
static_assert(sizeof(CCRxPacket) == 28, "CCRxPacket must be 28 bytes");
static_assert(sizeof(MotionTxPacket) == 52, "MotionTxPacket must be 52 bytes");
static_assert(sizeof(AuxTxPacket) == 32, "AuxTxPacket must be 32 bytes");

inline void print_header(const HeaderFrame& header)
{
  std::cout << "Header:\n";
  std::cout << "  SoF    : 0x" << std::hex << std::setw(2) << std::setfill('0')
            << static_cast<int>(header.sof) << std::dec << '\n';
  std::cout << "  Length : " << static_cast<int>(header.len) << '\n';
  std::cout << "  ID     : 0x" << std::hex << std::setw(2) << std::setfill('0')
            << static_cast<int>(header.id) << std::dec << '\n';
}

inline void print_eof(uint8_t eof)
{
  std::cout << "EoF    : 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(eof)
            << std::dec << '\n';
}

// ============================================================================
//  Print Receive Data
// ============================================================================
inline void print_h7_rx_packet(const H7RxPacket& rx_packet)
{
  std::cout << "\n=================== H7 RX PACKET ===================\n";
  print_header(rx_packet.header);
  std::cout << "Data:\n";

  std::cout << "  Actual Joint Position:\n";
  for (size_t i = 0; i < 7; ++i) {
    std::cout << "    Joint[" << i << "] : " << std::fixed << std::setprecision(6)
              << rx_packet.data.actualJointPosition[i] << '\n';
  }

  std::cout << "  Actual Joint Velocity:\n";
  for (size_t i = 0; i < 6; ++i) {
    std::cout << "    Joint[" << i << "] : " << std::fixed << std::setprecision(6)
              << rx_packet.data.actualJointVelocity[i] << '\n';
  }

  std::cout << "  Slot Status:\n";
  for (size_t i = 0; i < 2; ++i) {
    std::cout << "    Slot[" << i << "] : " << static_cast<unsigned>(rx_packet.data.realSlotStatus[i]) << '\n';
  }

  std::cout << "  Current Intent (Status):\n";
  const uint8_t intent = rx_packet.data.IntentStatus;
  std::cout << "    ID      : " << static_cast<unsigned>(intent) << " (0x" << std::hex << std::setw(2)
            << std::setfill('0') << static_cast<unsigned>(intent) << std::dec << ")\n";
  std::cout << "    Meaning : " << intent << '\n';
  print_eof(rx_packet.eof);
  std::cout << "=====================================================\n";
}

inline void print_cc_rx_packet(const CCRxPacket& rx_packet)
{
  std::cout << "\n=================== CC RX PACKET ===================\n";
  print_header(rx_packet.header);
  std::cout << "Data:\n";

  std::cout << "  Custom Joint Position:\n";
  for (size_t i = 0; i < 6; ++i) {
    std::cout << "    Joint[" << i << "] : " << std::fixed << std::setprecision(6)
              << rx_packet.data.customJointPosition[i] << '\n';
  }

  print_eof(rx_packet.eof);
  std::cout << "=====================================================\n";
}

// ============================================================================
//  Print Transmit Data
// ============================================================================
inline void print_motion_tx_packet(const MotionTxPacket& tx_packet)
{
  std::cout << "\n=================== MOTION TX PACKET ===================\n";
  print_header(tx_packet.header);
  std::cout << "Data:\n";

  std::cout << "  Target Joint Position:\n";
  for (size_t i = 0; i < 6; ++i) {
    std::cout << "    Joint[" << i << "] : " << std::fixed << std::setprecision(6)
              << tx_packet.data.targetJointPosition[i] << '\n';
  }

  std::cout << "  Target Joint Velocity:\n";
  for (size_t i = 0; i < 6; ++i) {
    std::cout << "    Joint[" << i << "] : " << std::fixed << std::setprecision(6)
              << tx_packet.data.targetJointVelocity[i] << '\n';
  }

  print_eof(tx_packet.eof);
  std::cout << "=====================================================\n";
}

inline void print_aux_tx_packet(const AuxTxPacket& tx_packet)
{
  std::cout << "\n=================== AUX TX PACKET ===================\n";
  print_header(tx_packet.header);
  std::cout << "Data:\n";

  std::cout << "  Target Joint Effort:\n";
  for (size_t i = 0; i < 6; ++i) {
    std::cout << "    Joint[" << i << "] : " << std::fixed << std::setprecision(6)
              << tx_packet.data.targetJointEffort[i] << '\n';
  }

  std::cout << "  Target Gripper Command:\n";
  std::cout << "    Value   : " << static_cast<unsigned>(tx_packet.data.targetGripperCommand) << '\n';

  std::cout << "  Target Slot Status:\n";
  for (size_t i = 0; i < 2; ++i) {
    std::cout << "    Slot[" << i << "] : " << static_cast<unsigned>(tx_packet.data.targetSlotStatus[i]) << '\n';
  }

  std::cout << "  Intent Finish (0=running, 1=fin):\n";
  const uint8_t fin = tx_packet.data.IntentFinish;
  std::cout << "    Value   : " << static_cast<unsigned>(fin) << " (0x" << std::hex << std::setw(2)
            << std::setfill('0') << static_cast<unsigned>(fin) << std::dec << ")\n";
  std::cout << "    State   : " << (fin ? "FIN" : "RUNNING") << '\n';

  print_eof(tx_packet.eof);
  std::cout << "=====================================================\n";
}

} // namespace usb_cdc

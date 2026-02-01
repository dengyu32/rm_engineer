#pragma once
// ============================================================================
//  Packet
// ----------------------------------------------------------------------------
//  - 定义 USB CDC 收发数据帧的公共头与工程报文结构
//  - 固定 SoF/EoF 边界，方便 DeviceParser 做帧同步
//  - 提供收发数据打印工具，便于串口调试
//  - 使用 #pragma pack(1) 保证与下位机一致的字节布局
// ============================================================================

// C++
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
};


// ============================================================================
//  EngineerReceiveData
// ----------------------------------------------------------------------------
//  - 下位机 → 上位机的状态数据包
//  - 携带 7 轴关节位置与当前意图 ID
//  - 按 HeaderFrame + payload + eof 固定布局
// ============================================================================
struct EngineerReceiveData {
  HeaderFrame header;
  struct {
    float actualJointPosition[7]; // 当前关节位置
    float customJointPosition[6]; // 自定义关节角度
    uint8_t currentIntent; ///< 当前意图
  } data;
  uint8_t eof; ///< 0xA5
};


// ============================================================================
//  EngineerTransmitData
// ----------------------------------------------------------------------------
//  - 上位机 → 下位机的控制数据包
//  - 提供目标关节位置/速度与夹爪指令
//  - 保持与 EngineerReceiveData 同样的帧边界字段
// ============================================================================
struct EngineerTransmitData {
  HeaderFrame header;
  struct {
    float targetJointPosition[6];  ///< 目标关节位置
    float targetJointVelocity[6];  ///< 目标关节速度
    uint8_t gripperCommand;        ///< 夹爪开合指令
  } data;
  uint8_t eof; ///< 0xA5
};

#pragma pack() // 取消字节对齐

/**
 * @brief 数据包打印模板函数（调试用）
 *
 * @param packet 待打印的数据包
 */
inline const char* switch_intent(uint8_t intent_id)
{
  switch (intent_id) {
    case 0:
      return "IDLE";
    case 1:  
      return "AUTO_INIT";
    case 2:
      return "TEST_SOLVE";
    case 3:
      return "TEST_CARTESIAN";
    case 4:
      return "AUTO_GRAB";
    case 5:
      return "AUTO_STORE";
    case 6:
      return "AUTO_GET";
    case 11:
      return "TELEOP_SERVO";
    default:
      return "UNKNOWN";
  }
}

inline void engineer_print_receive_data(const EngineerReceiveData &rx_data) {
  std::cout << "\n================ RECEIVE PACKET ================\n";
  // Header
  std::cout << "Header:\n";
  std::cout << "  SoF    : 0x" << std::hex << std::setw(2) << std::setfill('0')
            << static_cast<int>(rx_data.header.sof) << std::dec << '\n';
  std::cout << "  Length : " << static_cast<int>(rx_data.header.len) << '\n';
  std::cout << "  ID     : 0x" << std::hex << std::setw(2) << std::setfill('0')
            << static_cast<int>(rx_data.header.id) << std::dec << '\n';
  // Data
  std::cout << "Data:\n";
  std::cout << "  Actual Joint Position:\n";
  for (size_t i = 0; i < 7; ++i) {
    std::cout << "    Joint[" << i << "] : " << std::fixed
              << std::setprecision(6) << rx_data.data.actualJointPosition[i]
              << '\n';
  }
  std::cout << "  Current Intent :\n";
  uint8_t intent_id = rx_data.data.currentIntent;
  std::cout << "    ID      : " << static_cast<unsigned int>(intent_id) << " (0x"
            << std::hex << std::setw(2) << std::setfill('0')
            << static_cast<unsigned int>(intent_id) << std::dec << ")\n";
  std::cout << "    Meaning : " << switch_intent(intent_id) << '\n';
  // End
  std::cout << "EoF    : 0x" << std::hex << std::setw(2) << std::setfill('0')
            << static_cast<int>(rx_data.eof) << std::dec << '\n';
  std::cout << "===============================================\n";
}

/**
 * @brief 发送数据包打印模板函数（调试用）
 *
 * @param packet 待打印的数据包
 */
inline void engineer_print_transmit_data(const EngineerTransmitData &tx_data) {
  std::cout << "\n================ TRANSMIT PACKET ================\n";
  // Header
  std::cout << "Header:\n";
  std::cout << "  SoF    : 0x" << std::hex << std::setw(2) << std::setfill('0')
            << static_cast<int>(tx_data.header.sof) << std::dec << '\n';
  std::cout << "  Length : " << static_cast<int>(tx_data.header.len) << '\n';
  std::cout << "  ID     : 0x" << std::hex << std::setw(2) << std::setfill('0')
            << static_cast<int>(tx_data.header.id) << std::dec << '\n';
  // Data
  std::cout << "Data:\n";
  std::cout << "  Target Joint Position:\n";
  for (size_t i = 0; i < 6; ++i) {
    std::cout << "    Joint[" << i << "] : " << std::fixed
              << std::setprecision(6) << tx_data.data.targetJointPosition[i]
              << '\n';
  }
  std::cout << "  Target Joint Velocity:\n";
  for (size_t i = 0; i < 6; ++i) {
    std::cout << "    Joint[" << i << "] : " << std::fixed
              << std::setprecision(6) << tx_data.data.targetJointVelocity[i]
              << '\n';
  }
  std::cout << "  Gripper Command:\n";
  std::cout << "    Value   : " << tx_data.data.gripperCommand << " (0x"
            << std::hex << tx_data.data.gripperCommand << std::dec << ")\n";
  // End
  std::cout << "EoF    : 0x" << std::hex << std::setw(2) << std::setfill('0')
            << static_cast<int>(tx_data.eof) << std::dec << '\n';
  std::cout << "===============================================\n";
}

} // namespace usb_cdc

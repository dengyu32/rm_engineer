# USB CDC Node
基于 USB CDC 的 ROS2 通信节点，用于上位机与下位机的高速串口数据交换。

**架构概览**
```text
USB CDC Device (MCU)
    ↓ USB CDC (libusb)
┌──────────────────────────────────────────┐
│              UsbCdcNode                  │
│  ┌────────────────────────────────────┐  │
│  │     Device (libusb PIMPL)           │  │ ← 打开设备 / 发送 / 事件循环
│  └────────────────────────────────────┘  │
│  ┌────────────────────────────────────┐  │
│  │     DeviceParser (frame sync)       │  │ ← 帧同步 + CRC 校验 + 分发
│  └────────────────────────────────────┘  │
│  ┌────────────────────────────────────┐  │
│  │     ROS Interfaces                  │  │ ← JointState / Joints / Intent
│  └────────────────────────────────────┘  │
└──────────────────────────────────────────┘
    ↓
ROS2 Topics
```

**核心模块**
1. `Device` (libusb 封装)
- 异步 bulk IN 接收，同步 bulk OUT 发送
- 热插拔检测与重连
- 独立事件循环线程

2. `DeviceParser`
- SoF / EoF 帧同步
- CRC16 校验
- 通过帧 ID 分发解析回调

3. `UsbCdcNode`
- 订阅关节/夹爪/意图指令
- 发布 JointState / Joints / Intent

**通信协议**
- 帧结构：`SoF | len | id | crc | payload | EoF`
- CRC 覆盖范围：`len + id + payload`
- CRC 算法：`CRC-16/MODBUS`

`HeaderFrame`
```cpp
struct HeaderFrame {
  uint8_t sof;   // 0x5A
  uint8_t len;   // payload length
  uint8_t id;    // packet id
  uint16_t crc;  // CRC16(MODBUS)
};
```

接收包：`EngineerReceiveData`
```text
actualJointPosition[7]
customJointPosition[6]
IntentStatus
IntentAck
```

发送包：`EngineerTransmitData`
```text
targetJointPosition[6]
targetJointVelocity[6]
gripperCommand
IntentFinish
```

**Intent 信号语义**
- `IntentStatus`: 下位机当前意图状态
- `IntentAck`: 下位机确认信号
- `IntentFinish`: 上位机完成信号（电平信号）

**话题**
发布：
- `intent_out_topic` (`engineer_interfaces/Intent`)
- `joint_states_topic` (`sensor_msgs/JointState`)
- `joint_states_verbose_topic` (`engineer_interfaces/Joints`)
- `joint_states_custom_topic` (`engineer_interfaces/Joints`)

订阅：
- `intent_in_topic` (`engineer_interfaces/Intent`)
- `joint_cmd_topic` (`engineer_interfaces/Joints`)
- `gripper_cmd_topic` (`engineer_interfaces/GripperCommand`)

**参数**
参数由 `config/usb_cdc_node.yaml` 与 `BaseRobotConfig` 提供。

常用参数：
- `vendor_id` / `product_id`
- `publish_period_ms` / `send_period_ms`
- `intent_out_topic` / `intent_in_topic`
- `joint_states_topic` / `joint_states_verbose_topic` / `joint_states_custom_topic`
- `joint_cmd_topic` / `gripper_cmd_topic`
- `joint_count` / `joint_names`

**使用**
构建：
```bash
colcon build --packages-select usb_cdc
```

启动：
```bash
ros2 launch usb_cdc usb_cdc_node.launch.py
```

自定义参数：
```bash
ros2 launch usb_cdc usb_cdc_node.launch.py \
  params_file:=/path/to/usb_cdc_node.yaml
```

**依赖**
- ROS2 Humble+
- libusb-1.0
- CRCpp
- `engineer_interfaces`

**注意事项**
- 帧同步依赖完整帧传输，若设备端可能粘包/拆包，需升级解析器为流式状态机。
- CRC 未在实机上验证时，需与下位机参数对齐。

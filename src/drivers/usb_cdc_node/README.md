# USB CDC Node
基于 USB CDC 的 ROS2 通信节点，用于上位机与下位机的高速串口数据交换。

**架构概览**
```text
USB CDC Device (MCU)
    ↓ USB CDC (libusb)
┌──────────────────────────────────────────┐
│              UsbCdcNode                  │
│  ┌────────────────────────────────────┐  │
│  │     Device (libusb + frame sync)    │  │ ← 打开设备 / 发送 / 事件循环 / 拼帧回调
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
- SoF / len / EoF 流式拼帧
- 处理拆包与粘包
- 完整帧到达后回调上层解析

2. `UsbCdcNode`
- 订阅关节/夹爪/意图指令
- 发布 JointState / Joints / Intent

**通信协议**
- 帧结构：`SoF | len | id | payload | EoF`
- 关节位置、速度、力矩字段使用 `float` 传输。
- 反馈与控制命令拆成多个小包，每个完整帧小于 64 字节。

`HeaderFrame`
```cpp
struct HeaderFrame {
  uint8_t sof;   // 0x5A
  uint8_t len;   // payload length
  uint8_t id;    // packet id
  // uint8_t reserved; // not enabled yet
};
```

接收包：`H7RxPacket`，`id = 0x01`
```text
actualJointPosition[7]  float
actualJointVelocity[6]  float
realSlotStatus[2]
IntentStatus
```
完整帧长度为 59 字节。

接收包：`CCRxPacket`，`id = 0x02`
```text
customJointPosition[6]  float
```
完整帧长度为 28 字节。

发送包：`MotionTxPacket`，`id = 0x01`
```text
targetJointPosition[6]  float
targetJointVelocity[6]  float
```
完整帧长度为 52 字节。

发送包：`AuxTxPacket`，`id = 0x02`
```text
targetJointEffort[6]    float
targetGripperCommand
targetSlotStatus[2]
IntentFinish
```
完整帧长度为 32 字节。

**Intent 信号语义**
- `IntentStatus`: 下位机当前意图状态
- `IntentFinish`: 上位机完成信号（电平信号）

**话题**
发布：
- `intent_cmd_topic` (`engineer_interfaces/Intent`)
- `joint_states_topic` (`sensor_msgs/JointState`)
- `joint_states_verbose_topic` (`engineer_interfaces/Joints`)
- `joint_states_custom_topic` (`engineer_interfaces/Joints`)
- `slot_state_topic` (`engineer_interfaces/Slots`)

订阅：
- `intent_fb_topic` (`engineer_interfaces/Intent`)
- `joint_cmd_topic` (`engineer_interfaces/Joints`)
- `gripper_cmd_topic` (`engineer_interfaces/Gripper`)
- `slot_cmd_topic` (`engineer_interfaces/Slots`)

**参数**
参数由 `config/usb_cdc_node.yaml` 与 `JointResetConfig` / `IntentResetConfig` / `GripperResetConfig` 提供。

常用参数：
- `vendor_id` / `product_id`
- `publish_period_ms` / `send_period_ms`
- `intent_cmd_topic` / `intent_fb_topic`
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
- `engineer_interfaces`

**注意事项**
- 当前解析器已支持 USB CDC 的拆包与粘包。
- 发送侧使用 latest-only 待发送槽和独立 TX 线程，USB 写阻塞不会卡住 200Hz timer，旧控制包会被最新包覆盖。
- `len` 为 `uint8_t`，单帧 payload 最大为 255 字节。

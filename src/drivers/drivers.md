# Drivers

当前驱动层包含两个节点：

- `usb_cdc_node`
  面向实机，通过 USB CDC 与下位机通信。
- `fake_system_node`
  面向联调和上层测试，直接在 ROS 内部回环关节/夹爪/槽位状态。

## usb_cdc_node

`usb_cdc_node` 负责三件事：

- 打开 USB 设备并持续拉取 `libusb` 事件。
- 将下位机反馈包解析为 ROS 话题。
- 将 ROS 控制命令编码为固定长度发送包。

### 收发协议

接收包 `EngineerReceiveData`：

- `actualJointPosition[7]`
- `customJointPosition[6]`
- `realSlotStatus[2]`
- `IntentStatus`

发送包 `EngineerTransmitData`：

- `targetJointPosition[6]`
- `targetJointVelocity[6]`
- `targetGripperCommand`
- `targetSlotStatus[2]`
- `IntentFinish`

当前代码里 CRC 字段未启用，实际发送结构为：

- `SoF`
- `len`
- `id`
- `payload`
- `EoF`

### 当前行为

- 节点启动时，发送缓存会直接初始化为 `startup_joint_targets`。
- 默认启动姿态为 `HOME = [0.0, -0.6109, -2.1293, 0.0, 0.0, 0.0]`。
- 不再依赖“首包反馈后再解锁发送”的保护逻辑。
- 断连后会把发送缓存重新置回 `startup_joint_targets`，等待设备恢复。
- `rx_data_` 仅作为最新反馈快照使用，不再承担启动姿态 seed 逻辑。

### 主要话题

发布：

- `intent_cmd_topic`
- `joint_states_topic`
- `joint_states_verbose_topic`
- `joint_states_custom_topic`
- `slot_state_topic`

订阅：

- `intent_fb_topic`
- `joint_cmd_topic`
- `gripper_cmd_topic`
- `slot_cmd_topic`

### 关键参数

- `vendor_id`
- `product_id`
- `publish_period_ms`
- `send_period_ms`
- `debug_override_enabled`
- `debug_intent_id`
- `startup_joint_targets`

## fake_system_node

`fake_system_node` 用于无实机联调：

- 订阅关节、夹爪、槽位、意图反馈命令。
- 直接维护一份内部“假状态”。
- 周期性发布 `JointState`、`Joints`、`Intent`、`Slots`。

它的作用主要是替代下位机，验证上层控制链路和话题接口是否正常。

## 

printf '%s\n' 'SUBSYSTEM=="usb", ATTR{idVendor}=="0483", ATTR{idProduct}=="5740", MODE="0660", GROUP="dialout"' | sudo tee /etc/udev/rules.d/99-usb-cdc.rules >/dev/null

sudo udevadm control --reload-rules
sudo udevadm trigger

ls -l /dev/bus/usb/001/006

 libusb 没权限/无法脱离内核驱动
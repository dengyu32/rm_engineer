# Fake System Node
用于无实机调试的虚拟底层节点，提供 JointState/Joints 与可选的 Intent 话题，便于上层流程联调。

**架构概览**
```text
arm_servo / controller
    ↓ joint_cmd_topic
┌──────────────────────────────────────────┐
│              FakeSystemNode              │
│  ┌────────────────────────────────────┐  │
│  │   Joint/Gripper State Emulator      │  │ ← 内部状态更新
│  └────────────────────────────────────┘  │
│  ┌────────────────────────────────────┐  │
│  │   ROS Publishers (JointState/Joints)│  │
│  └────────────────────────────────────┘  │
└──────────────────────────────────────────┘
    ↓
/joint_states, /joint_states_custom, /joint_states_verbose
```

**核心逻辑**
- 订阅 `joint_cmd_topic` 与 `gripper_cmd_topic`，更新虚拟关节与夹爪状态
- 周期发布 `JointState` 与 `Joints`（verbose/custom）
- 可选发布 `Intent`（IDLE）用于上层联调

**话题**
发布：
- `joint_states_topic` (`sensor_msgs/JointState`)
- `joint_states_verbose_topic` (`engineer_interfaces/Joints`)
- `joint_states_custom_topic` (`engineer_interfaces/Joints`)
- `intent_out_topic` (`engineer_interfaces/Intent`, 可选)

订阅：
- `joint_cmd_topic` (`engineer_interfaces/Joints`)
- `gripper_cmd_topic` (`engineer_interfaces/GripperCommand`)

**参数**
参数由 `config/fake_system_node.yaml` 与 `BaseRobotConfig` 提供。

常用参数：
- `publish_period_ms`
- `publish_intent`
- `initial_joint_positions`
- `initial_gripper_position`
- `intent_out_topic` / `joint_states_topic` / `joint_states_verbose_topic` / `joint_states_custom_topic`
- `joint_cmd_topic` / `gripper_cmd_topic`
- `joint_count` / `joint_names`

**使用**
构建：
```bash
colcon build --packages-select fake_system
```

启动：
```bash
ros2 launch fake_system fake_system_node.launch.py
```

自定义参数：
```bash
ros2 launch fake_system fake_system_node.launch.py \
  params_file:=/path/to/fake_system_node.yaml
```

**数据流**
```text
fake_system -> joint_states -> robot_state_publisher / moveit
fake_system <- joint_commands <- arm_servo
```

**依赖**
- ROS2 Humble+
- `engineer_interfaces`
- `error_code_utils`

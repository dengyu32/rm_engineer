### fake_system 节点
> 用于无实机调试的虚拟底层节点。
**用途说明**
- 无实机时提供 `joint_states` 与 `joint_verbose_states`。
- 可与 `arm_servo` 形成闭环测试。

#### 数据流
> 描述虚拟底层与上层节点的通信关系。
**流入流出**
```text
fake_system -> joint_states -> moveit (robot_state_publisher)
fake_system <- joint_commands <- arm_servo
```
说明：用于验证规划与伺服链路的基本可用性。

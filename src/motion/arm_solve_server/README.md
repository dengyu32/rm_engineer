### arm_solve 节点
> 记录逆解节点的用途、数据流与规划闭环逻辑。
**节点定位**
- 用于单点逆解与 MoveIt 规划输出。
- 可用于视觉联调（target_pose 输入）。

#### 日志样例
> 当前逆解流程的典型日志输出。
**日志示例**
```text
[arm_solve_node-4] [INFO] [1766649763.565044789] [arm_solve_node]: /--------------- 逆解算开始 ---------------/
[arm_solve_node-4] [INFO] [1766649763.565051705] [arm_solve_node]: JointModelGroupName 'engineer_arm'
[arm_solve_node-4] [WARN] [1766649763.565069869] [arm_solve_node]: 当前状态缺少关节[joint1]，使用默认值
[arm_solve_node-4] [INFO] [1766649763.565081475] [arm_solve_node]: [DEBUG] start_state 在关节限制范围内
[arm_solve_node-4] [INFO] [1766649763.565094249] [arm_solve_node]: IK目标: frame='base_link', ee_link='link6', pos(0.000 0.000 0.000), quat(0.000 0.000 0.000 1.000)
[arm_solve_node-4] [ERROR] [1766649765.565439114] [arm_solve_node]: IK 直接解 target_pose_ 失败，尝试 fallback = 当前末端姿态(FK自检)
[arm_solve_node-4] [INFO] [1766649765.565560083] [arm_solve_node]: 当前末端FK: pos = (0.1458, -0.0622, 0.7505) m | quat = (x=-0.00284, y=0.00284, z=-0.70766, w=0.70654) | YPR(zyx) = (yaw=1.5692, pitch=-3.1416, roll=3.1336) rad
[arm_solve_node-4] [WARN] [1766649765.565587014] [arm_solve_node]: IK 使用 fallback_pose 成功,target_pose_ 可能有问题
[arm_solve_node-4] [INFO] [1766649765.565657186] [move_group_interface]: MoveGroup action client/server ready
[arm_solve_node-4] [INFO] [1766649765.584869464] [arm_solve_node]: 轨迹发布完成，共 2 个点
[arm_solve_node-4] [INFO] [1766649765.584877388] [arm_solve_node]: /--------------- 逆解算结束 ---------------/
```
说明：日志用于验证 IK 失败回退与轨迹发布是否正常。

#### 使用范围
> 说明节点适用场景与用途。
**用途**
- 单点 IK 求解与规划验证。
- 作为视觉控制链路中的求解节点。

#### 话题流入流出
> 汇总节点上下游话题。
**数据流**
```text
arm_solve -> joint_commands -> usb_cdc
         <- joint_states_verbose <- usb_cdc
         <- target_pose <- 视觉 (TODO)
```
说明：`target_pose` 来自视觉或上层规划。

#### 内部流程
> 说明初始化与规划闭环逻辑。
**流程说明**
1. 定时器 `init_timer_` 延迟初始化 `move_group_interface`，仅启动一次。
2. 规划闭环：`joint_states_verbose` -> `jointCallback`（避免高频规划）-> `planAndPublish()` -> `joint_commands`。

#### 代码分析
> 预留详细代码拆解。
**状态**
- 待补充具体函数与关键逻辑说明。

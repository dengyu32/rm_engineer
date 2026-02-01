### top_hfsm 节点
> 记录顶层状态机的设计分层、职责边界与最小实现思路。
**定位说明**
- 目标：建立可中断、可复用、可回退的分层状态机架构。
- 结构：Mode（模式仲裁）<-> Task（任务）<-> Action（动作复用）。

#### 架构思路
> 用层级结构表达自动与人工模式的关系。
**层级示意**
```text
Mode (模式仲裁)
├── AUTO
│   ├── AUTOINIT
│   │   ├── MOVE_BY_FIXED (待机点)
│   │   └── FINISH
│   ├── AUTOGRAB
│   │   ├── SELECT_TARGET (VISION)
│   │   ├── MOVE_BY_RANDOM (视觉规划点)
│   │   ├── GRIPPER_CLOSE
│   │   ├── MOVE_BY_CARTESIAN
│   │   ├── MOVE_BY_FIXED (待机点)
│   │   ├── VERIFY
│   │   └── FINISH
│   ├── AUTOSTORE
│   │   ├── SELECT_TARGET (内部决策)
│   │   ├── MOVE_BY_FIXED (放置点)
│   │   ├── GRIPPER_OPEN
│   │   ├── MOVE_BY_FIXED (待机点)
│   │   ├── VERIFY
│   │   └── FINISH
│   ├── AUTOGET
│   │   ├── SELECT_TARGET (内部决策)
│   │   ├── MOVE_BY_FIXED (抓取点)
│   │   ├── GRIPPER_CLOSE
│   │   ├── MOVE_BY_FIXED (待机点)
│   │   ├── VERIFY
│   │   └── FINISH
│   └── ...
├── TELEOPRATION
│   ├── SERVO
│   └── ...
└── IDLE
```
说明：自动任务由子状态机循环执行，人工伺服可中断自动流程。

#### 职责边界
> 明确 Mission、AUTO 与子状态机的职责划分。
**分层说明**
1. Mission
   - 控制是否允许 AUTO / TELEOP。
   - 处理强制中断。
   - 不直接进入子状态机。
2. AUTO（任务选择器）
   - 决定启动哪个自动子任务。
   - 同一时刻只允许一个任务运行。
   - 不关心子状态机内部循环。
3. 子状态机（任务执行）
   - 负责感知、决策、执行、验证与失败回退。
   - 通过状态跳转形成循环，不使用无限 while。

#### 设计原则
> 保证可中断与可复用的工程化结构。
**关键原则**
- 进入 SERVO 时，AUTO 必须可安全打断。
- 退出 SERVO 时，AUTO 从干净状态重启。
- 动作与决策分离：动作只执行，不做选目标。
- 决策函数只返回结果，状态机只负责“去哪一步”。

#### HFSM 基础接口示例
> 用最小接口定义动作状态与状态结果。
**C++ 示例**
```cpp
enum class StateResult {
    RUNNING,
    SUCCESS,
    FAILURE,
    INTERRUPTED
};

class State {
public:
    virtual ~State() = default;
    virtual void onEnter() {}
    virtual StateResult onUpdate() = 0;
    virtual void onExit() {}
};
```
说明：动作状态仅负责动作执行，决策与流程由上层状态机处理。

#### 实现建议
> 先搭建可运行骨架，再逐步扩展。
**建议路线**
- Route 1：自定义 HFSM（控制力强、调试直观）。
- AUTO 内部子任务封装为“可循环、可中断、可失败回退”的子状态机。
- 状态复用：动作状态继承 `IState`，只做动作不做决策。

#### 最小可行实验（MVP）
> 先验证模式切换与子状态机流程正确。
**TODO 清单**
- 实现 IDLE、AUTO 模式切换。
- 在 AUTO 下实现一个 AUTOSOLVE：依次执行两次 `MOVEBYFIX`。
- 建立 ROS2 动作通信，验证进入/退出与中断链路。

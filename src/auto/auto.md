### AUTO
> auto = automatic
> 与 teleop 相对，auto 负责自动完成机械臂的组合动作
> 本系统采用“半自动 + 显式数据流”的三层结构

#### 目录结构
src/auto/
├── auto.md
├── auto_node
├── capabilities
├── step_executor
└── task_orchestrator

#### 分层职责
- Task Layer（task_orchestrator）
  - 只定义步骤序列
  - 显式声明 inputs / outputs / bindings
  - 明确 timeout / retries
  - 不写 Spec，不做推导

- Step Layer（step_executor）
  - 只执行 step
  - 校验 inputs 是否存在
  - bindings 把上下文注入 params
  - 调 capability 执行 command
  - 校验 outputs 并写入上下文
  - 统一处理 retry / timeout / cancel

- Capabilities Layer（capabilities/*）
  - 只解析 command.kind + params 并执行
  - 返回 ExecuteResult（Succeeded/Running/Failed + outputs + ErrorInfo）
  - 不读 Context，不做推导，不做重试

#### 依赖方向
- auto_node -> task_orchestrator -> step_executor
- auto_node -> capabilities/*
- capabilities/* -> engineer_interfaces / rclcpp / ...

#### 核心对象
**Command（唯一执行对象）**
```c++
struct Command {
  std::string kind;                     // "arm.move" / "gripper.cmd" / ...
  std::unordered_map<std::string,Value> params; // 具体参数（variant）
};
```

**Step（显式依赖）**
```c++
struct Step {
  std::string id;                       // 唯一
  std::string label;                    // 展示用
  Command command;
  std::vector<ContextKey> inputs;       // 显式依赖
  std::vector<ContextKey> outputs;      // 显式产出
  std::vector<Binding> bindings;        // 绑定规则
  int timeout_ms;
  int max_retries;
};
```

**ContextKey（显式共享）**
```c++
struct ContextKey {
  std::string name;                     // "VisionPose" / "SlotID" / ...
  ContextScope scope;                   // Task | Persist（目前只用 Task）
};
```

**Binding（显式注入）**
```c++
struct Binding {
  ContextKey from;
  std::string to_param;                 // 注入到 command.params 的字段名
};
```

#### StepExecutor 执行流程（线性）
1. 读取 TaskPlan
2. 校验 inputs 在 Context 中是否存在
3. 依据 bindings 注入 command.params
4. 调 capability 执行 command
5. capability 返回 outputs
6. StepExecutor 校验 outputs 并写入 Context
7. 进入下一步

#### Capabilities Bridge
```c++
ExecuteResult run(const Command &cmd)
void cancel()
const char* lastError()
```

**ExecuteResult**
```c++
struct ExecuteResult {
  ExecuteStatus status;                 // Running | Succeeded | Failed
  std::unordered_map<std::string,Value> outputs;
  ErrorInfo error;                      // Failed 时必填
};
```

#### Command kind 约定
- arm.move
- gripper.cmd
- vision.detect
- slot.select
- slot.lock
- slot.unlock

#### 任务编排示例
**AUTO_GRAB（显式）**
1. vision.detect
   - outputs: VisionPose@task, VisionVector@task
2. arm.move
   - inputs: VisionPose@task
   - bindings: VisionPose -> params.target_pose
3. gripper.cmd
   - params: {action: "close"}
4. arm.move
   - inputs: VisionVector@task
   - bindings: VisionVector -> params.target_vector
5. arm.move
   - params: {target_joints: HOME}

**AUTO_STORE（显式）**
1. slot.select
   - params: {strategy: "put"}
   - outputs: SlotID@task
2. arm.move
   - inputs: SlotID@task
   - bindings: SlotID -> preset.SLOTS -> params.target_joints
3. gripper.cmd
   - params: {action: "open"}
4. slot.lock
   - inputs: SlotID@task
   - bindings: SlotID -> params.slot_id

#### Slot 链路（新）
1. TaskOrchestrator 插入 slot.select + 后续 slot.lock/unlock
2. StepExecutor 执行 command，按 outputs 写入 Context
3. SlotCapabilityBridge 返回 SlotID
4. StepExecutor 依据 bindings 将 SlotID 显式映射到 preset.SLOTS，再注入 target_joints
5. Arm capability 只接收 joints，不感知 slot

#### 代办
- DAG 执行（暂不启用，当前线性）
- Guard / Cleanup 机制扩展
- 行为树替换调度框架（待评估）

### AUTO
> auto = automatic
> 与 teleop 相对，auto 负责自动完成机械臂的组合动作
> 本系统采用“半自动 + 显式数据流”的三层结构

#### 目录结构
src/modes/auto/
├── auto.md
├── auto_library/
├── auto_node/
├── step_executor/
└── task_orchestrator/
    └── config/
        ├── presets.yaml
        └── tasks/
            ├── AUTO_INIT.yaml
            ├── AUTO_GRAB.yaml
            └── ...

src/capabilities/
├── arm
├── vision
├── gripper
└── slot

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

- Capabilities Layer（src/capabilities/*/*）
  - 只解析 command.kind + params 并执行
  - 返回 ExecuteResult（Succeeded/Running/Failed + outputs + ErrorInfo）
  - 不读 Context，不做推导，不做重试

#### 依赖方向
- auto_node -> task_orchestrator -> step_executor
- auto_node -> auto_library
- auto_node -> src/capabilities/*/*
- src/capabilities/*/* -> auto_library / engineer_interfaces / rclcpp / ...
- step_executor -> auto_library
- task_orchestrator -> auto_library

#### 核心对象
**auto_library（层间接口）**
- Command / Value / ExecuteResult / Context / Step / Task
- 放在 auto_library 包，供 task/step/capability 共用

**Command（唯一执行对象）**
```c++
struct Command {
  std::string kind;                     // "arm.move_pose" / "gripper.open" / ...
  std::unordered_map<std::string,Value> params; // 具体参数（variant）
};
```

Value 支持基础数值、字符串与向量：
```c++
using Value = std::variant<
  bool, int64_t, double, std::string,
  std::vector<double>
>;
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
  int post_delay_ms;                    // 动作完成后等待
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
7. 若 post_delay_ms > 0，等待后进入下一步
   - Guard 暂不实现，后续按需求再设计

#### Capability Registry
（不再使用独立 bridge 类，registry 直接注册能力对象）
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

**ErrorInfo**
```c++
struct ErrorInfo {
  std::string message;
  bool retriable;
  std::string detail;                   // 可选
};
```

#### Command kind 约定
- arm.move_pose
- arm.move_joints
- arm.move_vector
- gripper.open
- gripper.close
- vision.detect
- slot.select_put
- slot.select_take
- slot.lock
- slot.unlock

#### Task YAML 编写规则
任务编排已从 C++ 硬编码迁到 YAML，当前只支持线性步骤序列。

- `presets` 单独放在 [presets.yaml](/home/wrj/Desktop/rm_engineer/src/modes/auto/task_orchestrator/config/presets.yaml)
- `tasks` 按“一个任务一个文件”放在 `task_orchestrator/config/tasks/*.yaml`
- 文件名建议与任务名一致，例如 `AUTO_INIT.yaml`
- 一个任务文件只声明一个任务，顶层 key 必须是 `TaskId` 名称，如 `AUTO_INIT`、`AUTO_GRAB`
- 当前支持 `version: 1`
- 不允许重复声明同一个任务
- `IDLE` 不能写入 YAML

**任务文件最小格式**
```yaml
version: 1

AUTO_INIT:
  steps:
    - id: gripper_open
      kind: gripper.open

    - id: move_home_joints
      kind: arm.move_joints
      timeout_ms: 8000
      max_retries: 1
      params:
        target_joints:
          type: double_array
          preset: HOME
```

**step 字段**
- `id`: 必填，step 唯一标识
- `kind`: 必填，对应 capability 的 `command.kind`
- `label`: 可选，默认等于 `id`
- `timeout_ms`: 可选
- `post_delay_ms`: 可选
- `max_retries`: 可选
- `retries`: 兼容旧写法，等价于 `max_retries`
- `params`: 可选，命令参数
- `inputs`: 可选，显式依赖的上下文 key 列表
- `outputs`: 可选，显式产出的上下文 key 列表
- `bindings`: 可选，上下文到参数的注入规则

**params 写法**
- 标量会自动推断为 `bool / int / double / string`
- 数组字面量会解析为 `std::vector<double>`
- 需要显式类型或引用 preset 时，使用带 `type` 的 map

```yaml
params:
  enable: true
  speed: 0.2
  target_pose: [-0.5, 0.1, 0.6, 0.0, 0.0, 0.0, 1.0]
  target_joints:
    type: double_array
    preset: HOME
```

当前推荐的显式类型：
- `string`
- `bool`
- `int`
- `double`
- `double_array`

**presets 写法**
```yaml
version: 1

double_arrays:
  HOME: [0.0, -0.6109, -2.1293, 0.0, 0.0, 0.0]

double_tables:
  SLOTS:
    - [-0.9250, -0.1396, 1.9722, -3.0718, -1.2741, 0.7679]
    - [0.4363, -0.1047, 1.9024, 0.0175, 1.3265, -0.9774]
```

**inputs / outputs 写法**
- 简写：直接写字符串，默认作用域为 `task`
- 完整写法：`name + scope`

```yaml
inputs: [VisionPose]

outputs:
  - name: SlotID
    scope: task
```

当前支持的 `scope`：
- `task`
- `persist`

**bindings 写法**
```yaml
bindings:
  - from: VisionPose
    to_param: target_pose
    op: direct

  - from: SlotID
    to_param: target_joints
    op: index_to_joints_table
    table: SLOTS
```

当前支持的 `BindingOp`：
- `direct`
- `index_to_joints_table`

说明：
- `direct` 表示把上下文值直接写入 `command.params[to_param]`
- `index_to_joints_table` 表示把上下文中的索引映射到 `double_tables`

#### 轻量方法契约
- 固定业务分支直接写进 `kind`，例如 `gripper.open`、`slot.select_put`
- `params` 只保留真正动态的数据，例如 `target_pose`、`target_joints`、`slot_id`
- YAML 加载时会按方法契约做最小校验：
  - `kind` 是否存在
  - 是否写了多余 param
  - 必需 param 是否齐全
  - 必需 outputs 是否声明

#### 任务编排示例
**AUTO_GRAB（显式）**
1. vision.detect
   - outputs: VisionPose@task, VisionVector@task
2. arm.move_pose
   - inputs: VisionPose@task
   - bindings: VisionPose -> params.target_pose
3. gripper.close
4. arm.move_vector
   - inputs: VisionVector@task
   - bindings: VisionVector -> params.target_vector
5. arm.move_joints
   - params: {target_joints: HOME}

**AUTO_STORE（显式）**
1. slot.select_put
   - outputs: SlotID@task
2. arm.move_joints
   - inputs: SlotID@task
   - bindings: SlotID -> preset.SLOTS -> params.target_joints
3. gripper.open
4. slot.lock
   - inputs: SlotID@task
   - bindings: SlotID -> params.slot_id

#### Slot 链路
1. TaskOrchestrator 插入 slot.select_put / slot.select_take + 后续 slot.lock/unlock
2. StepExecutor 执行 command，按 outputs 写入 Context
3. SlotSelectNode 返回 SlotID
4. StepExecutor 依据 bindings 将 SlotID 显式映射到 preset.SLOTS，再注入 target_joints
5. Arm capability 只接收 joints，不感知 slot

#### 代办
- DAG 执行（暂不启用，当前线性）
- Cleanup 机制扩展
- 行为树替换调度框架（待评估）

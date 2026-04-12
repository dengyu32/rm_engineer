# YAML 配置规范

项目里有几类不同用途的 YAML。统一原则是：每个能在 YAML 里配置的字段，都必须能在代码里找到对应的 `declare_get`、`declare_get_checked` 或 `declare_parameter`；否则改 YAML 不会覆盖结构体默认值。

## ROS 参数文件

共享参数组使用 `/**`，表示这份参数会被多个节点复用：

- `src/robots/robot_config/config/joint_reset.yaml`
- `src/robots/robot_config/config/intent_reset.yaml`
- `src/robots/robot_config/config/gripper_reset.yaml`
- `src/robots/robot_config/config/moveit_reset.yaml`

独立节点自己的参数文件使用最终节点名作为顶层 key：

```yaml
node_name:
  ros__parameters:
    some_param: value
```

当前按这个规则维护的节点名包括：

- `auto_node`
- `arm_solve_server`
- `fake_system`
- `pose_marker_node`
- `teleop_node`
- `usb_cdc`

如果配置文件不是给独立节点用，而是给某个节点内部的 helper/capability 对象读取，就使用 `/**`，让参数落到宿主节点的参数空间里。当前 `auto_node` 内部加载的 arm solve client、gripper control、slot select 等配置属于这一类。

`solve_executor.yaml` 是 `executor` 库的专属配置，放在 `src/capabilities/arm/executor/config/solve_executor.yaml`。它当前由 `arm_solve_server` 节点加载，所以顶层 key 使用 `arm_solve_server`。

## 配置结构体

节点主头文件只保留节点本身的接口、成员状态、发布订阅和回调声明。参数加载、校验、summary 统一放到独立 config 头文件里：

- `auto_node/auto_node_config.hpp`
- `teleop_node/teleop_config.hpp`
- `arm_solve_client/arm_solve_client_config.hpp`
- `arm_solve_server/arm_solve_config.hpp`
- `executor/executor_config.hpp`
- `gripper_control_node/gripper_config.hpp`
- `slot_select_node/slot_select_config.hpp`
- `fake_system/fake_system_config.hpp`
- `usb_cdc/usb_cdc_config.hpp`

新增参数时，先在对应 config 头文件里补字段和读取逻辑，再同步更新 YAML 文件；不要只改结构体默认值。

## Launch 接线

安装到 `share/<package>/config` 下的 YAML 不会自动生效，必须出现在对应节点 launch 的 `parameters=[...]` 里。

包内配置文件统一安装到 `share/<package>/config`。CMake 推荐写法是：

```cmake
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)
```

避免写成 `install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/)`，这种写法会把 YAML 直接装到 `share/<package>` 根目录，容易和 launch 里常用的 `config/<file>.yaml` 路径不一致。

如果 launch 里给节点设置了 `name=...`，YAML 顶层 key 要匹配这个最终节点名，而不是 C++ 构造函数里写的默认名。

多个参数文件同时传入时，后面的文件可以覆盖前面的同名参数。建议顺序是：共享默认参数在前，节点专属参数在后。

## Task Orchestrator 配置

`src/modes/auto/task_orchestrator/config/tasks` 下的任务 YAML 不是 ROS 参数文件，不要写 `ros__parameters`。保持当前自定义 schema：

```yaml
version: 1

TASK_NAME:
  steps:
    - id: step_id
      kind: arm.move_joints
```

任务文件名、顶层任务名、`protocol.hpp` 里的 `TaskId` 名称应保持一致。

复用的关节数组、位姿数组放进 `presets.yaml`；只在单个测试任务里用一次的目标，可以直接写在 task 文件里。

## MoveIt 配置

MoveIt 生成或原生消费的 YAML 保持它自己的 schema，不强行改成 ROS 参数文件格式。例如 kinematics、joint limits、controller config、OMPL planning config。

## 视觉配置

本次清理没有改视觉相关 YAML。视觉配置如果要统一，建议单独处理，因为 `detect_node` 和 `pose_from_axis_node` 当前共享同一份参数文件，拆分时需要一起核对 composable node 的节点名和参数覆盖范围。

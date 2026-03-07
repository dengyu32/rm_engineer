### AUTO
> auto 即为 automatic 自动的缩写
> 与 teleop 相对的，auto 负责自动完成机械臂的组合动作,其中涉及到多层设计
> 大致可以分为三层 :
> 最外层业务层 只需设计节点本身 node 和 task 等业务
> 中层编排层 将组合动作拆解成一步一步完成
> 底层能力层 提供 robotic_arm规划控制 、视觉 、gripper io控制 的接口
> 通过封装业务抽象提炼出几个专有名词 Task Step Capabilities 

#### 目录结构
src/auto/
├── auto.md
├── auto_node
│   ├── CMakeLists.txt
│   ├── config
│   │   └── auto_node.yaml
│   ├── include
│   │   └── auto_node
│   │       └── auto_node.hpp
│   ├── launch
│   │   └── start_auto_node.launch.py
│   ├── package.xml
│   └── src
│       └── auto_node.cpp
├── capabilities
│   ├── arm_solve_client
│   │   ├── CMakeLists.txt
│   │   ├── include
│   │   │   └── arm_solve_client
│   │   │       └── arm_solve_client.hpp
│   │   ├── package.xml
│   │   └── src
│   │       └── arm_solve_client.cpp
│   ├── gripper_control_node
│   │   ├── CMakeLists.txt
│   │   ├── include
│   │   │   └── gripper_control_node
│   │   │       └── gripper_control_node.hpp
│   │   ├── package.xml
│   │   └── src
│   │       └── gripper_control_node.cpp
│   └── vision_detect_client
│       ├── CMakeLists.txt
│       ├── include
│       │   └── vision_detect_client
│       │       └── vision_detect_client.hpp
│       ├── package.xml
│       └── src
│           └── vision_detect_client.cpp
├── step_executor
│   ├── CMakeLists.txt
│   ├── include
│   │   └── step_executor
│   │       ├── bridges
│   │       │   ├── arm_capability_bridge.hpp
│   │       │   ├── composite_capability_bridge.hpp
│   │       │   ├── gripper_capability_bridge.hpp
│   │       │   └── vision_capability_bridge.hpp
│   │       ├── capability_bridge.hpp
│   │       └── step_executor.hpp
│   ├── package.xml
│   └── src
│       ├── bridges
│       │   ├── arm_capability_bridge.cpp
│       │   ├── composite_capability_bridge.cpp
│       │   ├── gripper_capability_bridge.cpp
│       │   └── vision_capability_bridge.cpp
│       └── step_executor.cpp
├── task_orchestrator
│   ├── CMakeLists.txt
│   ├── include
│   │   └── task_orchestrator
│   │       ├── i_task_orchestrator.hpp
│   │       └── static_task_orchestrator.hpp
│   ├── package.xml
│   └── src
│       └── static_task_orchestrator.cpp
└── task_step_library
    ├── CMakeLists.txt
    ├── include
    │   └── task_step_library
    │       └── types.hpp
    └── package.xml

其中，auto_node 只做 ROS 通信和生命周期（只做收发调度，不实现具体业务拆解）：
1. 订阅 Intent（含 intent_id）
2. intent_id -> TaskRequest 映射（只做协议转换）
3. 调 task_orchestrator.plan(TaskRequest) 拿 ExecutionPlan
4. 调 step_executor.start(plan) 执行
5. 发布 Intent 反馈（Running/Finished/Aborted）
task_step_library 只定义 task 层和 step 层等具体业务定义，不依赖第三方中间件(ROS MoveIt)
task_orchestrator 负责编排任务，输入 Task 输出 std::vector<Step> 不直接执行 step
step_executor 只做 Step 的具体执行, 与实际能力的接轨
capabilities 则放置特定的能力层，里面按照名称（如arm_solve_client）封装成多个功能包,业务逻辑更清晰 

#### 依赖方向
auto_node -> task_orchestrator -> task_step_library
auto_node -> step_executor -> task_step_library
step_executor -> capabilities/*
capabilities/* -> engineer_interfaces/rclcpp/...

#### 代办
移植行为树

#### 更新记录
从最初的状态机设想到如今，反反复复做了很多工作，先是移植南京理工大学的单层状态机，结果想进一步封装安全检查，同时为未来扩展考虑，从github中找到HFSM2的库文件，移植后，成功跑通一个复合任务，但由于代码重复的地方太多，考虑进一步封装设计，受限必须要做成操作手控制而并非自动，同时话题通信使用状态流，状态流状态严格受上游控制，放弃了HFSM2,自主分层设计了如今的一套代码，对此我只想说，不要自主设计调度框架！！！，学习行为树足矣解决这个问题，同时扩展性肯定比如今的要好不少。悔之晚矣。

ROS 这种成熟的中间件就没必要进一步封装
#### 其他
同样的，自动控制链路与伺服控制链路互斥隔离，但是更为复杂


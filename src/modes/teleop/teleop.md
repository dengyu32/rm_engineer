### TELEOP
> teleop 即 teleoperation 遥操作的缩写
> 在本工程的任务是将同构的自定义控制器远程控制机器人的robotic_arm（下文将这种高频闭环控制方式称为伺服 servo ）
> 其中通信使用ROS中间件和虚拟串口实现，本项目的任务主要是将上游流入的joint_states_custom进行碰撞检查，再原封不动的发回joint_cmd，具体的电机控制在下位机实现（使用位置环）
> 碰撞检测通过MoveIt API实现

#### 文件结构
.
├── teleop.md
└── teleop_node
    ├── include
    │   └── teleop_node
    │       ├── collision_checker.hpp
    │       └── teleop_node.hpp
    ├── launch
    │   └── start_teleop_node.launch.py
    └── src
        ├── collision_checker.cpp
        └── teleop_node.cpp

其中，teleop_node 负责以下几个方面：
- ROS 订阅发布
- intent 控制
- joint 状态缓存
- 动态安全机制
- 调用碰撞检测模块
collision_checker 则提供碰撞检查函数，不依赖ROS Topic
使用 start_teleop_node 以启动该节点

#### 注意
RCLCPP 日志系统需要传入 node 指针，所以内部模块 collision_checker 的日志接入 spdlog ,
避免 node 的滥用

#### 缺点
从碰撞安全位置回退时joint_cmd仍有可能发生越变,（最严重差不多0.8弧度），需要优化
未接入spdlog

#### 更新记录
原本使用 MoveIt realtime servo 链路，将上层接入 hfsm 调用，下层提供开启,关闭 moveit_servo_node 接口
总体链路是将( custom - verbose ) * kp 传入速度 joint_jog 并发布， moveit_servo_node 接收处理成轨迹, arm_servo_node 将轨迹处理成 cmd 发布到串口
现已废弃，原因有三，一是本质位置环需要调参，而 moveit_servo_node 本身为黑盒 ，效果不好；
二是 moveit_servo_node 虽有避免碰撞奇异的功能，但是一旦发生碰撞就会导致程序死机，无法自动规划，无法遥操作；
三是 原逻辑与自动规划部分深耦合，结构不清晰，难以理解，现将 teleop 严格拆开，方便之后调试并扩展代码

#### 其他
伺服控制链路与其他控制链路互斥隔离，便于分析

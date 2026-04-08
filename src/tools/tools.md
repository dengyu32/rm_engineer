### TOOLS
>  tools 包含了一些轻量级的工具功能包
>  这些功能包可用可不用，只是为了提供特定的作用
>  不写入总启动脚本中
>  待扩展

#### 目录结构

├── ee_path_node
├── log_tools
├── pose_marker_node
└── tools.md

其中 ee_path_node 订阅机械臂末端的/tf,发布 path 消息，供 FoxGlove 3D 可视化末端轨迹
log_tools 提供基于 spdlog 的轻量日志封装
pose_marker_node 发布一个固定 PoseStamped 和球形 Marker，供 FoxGlove 可视化位姿点


#### 使用方法

1. ee_path_node

ros2 run ee_path_node ee_path_node
ros2 service call /clear_ee_path std_srvs/srv/Empty

2. object_load

3. pose_marker_node

ros2 run pose_marker_node pose_marker_node --ros-args \
  -p frame_id:=base_link \
  -p pose:="[0.2, 0.1, 0.3, 0.0, 0.0, 0.0, 1.0]"

或使用参数文件：

ros2 run pose_marker_node pose_marker_node --ros-args --params-file \
  src/tools/pose_marker_node/config/pose_marker.yaml

或直接使用 launch：

ros2 launch pose_marker_node pose_marker.launch.py

Foxglove 中查看 `/pose_marker` 话题即可看到点，`/pose_marker_pose` 可查看对应位姿数值。


#### 其他
暂无

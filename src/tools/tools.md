### TOOLS
>  tools 包含了一些轻量级的工具功能包
>  这些功能包可用可不用，只是为了提供特定的作用
>  不写入总启动脚本中
>  待扩展

#### 目录结构

├── ee_path_node
└── tools.md

其中 ee_path_node 订阅机械臂末端的/tf,发布 path 消息，供 FoxGlove 3D 可视化末端轨迹


#### 使用方法

1. ee_path_node

ros2 run ee_path_node ee_path_node
ros2 service call /clear_ee_path std_srvs/srv/Empty

2. object_load



#### 其他
暂无

### 远程连接 foxglove 

#### 原理
NUC (机器人端)运行  Foxglove WebSocket Server
Foxglove 客户端通过 ws:// 或 wss:// 连接

#### NUC 启动  FoxGlove WebSocket
> 使用默认端口 8765
sudo apt install ros-humble-foxglove-bridge
ros2 launch foxglove_bridge foxglove_bridge_launch.xml

#### PC 通过 ip 端口连接
在 FoxGlove 修改 localhost 为 nuc ip
保持端口统一即可

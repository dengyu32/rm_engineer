### COLLISION

#### 需求

通过点云数据离线更新或实时更新规划场景

realsense 点云数据 --> Octomap 数据 -x> 排除自碰撞 
                                  --> 外界碰撞  --> 处理

目前只应用离线规划,先更新一次规划场景,然后调用规划

离线

#### 开发流程

1. 观察点云数据

ros2 launch realsense2_camera rs_launch.py \
  depth_module.depth_profile:=640x480x30 \
  rgb_camera.color_profile:=1280x720x30 \
  pointcloud.enable:=true \
  align_depth.enable:=true

点云话题为 camera/camera/depth/color/points

2. 添加 sensors_3d.yaml 文件




2. moveit_benchmark_resources 录制 3D 点云数据


2. 使用 Octomap

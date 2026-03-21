#!/bin/bash

# For Engineer NUC 

# 等桌面完全起来（很关键）
sleep 10

# bringup
gnome-terminal -- bash -c "source /opt/ros/humble/setup.bash; source /home/nuc/Desktop/rm_engineer/install/setup.bash; ros2 launch engineer_bringup base_bringup.launch.py; exec bash"

# servo
gnome-terminal -- bash -c "source /opt/ros/humble/setup.bash; source /home/nuc/Desktop/rm_engineer/install/setup.bash; ros2 launch arm_servo servo_container.launch.py; exec bash"

# hfsm
gnome-terminal -- bash -c "source /opt/ros/humble/setup.bash; source /home/nuc/Desktop/rm_engineer/install/setup.bash; ros2 launch top_hfsm top_hfsm_node.launch.py; exec bash"

# foxglove
gnome-terminal -- bash -c "source /opt/ros/humble/setup.bash; source /home/nuc/Desktop/rm_engineer/install/setup.bash; ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765; exec bash"

# fake / real（二选一）

# 默认 fake
gnome-terminal -- bash -c "source /opt/ros/humble/setup.bash; source /home/nuc/Desktop/rm_engineer/install/setup.bash; ros2 launch fake_system fake_system_node.launch.py; exec bash"

# 使用真机时，把上面注释掉，用这个
# gnome-terminal -- bash -c "source /opt/ros/humble/setup.bash; source /home/nuc/Desktop/rm_engineer/install/setup.bash; ros2 launch usb_cdc usb_cdc_node.launch.py; exec bash"
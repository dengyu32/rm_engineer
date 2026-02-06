# ----------------------------------------------------------------------------
#  导入库文件
# ----------------------------------------------------------------------------
from launch import LaunchDescription 
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

# ----------------------------------------------------------------------------
#  启动函数
# ----------------------------------------------------------------------------
def generate_launch_description():

    config_path = os.path.join(
        get_package_share_directory("usb_cdc"),
        "config",
        "usb_cdc_node.yaml",
    )

    # -------------------------------
    #  USB CDC 节点 : usb_cdc_node 负责串口通信，通常桥接 USB CDC 设备与 ROS 话题
    # --------------------------------------------------------------
    node_usb_cdc = Node(
        package="usb_cdc",
        executable="usb_cdc_node",
        output="screen",
        parameters=[config_path],
    )
    
    # -------------------------------
    #  LaunchDescription 组装
    # -------------------------------
    return LaunchDescription([
        node_usb_cdc,
    ])

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

    params_utils_share = get_package_share_directory("params_utils")
    joint_reset = os.path.join(params_utils_share, "config", "joint_reset.yaml")
    intent_reset = os.path.join(params_utils_share, "config", "intent_reset.yaml")
    gripper_reset = os.path.join(params_utils_share, "config", "gripper_reset.yaml")

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
        parameters=[joint_reset, intent_reset, gripper_reset, config_path],
    )
    
    # -------------------------------
    #  LaunchDescription 组装
    # -------------------------------
    return LaunchDescription([
        node_usb_cdc,
    ])

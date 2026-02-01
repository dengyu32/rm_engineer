from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # -------------------------------------------------------------------
    #  fake_system_node 节点 : 提供简化的系统模拟，便于在无真实硬件时跑通上层流程
    # -------------------------------------------------------------------
    config_path = os.path.join(
        get_package_share_directory("fake_system"),
        "config",
        "fake_system_node.yaml",
    )

    node_fake_system = Node(
        package="fake_system",
        executable="fake_system_node",
        name="fake_system_node",
        output="screen",
        parameters=[config_path],
    )

    return LaunchDescription([
        node_fake_system,
    ])

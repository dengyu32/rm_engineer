from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # -------------------------------------------------------------------
    #  fake_system_node 节点 : 提供简化的系统模拟，便于在无真实硬件时跑通上层流程
    # -------------------------------------------------------------------
    robot_config_share = get_package_share_directory("robot_config")
    joint_reset = os.path.join(robot_config_share, "config", "joint_reset.yaml")
    intent_reset = os.path.join(robot_config_share, "config", "intent_reset.yaml")
    gripper_reset = os.path.join(robot_config_share, "config", "gripper_reset.yaml")

    config_path = os.path.join(
        get_package_share_directory("fake_system"),
        "config",
        "fake_system_node.yaml",
    )

    params_file = LaunchConfiguration("params_file")

    node_fake_system = Node(
        package="fake_system",
        executable="fake_system_node",
        output="screen",
        parameters=[joint_reset, intent_reset, gripper_reset, params_file],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=config_path,
            description="Path to the fake_system_node parameter file",
        ),
        node_fake_system,
    ])

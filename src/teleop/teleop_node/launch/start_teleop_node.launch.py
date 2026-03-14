from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from xml.dom.minidom import Document
from typing import cast, Optional

import xacro


def load_file(path: str) -> str:
    with open(path, 'r') as f:
        return f.read()


def load_xacro(path: str, mappings: Optional[dict] = None) -> str:
    if mappings is None:
        mappings = {}
    doc = cast(Document, xacro.process_file(path, mappings=mappings))
    return doc.toxml()


def generate_launch_description():
    moveit_config_pkg = get_package_share_directory('engineer_moveit_config')
    urdf_path = os.path.join(moveit_config_pkg, 'config', 'engineer_v4.urdf.xacro')
    srdf_path = os.path.join(moveit_config_pkg, 'config', 'engineer_v4.srdf')

    robot_description = {'robot_description': load_xacro(urdf_path)}
    robot_description_semantic = {'robot_description_semantic': load_file(srdf_path)}

    params_utils_share = get_package_share_directory('params_utils')
    intent_reset = os.path.join(params_utils_share, 'config', 'intent_reset.yaml')
    joint_reset = os.path.join(params_utils_share, 'config', 'joint_reset.yaml')
    moveit_reset = os.path.join(params_utils_share, 'config', 'moveit_reset.yaml')
    teleop_config = os.path.join(
        get_package_share_directory('teleop_node'), 'config', 'teleop_node.yaml')

    return LaunchDescription([
        Node(
            package='teleop_node',
            executable='teleop_node',
            name='teleop_node',
            output='screen',
            parameters=[
                intent_reset,
                joint_reset,
                moveit_reset,
                teleop_config,
                robot_description,
                robot_description_semantic,
            ],
        ),
    ])

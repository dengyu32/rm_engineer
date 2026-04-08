from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    package_share = get_package_share_directory("pose_marker_node")
    params_file = os.path.join(package_share, "config", "pose_marker.yaml")

    return LaunchDescription([
        Node(
            package="pose_marker_node",
            executable="pose_marker_node",
            name="pose_marker_node",
            output="screen",
            parameters=[params_file],
        )
    ])

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("engineer_v5"))
    gazebo_share = Path(get_package_share_directory("gazebo_ros"))
    urdf_path = package_share / "urdf" / "engineer_v5.urdf"

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(gazebo_share / "launch" / "gazebo.launch.py")
                )
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="tf_footprint_base",
                arguments=["0", "0", "0", "0", "0", "0", "base_link", "base_footprint"],
            ),
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                name="spawn_engineer_v5",
                arguments=["-entity", "engineer_v5", "-file", str(urdf_path)],
                output="screen",
            ),
        ]
    )

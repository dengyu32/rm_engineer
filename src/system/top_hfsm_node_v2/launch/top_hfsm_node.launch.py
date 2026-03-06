from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('top_hfsm_node_v2'),
        'config',
        'top_hfsm_node.yaml'
    )

    return LaunchDescription([
        Node(
            package='top_hfsm_node_v2',
            executable='top_hfsm_node_v2',
            name='top_hfsm_v2_node',
            output='screen',
            parameters=[config],
        )
    ])

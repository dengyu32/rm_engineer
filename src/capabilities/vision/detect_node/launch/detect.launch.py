from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = PathJoinSubstitution([
        FindPackageShare('detect_node'),
        'config',
        'detect_node.yaml',
    ])

    container = ComposableNodeContainer(
        name='vision_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='detect_node',
                plugin='arm_controller::DetectNode',
                name='detect_node',
                extra_arguments=[{'use_intra_process_comms': True}],
                parameters=[params_file],
            ),
            ComposableNode(
                package='detect_node',
                plugin='arm_controller::PoseFromAxisNode',
                name='pose_from_axis_node',
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='screen',
    )
    return LaunchDescription([container])

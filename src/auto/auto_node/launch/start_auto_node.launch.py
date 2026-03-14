from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    params_utils_share = get_package_share_directory('params_utils')
    intent_reset = os.path.join(params_utils_share, 'config', 'intent_reset.yaml')
    gripper_reset = os.path.join(params_utils_share, 'config', 'gripper_reset.yaml')

    arm_solve_client_yaml = os.path.join(
        get_package_share_directory('arm_solve_client'), 'config', 'arm_solve_client.yaml')
    gripper_control_yaml = os.path.join(
        get_package_share_directory('gripper_control_node'), 'config', 'gripper_control_node.yaml')
    slot_select_yaml = os.path.join(
        get_package_share_directory('slot_select_node'), 'config', 'slot_select_node.yaml')
    vision_detect_yaml = os.path.join(
        get_package_share_directory('vision_detect_client'), 'config', 'vision_detect_client.yaml')

    config_path = os.path.join(get_package_share_directory('auto_node'), 'config', 'auto_node.yaml')
    return LaunchDescription([
        Node(
            package='auto_node',
            executable='auto_node_main',
            name='auto_node',
            output='screen',
            parameters=[
                intent_reset,
                gripper_reset,
                arm_solve_client_yaml,
                gripper_control_yaml,
                slot_select_yaml,
                vision_detect_yaml,
                config_path,
            ],
        )
    ])

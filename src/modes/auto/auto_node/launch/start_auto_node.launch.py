from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def package_yaml(package_name, yaml_name):
    share = get_package_share_directory(package_name)
    config_path = os.path.join(share, 'config', yaml_name)
    if os.path.exists(config_path):
        return config_path
    return os.path.join(share, yaml_name)


def generate_launch_description():
    robot_config_share = get_package_share_directory('robot_config')
    intent_reset = os.path.join(robot_config_share, 'config', 'intent_reset.yaml')
    gripper_reset = os.path.join(robot_config_share, 'config', 'gripper_reset.yaml')

    arm_solve_client_yaml = package_yaml('arm_solve_client', 'arm_solve_client.yaml')
    gripper_control_yaml = package_yaml('gripper_control_node', 'gripper_control_node.yaml')
    slot_select_yaml = package_yaml('slot_select_node', 'slot_select_node.yaml')
    vision_detect_yaml = package_yaml('vision_detect_client', 'vision_detect_client.yaml')

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

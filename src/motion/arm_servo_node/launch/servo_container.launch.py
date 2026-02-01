import os

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch_param_builder import ParameterBuilder


def generate_launch_description():
    # -------------------------------------------------------------------------
    # MoveIt config
    # -------------------------------------------------------------------------
    moveit_config = (
        MoveItConfigsBuilder("engineer")
        .robot_description(file_path="config/engineer_v3.urdf.xacro")
        .to_moveit_configs()
    )

    # -------------------------------------------------------------------------
    # MoveIt Servo params (namespace: moveit_servo)
    # -------------------------------------------------------------------------
    servo_package = "arm_servo"
    servo_config_path = os.path.join(
        get_package_share_directory(servo_package), "config", "servo_config.yaml"
    )

    servo_params = (
        ParameterBuilder(servo_package)
        .yaml(
            parameter_namespace="moveit_servo",
            file_path=servo_config_path,
        )
        .to_dict()
    )

    # -------------------------------------------------------------------------
    # Composable: MoveIt Servo (moveit_servo::ServoNode)
    # -------------------------------------------------------------------------
    servo_component = ComposableNode(
        package="moveit_servo",
        plugin="moveit_servo::ServoNode",
        name="servo_node",
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
        extra_arguments=[{"use_intra_process_comms": False}],
    )

    # -------------------------------------------------------------------------
    # Composable: ArmServoNode adapter (ArmServoNode as component)
    #
    # - 直接把 YAML 文件路径传给 parameters
    #   这样 launch 会按节点名 arm_servo_node: ros__parameters: 正确注入 override
    # -------------------------------------------------------------------------
    arm_servo_config_path = os.path.join(
        get_package_share_directory(servo_package), "config", "arm_servo_node.yaml"
    )

    arm_servo_component = ComposableNode(
        package="arm_servo",
        plugin="arm_servo::ArmServoNode",
        name="arm_servo_node",
        parameters=[
            arm_servo_config_path,  # 直接传路径
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    # -------------------------------------------------------------------------
    # Container (MultiThreaded)
    # -------------------------------------------------------------------------
    container = ComposableNodeContainer(
        name="arm_servo_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            servo_component,
            arm_servo_component,
        ],
        output="screen",
    )

    return LaunchDescription([container])

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from moveit_configs_utils import MoveItConfigsBuilder
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    robot_name = "engineer"
    description_package='engineer_moveit_config'
    description_file='engineer_v3.urdf.xacro'
    
        # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory(description_package),
        "config",
        "ros2_controllers.yaml",
    )
    
            # RViz
    rviz_config_file = (
        get_package_share_directory(description_package) + "/config/moveit.rviz"
    )

# 加载机器人描述文件，并将其设置为参数
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package),"config",description_file]
                # [FindPackageShare(description_package), "urdf", description_file]
            ),
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    # 加载moveit配置
    moveit_config = (MoveItConfigsBuilder(robot_name)
    .robot_description(file_path="config/engineer_v3.urdf.xacro")
    .trajectory_execution(file_path="config/moveit_controllers.yaml")
    .planning_pipelines(
        pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]
    )).to_moveit_configs()

    # 启动一些 node 
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
        arguments=["--ros-args", "--log-level", "info"],
    )
    
    arm_servo_node = Node(
        package="arm_solve",
        executable="arm_solve_node",
        name="arm_solve_node",
        output="screen",
        parameters=[robot_description],
    )
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    # static_tf = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='static_transform_publisher',
    #     output='both',
    #     arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_link'], # x y z roll pitch yaw frame_id child_frame_id
    # )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # ros2 control 架构
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="both",
    )

    joint_state_broadcaster = Node(package='controller_manager', executable='spawner',
                              arguments=["joint_state_broadcaster","-c", "/controller_manager"],)
    
    arm_controller = Node(package='controller_manager', executable='spawner',
                              arguments=["engineer_arm_controller","-c", "/controller_manager"],)
    
    hand_controller = Node(package='controller_manager', executable='spawner',
                              arguments=["engineer_hand_controller","-c", "/controller_manager"],)
    
    return LaunchDescription(
        [
            move_group_node,
            rviz_node,
            robot_state_publisher_node,
            ros2_control_node,
            joint_state_broadcaster,
            arm_controller,
            hand_controller,
        ]
    )
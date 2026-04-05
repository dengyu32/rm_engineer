# ----------------------------------------------------------------------------
#  导入库文件
# ----------------------------------------------------------------------------
# -------------------------------
#  Launch / ROS2
# -------------------------------
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


# -------------------------------
#  Python Std
# -------------------------------
from xml.dom.minidom import Document
from typing import cast, Optional
import os

# -------------------------------
#  Third-party
# -------------------------------
import yaml
import xacro


def load_file(path: str) -> str:
    with open(path, "r") as f:
        return f.read()


def load_yaml(path: str):
    with open(path, "r") as f:
        return yaml.safe_load(f)


def normalize_request_adapters(adapters) -> list[str]:
    if isinstance(adapters, str):
        return [item for item in adapters.split() if item]
    if isinstance(adapters, list):
        return [str(item) for item in adapters if str(item).strip()]
    return []


def build_ompl_config(path: str, enable_chomp_adapter: bool):
    ompl = load_yaml(path) if os.path.exists(path) else {}
    ompl_ns = ompl.get("ompl")
    if not isinstance(ompl_ns, dict):
        return ompl

    adapters = normalize_request_adapters(ompl_ns.get("request_adapters", ""))
    if enable_chomp_adapter:
        ompl_ns["request_adapters"] = " ".join(adapters)
        return ompl

    ompl_ns["request_adapters"] = " ".join(
        item for item in adapters if item != "chomp/OptimizerAdapter"
    )
    return ompl


def load_xacro(path: str, mappings: Optional[dict] = None) -> str:
    if mappings is None:
        mappings = {}
    try:
        doc = cast(Document, xacro.process_file(path, mappings=mappings))
        return doc.toxml()
    except Exception as e:
        print(f"Error processing xacro file '{path}': {e}")
        return ""


def build_common_params(use_chomp_adapter: bool):
    pkg_config = get_package_share_directory("engineer_moveit_config")
    bringup_config = get_package_share_directory("engineer_bringup")

    urdf_path = os.path.join(pkg_config, "config", "engineer_v4.urdf.xacro")
    srdf_path = os.path.join(pkg_config, "config", "engineer_v4.srdf")
    kinematics_path = os.path.join(pkg_config, "config", "kinematics.yaml")
    joint_limits_path = os.path.join(pkg_config, "config", "joint_limits.yaml")
    moveit_controllers_path = os.path.join(pkg_config, "config", "moveit_controllers.yaml")
    ompl_path = os.path.join(bringup_config, "config", "ompl_planning.yaml")
    chomp_path = os.path.join(bringup_config, "config", "chomp_planning.yaml")

    robot_description = {"robot_description": load_xacro(urdf_path)}
    robot_description_semantic = {"robot_description_semantic": load_file(srdf_path)}
    kinematics = {"robot_description_kinematics": load_yaml(kinematics_path)} if os.path.exists(kinematics_path) else {}
    ompl = build_ompl_config(ompl_path, enable_chomp_adapter=use_chomp_adapter)
    chomp = {"chomp": load_yaml(chomp_path)} if use_chomp_adapter and os.path.exists(chomp_path) else {}
    joint_limits = {"robot_description_planning": load_yaml(joint_limits_path)}
    moveit_controllers = load_yaml(moveit_controllers_path)

    common_params = [
        robot_description,
        robot_description_semantic,
        kinematics,
        ompl,
        chomp,
        joint_limits,
        moveit_controllers,
    ]

    return common_params, robot_description, bringup_config


def build_base_nodes(common_params, robot_description, bringup_config):
    pkg_config = get_package_share_directory("engineer_moveit_config")
    params_utils_share = get_package_share_directory("params_utils")

    rviz_config_path = os.path.join(pkg_config, "config", "moveit.rviz")
    bringup_config_path = os.path.join(bringup_config, "config", "bringup.yaml")
    joint_reset_path = os.path.join(params_utils_share, "config", "joint_reset.yaml")
    moveit_reset_path = os.path.join(params_utils_share, "config", "moveit_reset.yaml")
    solve_executor_path = os.path.join(params_utils_share, "config", "solve_executor.yaml")

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="both",
        arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"],
    )
    node_rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        parameters=common_params,
        arguments=["-d", rviz_config_path],
    )
    node_object_load = Node(
        package="object_load",
        executable="object_load",
        output="both",
        parameters=common_params,
    )
    node_move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=common_params + [bringup_config_path],
    )
    node_arm_solve = Node(
        package="arm_solve_server",
        executable="arm_solve_server_node",
        name="arm_solve_server",
        output="screen",
        parameters=common_params + [joint_reset_path, moveit_reset_path, solve_executor_path, bringup_config_path],
    )
    late_init = TimerAction(
        period=3.0,
        actions=[node_arm_solve],
    )

    return [
        node_robot_state_publisher,
        static_tf,
        node_object_load,
        node_move_group,
        node_rviz,
        late_init,
    ]


def build_robot_nodes(common_params, robot_description, bringup_config):
    pkg_config = get_package_share_directory("engineer_moveit_config")
    params_utils_share = get_package_share_directory("params_utils")
    top_hfsm_config = get_package_share_directory("top_hfsm")

    rviz_config_path = os.path.join(pkg_config, "config", "moveit.rviz")
    bringup_config_path = os.path.join(bringup_config, "config", "bringup.yaml")
    joint_reset_path = os.path.join(params_utils_share, "config", "joint_reset.yaml")
    moveit_reset_path = os.path.join(params_utils_share, "config", "moveit_reset.yaml")
    solve_executor_path = os.path.join(params_utils_share, "config", "solve_executor.yaml")

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    node_arm_solve = Node(
        package="arm_solve",
        executable="arm_solve_server",
        name="arm_solve_server",
        output="screen",
        parameters=common_params + [joint_reset_path, moveit_reset_path, solve_executor_path, bringup_config_path],
    )
    launch_top_hfsm = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            top_hfsm_config, "launch", "top_hfsm_node.launch.py"
        ))
    )
    node_move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=common_params + [bringup_config_path],
    )
    node_rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        parameters=common_params,
        arguments=["-d", rviz_config_path],
    )

    return [
        node_robot_state_publisher,
        node_move_group,
        node_rviz,
        node_arm_solve,
        launch_top_hfsm,
    ]


def launch_setup(context, *args, **kwargs):
    variant = LaunchConfiguration("bringup_variant").perform(context).strip().lower()
    mode = LaunchConfiguration("planning_ab_mode").perform(context).strip().lower()
    use_chomp_adapter = mode != "a"

    common_params, robot_description, bringup_config = build_common_params(
        use_chomp_adapter=use_chomp_adapter
    )

    if variant == "base":
        return build_base_nodes(common_params, robot_description, bringup_config)
    if variant == "robot":
        return build_robot_nodes(common_params, robot_description, bringup_config)

    raise RuntimeError(
        "bringup_variant must be 'base' or 'robot', got: {}".format(variant)
    )


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "bringup_variant",
            default_value="base",
            description="base or robot",
        ),
        DeclareLaunchArgument(
            "planning_ab_mode",
            default_value="b",
            description="A: OMPL only, B: OMPL plus CHOMP adapter",
        ),
        OpaqueFunction(function=launch_setup),
    ])

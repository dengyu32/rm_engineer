# ----------------------------------------------------------------------------
#  导入库文件
# ----------------------------------------------------------------------------
# -------------------------------
#  Launch / ROS2
# -------------------------------
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
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

# ----------------------------------------------------------------------------
#  可复用函数
# ----------------------------------------------------------------------------
def load_file(path: str) -> str:
    with open(path, 'r') as f:
        return f.read()

def load_yaml(path: str):
    with open(path, 'r') as f:
        return yaml.safe_load(f)

def load_xacro(path: str, mappings: Optional[dict] = None) -> str:
    if mappings is None:
        mappings = {}
    try:
        doc = cast(Document, xacro.process_file(path, mappings=mappings))
        return doc.toxml()

    except Exception as e:
        print(f"Error processing xacro file '{path}': {e}")
        return ""

# ----------------------------------------------------------------------------
#  启动函数
# ----------------------------------------------------------------------------
# TODO : 使用组件实现 多进程多终端跑多个节点
def generate_launch_description():

    # ---------------------------------------------------------------------------------------------
    #  自定义配置
    # ---------------------------------------------------------------------------------------------
    
    # 自定义包
    pkg_config      = get_package_share_directory("engineer_moveit_config")
    bringup_config  = get_package_share_directory("engineer_bringup")
    arm_servo_config = get_package_share_directory("arm_servo")
    top_hfsm_config = get_package_share_directory("top_hfsm")
    
    # 自定义路径
    urdf_path        = os.path.join(pkg_config, "config", "engineer_v3.urdf.xacro")
    srdf_path        = os.path.join(pkg_config, "config", "engineer_v3.srdf")
    kinematics_path  = os.path.join(pkg_config, "config", "kinematics.yaml")
    joint_limits_path       = os.path.join(pkg_config, "config", "joint_limits.yaml")
    moveit_controllers_path = os.path.join(pkg_config, "config", "moveit_controllers.yaml")
    rviz_config_path        = os.path.join(pkg_config, "config", "moveit.rviz")
    # initial_positions_path  = os.path.join(pkg_config, "config", "initial_positions.yaml")
    ompl_path               = os.path.join(bringup_config, "config", "ompl_planning.yaml")
    bringup_config_path     = os.path.join(bringup_config, "config", "bringup.yaml")
    
    # 一些配置参数
    # initial_positions = load_yaml(initial_positions_path)["initial_positions"]
    # initial_joint_states = {"initial_joint_positions": initial_positions}
    robot_description = {"robot_description": load_xacro(urdf_path)}
    robot_description_semantic = {"robot_description_semantic": load_file(srdf_path)}
    kinematics = {"robot_description_kinematics": load_yaml(kinematics_path)} if os.path.exists(kinematics_path) else {}
    ompl = load_yaml(ompl_path) if os.path.exists(ompl_path) else {}
    joint_limits = {"robot_description_planning": load_yaml(joint_limits_path)}
    moveit_controllers = load_yaml(moveit_controllers_path)


    # ---------------------------------------------------------------------------------------------
    #  普通参数列表
    # ---------------------------------------------------------------------------------------------
    common_params = [
        robot_description,
        robot_description_semantic,
        kinematics,
        ompl,
        joint_limits,
        moveit_controllers
    ]


    # ---------------------------------------------------------------------------------------------
    #  节点定义
    # ---------------------------------------------------------------------------------------------

    # robot_state_publisher 节点 : 订阅 joint_states，发布 TF 与机器人状态，供 RViz 与 MoveIt 获取模型状态
    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description], 
    )
    
    # arm_solve_server 节点 : 提供 IK/轨迹求解服务，读取 MoveIt 参数与 bringup.yaml 的业务配置
    node_arm_solve = Node(
        package="arm_solve",
        executable="arm_solve_server",
        name="arm_solve_server",
        output="screen",
        parameters=common_params + [bringup_config_path],
    )
    
    # top_hfsm launch 文件（多线程组件容器）
    launch_top_hfsm = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            top_hfsm_config, "launch", "top_hfsm_node.launch.py"
        ))
    )
    
    # arm_servo launch 文件
    launch_arm_servo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            arm_servo_config, "launch", "servo_container.launch.py"
        ))
    )
    
    # move_group 节点 : MoveIt 核心规划节点，使用 URDF/SRDF/规划配置提供规划与执行服务
    node_move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=common_params + [bringup_config_path]
    )
    
    # rviz2 节点 : 可视化节点，读取 robot_description 系列参数与 RViz 配置展示模型与规划结果
    node_rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        parameters=common_params,
        arguments=["-d", rviz_config_path]
    )


    # ---------------------------------------------------------------------------------------------
    #  LaunchDescription 组装（保持节点启动逻辑，去除横幅/分组/延时）
    # ---------------------------------------------------------------------------------------------
    return LaunchDescription([
        node_robot_state_publisher,
        node_move_group,
        node_rviz,
        node_arm_solve,
        launch_top_hfsm,
        launch_arm_servo,
    ])
    

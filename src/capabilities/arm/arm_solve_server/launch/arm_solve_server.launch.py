from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from typing import Optional, cast
from xml.dom.minidom import Document
import os

import xacro
import yaml


def load_file(path: str) -> str:
    with open(path, "r") as f:
        return f.read()


def load_yaml(path: str):
    with open(path, "r") as f:
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


def generate_launch_description():
    robot_config_share = get_package_share_directory("robot_config")
    arm_solve_server_share = get_package_share_directory("arm_solve_server")
    executor_share = get_package_share_directory("executor")

    joint_reset_path = os.path.join(robot_config_share, "config", "joint_reset.yaml")
    moveit_reset_path = os.path.join(robot_config_share, "config", "moveit_reset.yaml")
    arm_solve_server_path = os.path.join(arm_solve_server_share, "config", "arm_solve_server.yaml")
    solve_executor_path = os.path.join(executor_share, "config", "solve_executor.yaml")

    return LaunchDescription([
        Node(
            package="arm_solve_server",
            executable="arm_solve_server_node",
            name="arm_solve_server",
            output="screen",
            parameters=[
                joint_reset_path,
                moveit_reset_path,
                solve_executor_path,
                arm_solve_server_path,
            ],
        ),
    ])

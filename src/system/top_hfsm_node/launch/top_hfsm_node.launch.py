# ----------------------------------------------------------------------------
#  top_hfsm_node.launch.py
# ----------------------------------------------------------------------------
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

import os


# ----------------------------------------------------------------------------
#  generate
# ----------------------------------------------------------------------------
def generate_launch_description():

    # -------------------------------
    #  Paths
    # -------------------------------
    config_pkg = get_package_share_directory("top_hfsm")
    config_path = os.path.join(config_pkg, "config", "top_hfsm_node.yaml")

    # -------------------------------
    #  Container (MultiThreadedExecutor)
    # -------------------------------
    container = ComposableNodeContainer(
        name="top_hfsm_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            ComposableNode(
                package="top_hfsm",
                plugin="top_hfsm::TopHFSMNode",
                name="top_hfsm_node",
                parameters=[config_path],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
        output="screen",
    )

    return LaunchDescription([container])

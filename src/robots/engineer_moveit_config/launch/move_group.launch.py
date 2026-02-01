from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("engineer_v3", package_name="engineer_moveit_config").to_moveit_configs()
    no_octomap = os.path.join(
        get_package_share_directory("engineer_moveit_config"),
        "config",
        "no_octomap.yaml",
    )
    return generate_move_group_launch(moveit_config, parameters=[no_octomap])

"""move_group for the UR7e + Robotiq 2F-85.

Starts no controllers; it uses the ones started by ur7e_robotiq_bringup's
drivers.launch.py. Normally started through ur7e_robotiq_bringup's
moveit.launch.py.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    combined_xacro = str(
        Path(get_package_share_directory("ur7e_robotiq_bringup")) / "urdf" / "ur7e_robotiq.urdf.xacro"
    )

    moveit_config = (
        MoveItConfigsBuilder("ur7e_robotiq", package_name="ur7e_robotiq_moveit_config")
        .robot_description(
            file_path=combined_xacro,
            # Only what actually affects geometry/kinematics: robot_ip and
            # the tool-communication/hardware-interface args ur.urdf.xacro
            # also takes are all in the <ros2_control> block, which
            # move_group never reads (it loads no hardware plugins), so
            # their defaults are fine left unset here.
            mappings={"name": "ur7e_robotiq", "ur_type": "ur7e"},
        )
        .robot_description_semantic(file_path="config/ur7e_robotiq.srdf.xacro")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    return generate_move_group_launch(moveit_config)

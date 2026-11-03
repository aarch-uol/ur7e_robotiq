"""MoveIt move_group configured for the UR7e admittance controller chain."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    combined_xacro = str(
        Path(get_package_share_directory("ur7e_robotiq_bringup"))
        / "urdf"
        / "ur7e_robotiq.urdf.xacro"
    )

    moveit_config = (
        MoveItConfigsBuilder("ur7e_robotiq", package_name="ur7e_robotiq_moveit_config")
        .robot_description(
            file_path=combined_xacro,
            mappings={
                "name": "ur7e_robotiq",
                "ur_type": "ur7e",
                "kinematics_params": LaunchConfiguration("kinematics_params_file"),
            },
        )
        .robot_description_semantic(file_path="config/ur7e_robotiq.srdf.xacro")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers_gui.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    move_group = generate_move_group_launch(moveit_config)
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "kinematics_params_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("ur7e_robotiq_bringup"),
                        "config",
                        "ur7e_calibration.yaml",
                    ]
                ),
            ),
            *move_group.entities,
        ]
    )

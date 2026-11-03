"""Two MoveIt Servo instances used by the GUI stack.

``position_servo_node`` is the UR-native manual-jog path and publishes smoothed
joint positions directly to ``forward_position_controller``. ``servo_node`` is
kept for the existing servo_admittance policy backend and publishes trajectories
to the admittance chain. Their output controllers are mutually exclusive.
"""

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    combined_xacro = str(
        Path(get_package_share_directory("ur7e_robotiq_bringup")) / "urdf" / "ur7e_robotiq.urdf.xacro"
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
        # Servo does not plan, but without this the builder loads its default pipelines (Pilz included)
        # and fails on the missing config/pilz_cartesian_limits.yaml.
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    config_dir = Path(get_package_share_directory("ur7e_robotiq_moveit_config")) / "config"
    admittance_servo_config = yaml.safe_load((config_dir / "ur7e_servo.yaml").read_text())
    position_servo_config = yaml.safe_load((config_dir / "ur7e_servo_position.yaml").read_text())
    admittance_servo_params = {"moveit_servo": admittance_servo_config}
    position_servo_params = {"moveit_servo": position_servo_config}
    # The smoothing plugin (AccelerationLimitedPlugin, MoveIt 2.12) reads these node-level parameters and
    # aborts servo_node if they are unset. Same values as Servo's own.
    smoothing_params = {
        "update_period": float(admittance_servo_config["publish_period"]),
        "planning_group_name": admittance_servo_config["move_group_name"],
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "kinematics_params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("ur7e_robotiq_bringup"), "config", "ur7e_calibration.yaml"]
                ),
            ),
            # Republishes /joint_states with the admittance JTC reference as arm positions (see script docstring).
            Node(
                package="ur7e_robotiq_bringup",
                executable="servo_reference_state.py",
                name="servo_reference_state",
                output="screen",
            ),
            # Keep this name: the policy runner's servo_admittance backend uses /servo_node/*.
            Node(
                package="moveit_servo",
                executable="servo_node",
                name="servo_node",
                output="screen",
                parameters=[
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    moveit_config.joint_limits,
                    admittance_servo_params,
                    smoothing_params,
                ],
            ),
            # Position-mode jogging: Servo -> JointTrajectory -> joint_trajectory_controller (see
            # ur7e_servo_position.yaml for why not Float64MultiArray -> forward_position_controller).
            Node(
                package="moveit_servo",
                executable="servo_node",
                name="position_servo_node",
                output="screen",
                parameters=[
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    moveit_config.joint_limits,
                    position_servo_params,
                    smoothing_params,
                ],
            ),
        ]
    )

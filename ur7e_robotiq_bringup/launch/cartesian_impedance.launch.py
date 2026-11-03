"""Experimental UR7e direct-torque Cartesian impedance bringup.

This launch is intentionally separate from normal MoveIt and position-interface
admittance. By default the controller is configured but left inactive so its
interfaces and parameters can be inspected before effort mode is entered.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("ur7e_robotiq_bringup")
    controllers_file = PathJoinSubstitution(
        [package_share, "config", "ur_controllers_cartesian_impedance.yaml"]
    )

    drivers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([package_share, "launch", "drivers.launch.py"])
        ),
        launch_arguments={
            "robot_ip": LaunchConfiguration("robot_ip"),
            "kinematics_params_file": LaunchConfiguration("kinematics_params_file"),
            "headless_mode": LaunchConfiguration("headless_mode"),
            # Effort control is mutually exclusive with every UR joint position
            # or velocity controller. Never activate the normal initial controller.
            "activate_joint_controller": "false",
            "ur_controllers_file": controllers_file,
            "load_ft_sensor": LaunchConfiguration("load_ft_sensor"),
            "ft_ip_address": LaunchConfiguration("ft_ip_address"),
            "ft_sampling_rate": LaunchConfiguration("ft_sampling_rate"),
            "ft_hardware_biasing": "false",
        }.items(),
    )

    controller_inactive = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "cartesian_impedance_controller",
            "--inactive",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
        ],
        condition=UnlessCondition(LaunchConfiguration("activate_controller")),
    )

    controller_active = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "cartesian_impedance_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
        ],
        condition=IfCondition(LaunchConfiguration("activate_controller")),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_ip", description="IP address of the UR7e controller"),
            DeclareLaunchArgument(
                "kinematics_params_file",
                default_value=PathJoinSubstitution(
                    [package_share, "config", "ur7e_calibration.yaml"]
                ),
            ),
            DeclareLaunchArgument("headless_mode", default_value="false"),
            DeclareLaunchArgument(
                "activate_controller",
                default_value="false",
                description=(
                    "Enter direct joint-torque mode. Keep false until PolyScope >= 5.25.1, "
                    "payload, TCP, clear workspace, and emergency-stop readiness are verified."
                ),
            ),
            DeclareLaunchArgument("load_ft_sensor", default_value="true"),
            DeclareLaunchArgument("ft_ip_address", default_value="192.168.1.1"),
            DeclareLaunchArgument("ft_sampling_rate", default_value="1000"),
            LogInfo(
                msg=(
                    "Cartesian impedance is experimental direct-torque control; "
                    "normal MoveIt and admittance controllers are not started."
                )
            ),
            drivers,
            controller_inactive,
            controller_active,
        ]
    )

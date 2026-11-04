"""UR7e + ATI Gamma + Robotiq bringup with MoveIt through admittance control.

Control chain:
  MoveIt -> admittance_trajectory_controller -> admittance_controller
         -> UR joint position interfaces
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    def launch_file(package, name):
        return PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package), "launch", name])
        )

    drivers = IncludeLaunchDescription(
        launch_file("ur7e_robotiq_bringup", "drivers.launch.py"),
        launch_arguments={
            "robot_ip": LaunchConfiguration("robot_ip"),
            "kinematics_params_file": LaunchConfiguration("kinematics_params_file"),
            "headless_mode": LaunchConfiguration("headless_mode"),
            "activate_joint_controller": "false",
            "ur_controllers_file": PathJoinSubstitution(
                [
                    FindPackageShare("ur7e_robotiq_bringup"),
                    "config",
                    "ur_controllers_admittance.yaml",
                ]
            ),
            "load_ft_sensor": "true",
            "ft_ip_address": LaunchConfiguration("ft_ip_address"),
            "ft_sampling_rate": LaunchConfiguration("ft_sampling_rate"),
            "ft_hardware_biasing": LaunchConfiguration("ft_hardware_biasing"),
        }.items(),
    )

    admittance_chain = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "admittance_controller",
            "admittance_trajectory_controller",
            "--activate-as-group",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
        ],
    )

    move_group = IncludeLaunchDescription(
        launch_file("ur7e_robotiq_moveit_config", "move_group_admittance.launch.py"),
        launch_arguments={
            "kinematics_params_file": LaunchConfiguration("kinematics_params_file"),
        }.items(),
    )
    rviz = IncludeLaunchDescription(
        launch_file("ur7e_robotiq_moveit_config", "moveit_rviz.launch.py"),
        launch_arguments={
            "kinematics_params_file": LaunchConfiguration("kinematics_params_file"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    # The wrench controllers have no input timeout: zero their target if the
    # (GUI) publisher dies while a force is commanded. See ur7e_safety.
    wrench_watchdog = Node(
        package="ur7e_safety",
        executable="wrench_watchdog",
        name="wrench_watchdog",
        output="screen",
        parameters=[{"topics": ["/admittance_controller/wrench_reference"], "timeout_s": 0.3}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_ip", description="IP address of the UR7e controller"),
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
            DeclareLaunchArgument("headless_mode", default_value="false"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("ft_ip_address", default_value="192.168.1.1"),
            DeclareLaunchArgument("ft_sampling_rate", default_value="1000"),
            DeclareLaunchArgument("ft_hardware_biasing", default_value="false"),
            drivers,
            admittance_chain,
            move_group,
            rviz,
            wrench_watchdog,
        ]
    )

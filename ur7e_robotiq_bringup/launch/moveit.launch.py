"""Everything in one go: arm + gripper drivers, MoveIt and RViz.

    ros2 launch ur7e_robotiq_bringup moveit.launch.py robot_ip:=<ROBOT_IP>

Then press Play on the External Control program on the pendant.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    def launch_file(package, name):
        return PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare(package), "launch", name]))

    drivers = IncludeLaunchDescription(
        launch_file("ur7e_robotiq_bringup", "drivers.launch.py"),
        launch_arguments={
            "robot_ip": LaunchConfiguration("robot_ip"),
            "kinematics_params_file": LaunchConfiguration("kinematics_params_file"),
            "headless_mode": LaunchConfiguration("headless_mode"),
            "load_ft_sensor": LaunchConfiguration("load_ft_sensor"),
            "ft_ip_address": LaunchConfiguration("ft_ip_address"),
            "ft_sampling_rate": LaunchConfiguration("ft_sampling_rate"),
            "ft_hardware_biasing": LaunchConfiguration("ft_hardware_biasing"),
        }.items(),
    )
    # move_group and RViz can start right away: MoveIt connects to the arm and
    # gripper controllers when it first executes a motion, not at startup.
    move_group = IncludeLaunchDescription(
        launch_file("ur7e_robotiq_moveit_config", "move_group.launch.py"),
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
                description="Calibration YAML extracted from your robot (see README step 3)",
            ),
            DeclareLaunchArgument("headless_mode", default_value="false"),
            DeclareLaunchArgument("load_ft_sensor", default_value="true"),
            DeclareLaunchArgument("ft_ip_address", default_value="192.168.1.1"),
            DeclareLaunchArgument("ft_sampling_rate", default_value="1000"),
            DeclareLaunchArgument("ft_hardware_biasing", default_value="false"),
            DeclareLaunchArgument("rviz", default_value="true", description="Start RViz with the MotionPlanning panel"),
            drivers,
            move_group,
            rviz,
        ]
    )

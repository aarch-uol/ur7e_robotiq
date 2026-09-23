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
        }.items(),
    )
    # move_group and RViz can start right away: MoveIt connects to the arm and
    # gripper controllers when it first executes a motion, not at startup.
    move_group = IncludeLaunchDescription(launch_file("ur7e_robotiq_moveit_config", "move_group.launch.py"))
    rviz = IncludeLaunchDescription(
        launch_file("ur7e_robotiq_moveit_config", "moveit_rviz.launch.py"),
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_ip", description="IP address of the UR7e controller"),
            DeclareLaunchArgument(
                "kinematics_params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("ur_description"), "config", "ur7e", "default_kinematics.yaml"]
                ),
                description="Calibration YAML extracted from your robot (see README step 3)",
            ),
            DeclareLaunchArgument("headless_mode", default_value="false"),
            DeclareLaunchArgument("rviz", default_value="true", description="Start RViz with the MotionPlanning panel"),
            drivers,
            move_group,
            rviz,
        ]
    )

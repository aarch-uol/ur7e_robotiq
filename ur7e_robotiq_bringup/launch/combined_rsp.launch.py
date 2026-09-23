"""robot_state_publisher for the combined UR7e + 2F-85 model.

Passed to ur_control.launch.py as its description_launchfile. That file only
forwards robot_ip and ur_type explicitly; kinematics_params_file and
headless_mode reach this file because launch configurations set by the parent
launch file stay visible here.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ur_type = LaunchConfiguration("ur_type")

    official_rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ur_robot_driver"), "launch", "ur_rsp.launch.py"])
        ),
        launch_arguments={
            "robot_ip": LaunchConfiguration("robot_ip"),
            "ur_type": ur_type,
            "description_file": PathJoinSubstitution(
                [FindPackageShare("ur7e_robotiq_bringup"), "urdf", "ur7e_robotiq.urdf.xacro"]
            ),
            "kinematics_params_file": LaunchConfiguration("kinematics_params_file"),
            "headless_mode": LaunchConfiguration("headless_mode"),
            "use_tool_communication": "false",
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_ip"),
            DeclareLaunchArgument("ur_type", default_value="ur7e"),
            DeclareLaunchArgument(
                "kinematics_params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("ur_description"), "config", ur_type, "default_kinematics.yaml"]
                ),
            ),
            DeclareLaunchArgument("headless_mode", default_value="false"),
            official_rsp,
        ]
    )

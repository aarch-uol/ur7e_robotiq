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
    use_tool_communication = LaunchConfiguration("use_tool_communication")
    tool_device_name = LaunchConfiguration("tool_device_name")
    tool_tcp_port = LaunchConfiguration("tool_tcp_port")

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
            "use_tool_communication": use_tool_communication,
            "tool_device_name": tool_device_name,
            "tool_tcp_port": tool_tcp_port,
            "tool_voltage": LaunchConfiguration("tool_voltage"),
            "tool_parity": LaunchConfiguration("tool_parity"),
            "tool_baud_rate": LaunchConfiguration("tool_baud_rate"),
            "tool_stop_bits": LaunchConfiguration("tool_stop_bits"),
            "tool_rx_idle_chars": LaunchConfiguration("tool_rx_idle_chars"),
            "tool_tx_idle_chars": LaunchConfiguration("tool_tx_idle_chars"),
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_ip"),
            DeclareLaunchArgument("ur_type", default_value="ur7e"),
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
            DeclareLaunchArgument("use_tool_communication", default_value="false"),
            DeclareLaunchArgument("tool_device_name", default_value="/tmp/ttyUR"),
            DeclareLaunchArgument("tool_tcp_port", default_value="54321"),
            DeclareLaunchArgument("tool_voltage", default_value="0"),
            DeclareLaunchArgument("tool_parity", default_value="0"),
            DeclareLaunchArgument("tool_baud_rate", default_value="115200"),
            DeclareLaunchArgument("tool_stop_bits", default_value="1"),
            DeclareLaunchArgument("tool_rx_idle_chars", default_value="1.5"),
            DeclareLaunchArgument("tool_tx_idle_chars", default_value="3.5"),
            official_rsp,
        ]
    )

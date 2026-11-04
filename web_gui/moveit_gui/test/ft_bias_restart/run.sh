#!/bin/bash
# Runs INSIDE an isolated ur7e_robotiq image (docker run --network none, its own ROS_DOMAIN_ID): a mock UR arm on the
# main controller manager and the robot stack's own F/T controller manager (/ft, ft_controllers.yaml) on a mock
# sensor, then restart.py: sets a bias through the real GUI core, restarts the stack and the GUI, and checks the
# bias comes back on the broadcaster. Started by test_ft_bias_restart_docker.py.
set -e
source /opt/ros/jazzy/setup.bash && source /opt/ur7e_ws/install/setup.bash
W=$(mktemp -d) && cd "$W"
B=$(ros2 pkg prefix ur7e_robotiq_bringup)/share/ur7e_robotiq_bringup
xacro $B/urdf/ur7e_robotiq.urdf.xacro name:=ur7e_robotiq ur_type:=ur7e use_mock_hardware:=true robot_ip:=0.0.0.0 > robot.urdf
sed "s/\$(var tf_prefix)//g" $B/config/ur_controllers_gui.yaml > controllers.yaml
export ARM_URDF=$W/robot.urdf ARM_YAML=$W/controllers.yaml FT_YAML=$B/config/ft_controllers.yaml
python3 /suite/restart.py

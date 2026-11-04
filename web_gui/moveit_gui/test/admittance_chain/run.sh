#!/bin/bash
# Runs INSIDE an isolated ur7e_robotiq image (docker run --network none): mock UR arm + the GUI's admittance chain
# (admittance_controller <- admittance_trajectory_controller) + a simulated, zero F/T topic, then cycles.py.
# Never talks to a robot: no network, its own ROS_DOMAIN_ID. Started by test_admittance_chain_docker.py.
set -e
source /opt/ros/jazzy/setup.bash && source /opt/ur7e_ws/install/setup.bash
W=$(mktemp -d) && cd "$W"
B=$(ros2 pkg prefix ur7e_robotiq_bringup)/share/ur7e_robotiq_bringup
xacro $B/urdf/ur7e_robotiq.urdf.xacro name:=ur7e_robotiq ur_type:=ur7e use_mock_hardware:=true robot_ip:=0.0.0.0 > robot.urdf
sed "s/\$(var tf_prefix)//g" $B/config/ur_controllers_gui.yaml > controllers.yaml
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat robot.urdf)" > rsp.log 2>&1 &
ros2 run controller_manager ros2_control_node --ros-args \
  --params-file /opt/ros/jazzy/share/ur_robot_driver/config/ur7e_update_rate.yaml --params-file controllers.yaml \
  -p robot_description:="$(cat robot.urdf)" > cm.log 2>&1 &
python3 /suite/ftpub.py > ftpub.log 2>&1 &
sleep 4
ros2 run controller_manager spawner joint_state_broadcaster -c /controller_manager > /dev/null 2>&1
ros2 run controller_manager spawner admittance_controller admittance_trajectory_controller -c /controller_manager > /dev/null 2>&1
python3 /suite/cycles.py

#!/usr/bin/env bash
# Move the gripper without MoveIt.
#
#   ros2 run ur7e_robotiq_bringup gripper.sh open
#   ros2 run ur7e_robotiq_bringup gripper.sh close
#   ros2 run ur7e_robotiq_bringup gripper.sh 0.4        # any position from 0.0 (open) to 0.79 (closed)
#   ros2 run ur7e_robotiq_bringup gripper.sh 0.4 20     # optional effort (default 40)
set -e

case "${1:-}" in
  open)  pos=0.0 ;;
  close) pos=0.79 ;;
  "")    sed -n '2,7p' "$0"; exit 1 ;;
  *)     pos="$1" ;;
esac
effort="${2:-40.0}"

ros2 action send_goal /robotiq_gripper_controller/gripper_cmd \
  control_msgs/action/ParallelGripperCommand \
  "{command: {name: ['robotiq_85_left_knuckle_joint'], position: [$pos], effort: [$effort]}}"

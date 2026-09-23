#!/usr/bin/env python3
"""Send one joint goal through MoveIt (plan + execute) and print the result.

    ros2 run ur7e_robotiq_bringup moveit_test.py ur_manipulator wrist_3_joint 1.0
    ros2 run ur7e_robotiq_bringup moveit_test.py gripper robotiq_85_left_knuckle_joint 0.4

error_code 1 = success. See docs/TROUBLESHOOTING.md for the other codes.
"""
import sys
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    PlanningOptions,
)


def main():
    group = sys.argv[1] if len(sys.argv) > 1 else "gripper"
    joint = sys.argv[2] if len(sys.argv) > 2 else "robotiq_85_left_knuckle_joint"
    target = float(sys.argv[3]) if len(sys.argv) > 3 else 0.6

    rclpy.init()
    node = Node("moveit_test_client")
    client = ActionClient(node, MoveGroup, "/move_action")

    node.get_logger().info("waiting for /move_action server...")
    if not client.wait_for_server(timeout_sec=10.0):
        node.get_logger().error("move_action server not available")
        sys.exit(1)

    goal = MoveGroup.Goal()
    req = MotionPlanRequest()
    req.group_name = group
    req.num_planning_attempts = 3
    req.allowed_planning_time = 5.0
    req.max_velocity_scaling_factor = 0.3
    req.max_acceleration_scaling_factor = 0.3

    jc = JointConstraint()
    jc.joint_name = joint
    jc.position = target
    jc.tolerance_above = 0.01
    jc.tolerance_below = 0.01
    jc.weight = 1.0
    constraints = Constraints()
    constraints.joint_constraints = [jc]
    req.goal_constraints = [constraints]

    goal.request = req
    goal.planning_options = PlanningOptions()
    goal.planning_options.plan_only = False
    goal.planning_options.replan = False

    node.get_logger().info(f"sending goal: group={group} joint={joint} target={target}")
    send_future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send_future, timeout_sec=10.0)
    goal_handle = send_future.result()
    if goal_handle is None or not goal_handle.accepted:
        node.get_logger().error("goal rejected or no response")
        sys.exit(1)

    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future, timeout_sec=20.0)
    result = result_future.result()
    if result is None:
        node.get_logger().error("no result received (timeout)")
        sys.exit(1)

    print("error_code:", result.result.error_code.val)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

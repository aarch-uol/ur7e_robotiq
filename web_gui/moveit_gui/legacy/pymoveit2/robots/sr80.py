from typing import List

MOVE_GROUP_ARM: str = "sr80_arm"
# MOVE_GROUP_GRIPPER: str = "gripper"

OPEN_GRIPPER_JOINT_POSITIONS: List[float] = [0.04, 0.04]
CLOSED_GRIPPER_JOINT_POSITIONS: List[float] = [0.0, 0.0]


def joint_names(prefix: str = "L") -> List[str]:
    return [
        prefix + "1",
        prefix + "2",
        prefix + "3",
        prefix + "4",
        prefix + "5",
        prefix + "6",
    ]


def base_link_name(prefix: str = "base_") -> str:
    return prefix + "link"


def end_effector_name(prefix: str = "") -> str:
    return prefix + "tool0"


#def gripper_joint_names(prefix: str = "panda_") -> List[str]:
#    return [
#        prefix + "finger_joint1",
#        prefix + "finger_joint2",
#    ]

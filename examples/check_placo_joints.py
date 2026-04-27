#!/usr/bin/env python3
"""
检查 lift.urdf 在 placo 中的关节顺序
"""
import sys
from pathlib import Path

# Add project paths
sys.path.insert(0, "/home/arx/ARX_new/lerobot_data_collection/arx_vr_data_collection")

import placo
import numpy as np


def main():
    # Load URDF
    urdf_path = Path("/home/arx/ARX_new/robot_description/lift.urdf")

    print(f"Loading URDF from: {urdf_path}")
    print(f"URDF exists: {urdf_path.exists()}")
    print()

    # Load with placo
    robot = placo.RobotWrapper(str(urdf_path))

    print("=" * 70)
    print("Placo Robot Joints:")
    print("=" * 70)
    print(f"Total joints: {len(robot.joint_names)}")
    print()

    for i, name in enumerate(robot.joint_names):
        print(f"  [{i:2d}] {name}")

    print()
    print("=" * 70)
    print("Looking for left/right arm joints:")
    print("=" * 70)

    left_joints = []
    right_joints = []
    other_joints = []

    for i, name in enumerate(robot.joint_names):
        if "left_arm_joint" in name:
            left_joints.append((i, name))
        elif "right_arm_joint" in name:
            right_joints.append((i, name))
        else:
            other_joints.append((i, name))

    print(f"\nLeft arm joints ({len(left_joints)}):")
    for i, name in left_joints:
        print(f"  [{i:2d}] {name}")

    print(f"\nRight arm joints ({len(right_joints)}):")
    for i, name in right_joints:
        print(f"  [{i:2d}] {name}")

    print(f"\nOther joints ({len(other_joints)}):")
    for i, name in other_joints:
        print(f"  [{i:2d}] {name}")

    print()
    print("=" * 70)
    print("Checking floating base:")
    print("=" * 70)
    print(f"Has floating base: {robot.has_floating_base}")

    if robot.has_floating_base:
        print(f"Floating base joints would add 6 more at the beginning")


if __name__ == "__main__":
    main()

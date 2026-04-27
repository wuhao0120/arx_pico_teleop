#!/usr/bin/env python3
"""
Simple offline test for teleop_arx.py core functionality — no hardware or VR needed.

Focuses on the Placo IK integration and joint handling:
  1. Test joint index mapping for revolute joints
  2. Test set_joint/get_joint with dual_R5a.urdf
  3. Test FK/IK pipeline
  4. Verify revolute joints work correctly (not continuous)

Usage:
    conda run -n ur_data python test_teleop_arx_simple.py
"""

import sys
import numpy as np
from pathlib import Path

# Add the project root to path
project_root = Path(__file__).parent
sys.path.insert(0, str(project_root))


def print_header(text):
    print(f"\n{'='*70}")
    print(f"  {text}")
    print(f"{'='*70}")


def print_result(label, passed):
    status = "PASS" if passed else "FAIL"
    print(f"  [{status}] {label}")


def main():
    n_pass = 0
    n_fail = 0

    def check(label, condition):
        nonlocal n_pass, n_fail
        print_result(label, condition)
        if condition:
            n_pass += 1
        else:
            n_fail += 1
        return condition

    # ── 1. Import and basic setup ─────────────────────────────────────────
    print_header("Test 1: Import placo and load URDF")
    try:
        import placo
        urdf_path = project_root / "assets/r5_urdf/dual_R5a.urdf"
        robot = placo.RobotWrapper(str(urdf_path))
        print(f"  [PASS] Loaded URDF: {urdf_path.name}")
        print(f"  Joint count: {len(robot.joint_names())}")
        print(f"  Joints: {robot.joint_names()}")
        n_pass += 1
    except Exception as e:
        print(f"  [FAIL] Load URDF failed: {e}")
        import traceback
        traceback.print_exc()
        n_fail += 1
        return 1

    # ── 2. Build q index map (like teleop_arx.py) ────────────────────────
    print_header("Test 2: Build joint index map")
    try:
        q_indices = {}
        joint_is_continuous = {}
        for jname in robot.joint_names():
            q_before = robot.state.q.copy()
            robot.set_joint(jname, 0.777)
            q_after = robot.state.q.copy()
            robot.set_joint(jname, 0.0)
            changed = np.where(np.abs(q_after - q_before) > 1e-10)[0]
            if len(changed) > 0:
                q_indices[jname] = [int(i) for i in changed]
                joint_is_continuous[jname] = len(changed) == 2
        robot.update_kinematics()

        print(f"  state.q size: {len(robot.state.q)}")
        print(f"  q_indices built for {len(q_indices)} joints")
        check("has q_indices for all joints", len(q_indices) == len(robot.joint_names()))

        # Verify revolute joints have single index
        left_j1_is_revolute = not joint_is_continuous.get('left_joint1', True)
        check("left_joint1 is revolute (single q index)", left_j1_is_revolute)
        right_j1_is_revolute = not joint_is_continuous.get('right_joint1', True)
        check("right_joint1 is revolute (single q index)", right_j1_is_revolute)

        print(f"  left_joint1 q indices: {q_indices.get('left_joint1', 'N/A')}")
        print(f"  right_joint1 q indices: {q_indices.get('right_joint1', 'N/A')}")
    except Exception as e:
        print(f"  [FAIL] Build q index map failed: {e}")
        import traceback
        traceback.print_exc()
        n_fail += 1

    # ── 3. Test set_joint/get_joint for revolute joints ──────────────────
    print_header("Test 3: Set/get joint angles for revolute joints")
    try:
        test_angle = 0.5

        # Directly using robot.set_joint/get_joint (placo handles revolute)
        robot.set_joint('left_joint1', test_angle)
        robot.update_kinematics()
        read_back = robot.get_joint('left_joint1')
        check(f"direct set/get left_joint1: {test_angle:.4f} -> {read_back:.4f}",
              abs(read_back - test_angle) < 1e-6)

        # Using the teleop_arx.py pattern (direct q write)
        idx = q_indices['left_joint2'][0]
        robot.state.q[idx] = test_angle
        robot.update_kinematics()
        read_back_q = robot.state.q[idx]
        check(f"direct q write left_joint2: {test_angle:.4f} -> {read_back_q:.4f}",
              abs(read_back_q - test_angle) < 1e-6)

        # Verify both arms work
        robot.set_joint('right_joint1', -test_angle)
        robot.update_kinematics()
        read_back_right = robot.get_joint('right_joint1')
        check(f"direct set/get right_joint1: {-test_angle:.4f} -> {read_back_right:.4f}",
              abs(read_back_right + test_angle) < 1e-6)
    except Exception as e:
        print(f"  [FAIL] Set/get joint failed: {e}")
        import traceback
        traceback.print_exc()
        n_fail += 1

    # ── 4. Test full arm joint setting ────────────────────────────────────
    print_header("Test 4: Set full arm joints and FK")
    try:
        # Test joint angles (rad)
        left_test = [0.1, -0.3, 0.2, -0.1, 0.15, -0.05]
        right_test = [-0.1, 0.3, -0.2, 0.1, -0.15, 0.05]

        # Set left arm
        for i in range(6):
            robot.set_joint(f'left_joint{i+1}', left_test[i])
        # Set right arm
        for i in range(6):
            robot.set_joint(f'right_joint{i+1}', right_test[i])
        robot.update_kinematics()

        # Read back and verify
        left_read = [robot.get_joint(f'left_joint{i+1}') for i in range(6)]
        right_read = [robot.get_joint(f'right_joint{i+1}') for i in range(6)]

        left_ok = all(abs(l - r) < 1e-6 for l, r in zip(left_test, left_read))
        right_ok = all(abs(l - r) < 1e-6 for l, r in zip(right_test, right_read))

        check("left arm joints read back correctly", left_ok)
        check("right arm joints read back correctly", right_ok)
        print(f"  left:  {[f'{x:.4f}' for x in left_read]}")
        print(f"  right: {[f'{x:.4f}' for x in right_read]}")

        # Get EE poses via FK
        T_left = robot.get_T_world_frame('left_link6')
        T_right = robot.get_T_world_frame('right_link6')
        check("left_link6 FK available", T_left is not None)
        check("right_link6 FK available", T_right is not None)
        print(f"  left EE pos:  [{T_left[0,3]:.4f}, {T_left[1,3]:.4f}, {T_left[2,3]:.4f}]")
        print(f"  right EE pos: [{T_right[0,3]:.4f}, {T_right[1,3]:.4f}, {T_right[2,3]:.4f}]")
    except Exception as e:
        print(f"  [FAIL] Full arm test failed: {e}")
        import traceback
        traceback.print_exc()
        n_fail += 1

    # ── 5. Test IK solver setup ───────────────────────────────────────────
    print_header("Test 5: IK solver setup")
    try:
        from placo import KinematicsSolver

        solver = KinematicsSolver(robot)
        solver.dt = 0.017
        solver.mask_fbase(True)
        solver.add_kinetic_energy_regularization_task(1e-6)

        # Add frame tasks
        left_task = solver.add_frame_task('left_link6', np.eye(4))
        left_task.configure("left_frame", "soft", 1.0)
        right_task = solver.add_frame_task('right_link6', np.eye(4))
        right_task.configure("right_frame", "soft", 1.0)

        # Add manipulability tasks
        left_manip = solver.add_manipulability_task('left_link6', "both", 1.0)
        left_manip.configure("left_manip", "soft", 1e-3)
        right_manip = solver.add_manipulability_task('right_link6', "both", 1.0)
        right_manip.configure("right_manip", "soft", 1e-3)

        check("solver created", solver is not None)
        check("left frame task added", left_task is not None)
        check("right frame task added", right_task is not None)

        # Set initial targets to current poses
        T_left = robot.get_T_world_frame('left_link6')
        T_right = robot.get_T_world_frame('right_link6')
        left_task.T_world_frame = T_left
        right_task.T_world_frame = T_right

        # Solve a few steps
        for _ in range(10):
            solver.solve(True)
            robot.update_kinematics()

        # Check that joints didn't change much (hold position)
        left_after = [robot.get_joint(f'left_joint{i+1}') for i in range(6)]
        right_after = [robot.get_joint(f'right_joint{i+1}') for i in range(6)]

        left_stable = all(abs(a - b) < 0.01 for a, b in zip(left_read, left_after))
        right_stable = all(abs(a - b) < 0.01 for a, b in zip(right_read, right_after))

        check("left arm holds position (IK solve)", left_stable)
        check("right arm holds position (IK solve)", right_stable)
    except Exception as e:
        print(f"  [FAIL] IK solver test failed: {e}")
        import traceback
        traceback.print_exc()
        n_fail += 1

    # ── 6. Test IK with position delta ────────────────────────────────────
    print_header("Test 6: IK with position delta")
    try:
        # Get initial pose
        T_initial = robot.get_T_world_frame('left_link6').copy()

        # Apply +5cm X delta
        T_target = T_initial.copy()
        T_target[0, 3] += 0.05
        left_task.T_world_frame = T_target

        # Solve
        for _ in range(100):
            solver.solve(True)
            robot.update_kinematics()

        # Check result
        T_final = robot.get_T_world_frame('left_link6')
        pos_error = np.linalg.norm(T_final[:3, 3] - T_target[:3, 3])

        print(f"  Initial pos: [{T_initial[0,3]:.4f}, {T_initial[1,3]:.4f}, {T_initial[2,3]:.4f}]")
        print(f"  Target pos:  [{T_target[0,3]:.4f}, {T_target[1,3]:.4f}, {T_target[2,3]:.4f}]")
        print(f"  Final pos:   [{T_final[0,3]:.4f}, {T_final[1,3]:.4f}, {T_final[2,3]:.4f}]")
        print(f"  Position error: {pos_error*1000:.2f}mm")

        # Accept up to 5mm error
        check(f"IK position error < 5mm: {pos_error*1000:.2f}mm", pos_error < 0.005)
    except Exception as e:
        print(f"  [FAIL] IK delta test failed: {e}")
        import traceback
        traceback.print_exc()
        n_fail += 1

    # ── Summary ──────────────────────────────────────────────────────────
    print_header(f"Summary: {n_pass} passed, {n_fail} failed")
    if n_fail == 0:
        print("  All tests PASSED! Placo IK with dual_R5a.urdf is working.\n")
        print("  Key confirmations:")
        print("  - URDF loads successfully")
        print("  - Joints are detected as revolute (not continuous)")
        print("  - Single q index per revolute joint")
        print("  - FK works correctly")
        print("  - IK solver setup and solves correctly")
        return 0
    else:
        print("  Some tests FAILED. Review results above.\n")
        return 1


if __name__ == "__main__":
    exit_code = main()
    sys.exit(exit_code)

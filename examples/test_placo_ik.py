#!/usr/bin/env python3
"""
Offline Placo IK test for lift.urdf — no hardware or VR needed.

Tests the exact same Placo pipeline used by teleop_arx.py:
  1. Load lift.urdf, set up solver + frame tasks + joints regularization
  2. Set initial joint angles → FK → get EE pose
  3. Apply position/orientation deltas to EE target
  4. IK solve → read back joint angles → FK verify

Key insight: lift.urdf has continuous joints that use cos/sin pairs in
state.q. The set_joint/get_joint API does NOT do angle↔cos/sin conversion,
so we must write state.q[idx]=cos(θ), state.q[idx+1]=sin(θ) directly and
read back via atan2(sin, cos).

Usage:
    conda run -n ur_data python test_placo_ik.py
"""

import math
import numpy as np
import placo
import meshcat.transformations as tf

# ── Config ──────────────────────────────────────────────────────────────────
URDF_PATH = "assets/lift_urdf/robot_description/lift.urdf"
SERVO_DT = 0.017

ARMS = {
    "left_arm": {
        "link_name": "left_arm_link6",
        "joint_names": [f"left_arm_joint{i}" for i in range(1, 7)],
    },
    "right_arm": {
        "link_name": "right_arm_link6",
        "joint_names": [f"right_arm_joint{i}" for i in range(1, 7)],
    },
}

# Test joint angles (rad) — a non-zero pose
TEST_JOINTS = {
    "left_arm_joint1": 0.3,
    "left_arm_joint2": -0.5,
    "left_arm_joint3": 0.4,
    "left_arm_joint4": -0.2,
    "left_arm_joint5": 0.6,
    "left_arm_joint6": -0.1,
    "right_arm_joint1": -0.3,
    "right_arm_joint2": 0.5,
    "right_arm_joint3": -0.4,
    "right_arm_joint4": 0.2,
    "right_arm_joint5": -0.6,
    "right_arm_joint6": 0.1,
}


# ── Helpers ─────────────────────────────────────────────────────────────────

def build_q_index_map(robot):
    """Build mapping from joint name → state.q cos-slot index."""
    q_indices = {}
    for jname in robot.joint_names():
        q_before = robot.state.q.copy()
        robot.set_joint(jname, 0.777)
        q_after = robot.state.q.copy()
        robot.set_joint(jname, 0.0)
        changed = np.where(np.abs(q_after - q_before) > 1e-10)[0]
        if len(changed) > 0:
            q_indices[jname] = int(changed[0])
    robot.update_kinematics()
    return q_indices


def set_joint_angle(robot, q_indices, joint_name, angle):
    """Set a continuous joint angle using proper cos/sin encoding."""
    idx = q_indices[joint_name]
    robot.state.q[idx] = math.cos(angle)
    robot.state.q[idx + 1] = math.sin(angle)


def get_joint_angle(robot, q_indices, joint_name):
    """Read a continuous joint angle from cos/sin state."""
    idx = q_indices[joint_name]
    return math.atan2(robot.state.q[idx + 1], robot.state.q[idx])


def set_all_joints(robot, q_indices, joints_dict):
    """Set multiple joint angles and update kinematics."""
    for name, val in joints_dict.items():
        set_joint_angle(robot, q_indices, name, val)
    robot.update_kinematics()


def apply_delta_pose(source_pos, source_quat, delta_pos, delta_rot, eps=1e-6):
    """Same as xrobotoolkit_teleop.utils.geometry.apply_delta_pose."""
    target_pos = source_pos + delta_pos
    angle = np.linalg.norm(delta_rot)
    if angle > eps:
        axis = delta_rot / angle
        rot_delta_quat = tf.quaternion_about_axis(angle, axis)
    else:
        rot_delta_quat = np.array([1.0, 0.0, 0.0, 0.0])
    target_quat = tf.quaternion_multiply(rot_delta_quat, source_quat)
    return target_pos, target_quat


def pose_error(T_actual, T_target):
    """Compute position error (m) and orientation error (deg)."""
    pos_err = np.linalg.norm(T_actual[:3, 3] - T_target[:3, 3])
    R_err = T_actual[:3, :3].T @ T_target[:3, :3]
    angle = np.arccos(np.clip((np.trace(R_err) - 1) / 2, -1.0, 1.0))
    return pos_err, np.degrees(angle)


def setup_solver(robot, q_indices):
    """Create fresh solver with frame tasks and joint regularization.
    Mirrors teleop_arx.py: _check_placo_setup + _check_endeffector_setup + _setup_joints_regularization
    """
    solver = placo.KinematicsSolver(robot)
    solver.dt = SERVO_DT
    solver.mask_fbase(True)
    solver.add_kinetic_energy_regularization_task(1e-6)

    effector_task = {}
    for arm_name, arm_cfg in ARMS.items():
        task = solver.add_frame_task(arm_cfg["link_name"], np.eye(4))
        task.configure(f"{arm_name}_frame", "soft", 1.0)
        manip = solver.add_manipulability_task(arm_cfg["link_name"], "both", 1.0)
        manip.configure(f"{arm_name}_manipulability", "soft", 1e-3)
        effector_task[arm_name] = task

    # Lock non-arm joints using zero-config q values
    arm_joint_names = set()
    for arm_cfg in ARMS.values():
        arm_joint_names.update(arm_cfg["joint_names"])

    q0 = robot.state.q
    non_arm_joints = {}
    for jn in robot.joint_names():
        if jn not in arm_joint_names:
            non_arm_joints[jn] = q0[q_indices[jn]]
    joints_task = solver.add_joints_task()
    joints_task.set_joints(non_arm_joints)
    joints_task.configure("non_arm_regularization", "soft", 1e-4)

    return solver, effector_task


def print_header(text):
    print(f"\n{'='*70}")
    print(f"  {text}")
    print(f"{'='*70}")


def print_result(label, pos_err, rot_err, passed):
    status = "PASS" if passed else "FAIL"
    print(f"  [{status}] {label:30s}  pos_err={pos_err*1000:.3f}mm  rot_err={rot_err:.3f}deg")


# ── Main ────────────────────────────────────────────────────────────────────

def main():
    n_pass = 0
    n_fail = 0

    def check(label, pos_err, rot_err, pos_tol=0.002, rot_tol=1.0):
        nonlocal n_pass, n_fail
        passed = pos_err < pos_tol and rot_err < rot_tol
        print_result(label, pos_err, rot_err, passed)
        if passed:
            n_pass += 1
        else:
            n_fail += 1
        return passed

    # ── 1. Load URDF + build index map ──────────────────────────────────────
    print_header("Loading URDF")
    robot = placo.RobotWrapper(URDF_PATH)
    q_indices = build_q_index_map(robot)
    print(f"  Joints: {len(robot.joint_names())}  |  state.q: {len(robot.state.q)}")
    print(f"  q_indices sample: left_arm_joint1 -> q[{q_indices['left_arm_joint1']}]")

    # ── 2. Joint set/get roundtrip ──────────────────────────────────────────
    print_header("Test 1: Joint set/get roundtrip (cos/sin encoding)")

    set_all_joints(robot, q_indices, TEST_JOINTS)
    all_match = True
    for jn, target_val in TEST_JOINTS.items():
        actual_val = get_joint_angle(robot, q_indices, jn)
        err = abs(actual_val - target_val)
        if err > 1e-6:
            print(f"  [FAIL] {jn}: set={target_val:.6f}, get={actual_val:.6f}, err={err:.2e}")
            all_match = False
            n_fail += 1
    if all_match:
        print("  [PASS] All 12 arm joints: set/get roundtrip matches.")
        n_pass += 1

    # ── 3. FK consistency ───────────────────────────────────────────────────
    print_header("Test 2: FK consistency (det(R) = 1.0)")

    for arm_name, arm_cfg in ARMS.items():
        T_ee = robot.get_T_world_frame(arm_cfg["link_name"])
        det_R = np.linalg.det(T_ee[:3, :3])
        q_readback = [get_joint_angle(robot, q_indices, jn) for jn in arm_cfg["joint_names"]]
        print(f"\n  {arm_name}:")
        print(f"    joints = [{', '.join(f'{v:.4f}' for v in q_readback)}]")
        print(f"    EE pos = [{T_ee[0,3]:.5f}, {T_ee[1,3]:.5f}, {T_ee[2,3]:.5f}]")
        print(f"    det(R) = {det_R:.6f}")
        ok = abs(det_R - 1.0) < 1e-4
        if ok:
            n_pass += 1
            print(f"    [PASS]")
        else:
            n_fail += 1
            print(f"    [FAIL] det(R) should be 1.0")

    # ── 4. Hold position test ───────────────────────────────────────────────
    print_header("Test 3: Hold position (fresh solver, IK target = current FK)")

    # Fresh solver on current joint config
    solver, effector_task = setup_solver(robot, q_indices)

    # Set IK targets to current EE poses
    for arm_name, arm_cfg in ARMS.items():
        T_current = robot.get_T_world_frame(arm_cfg["link_name"])
        effector_task[arm_name].T_world_frame = T_current

    # Solve 100 steps
    for _ in range(100):
        solver.solve(True)
        robot.update_kinematics()

    for arm_name, arm_cfg in ARMS.items():
        T_actual = robot.get_T_world_frame(arm_cfg["link_name"])
        T_target = effector_task[arm_name].T_world_frame
        pos_err, rot_err = pose_error(T_actual, T_target)
        check(f"{arm_name} hold", pos_err, rot_err, pos_tol=0.001, rot_tol=0.5)

    # ── 5. Position deltas (fresh solver for each) ──────────────────────────
    print_header("Test 4: Position deltas (left_arm)")

    position_deltas = [
        ("X +5cm", np.array([0.05, 0.0, 0.0])),
        ("Y +5cm", np.array([0.0, 0.05, 0.0])),
        ("Z +5cm", np.array([0.0, 0.0, 0.05])),
        ("XYZ +3cm each", np.array([0.03, 0.03, 0.03])),
    ]

    for delta_label, delta_xyz in position_deltas:
        # Reset joints and create fresh solver
        set_all_joints(robot, q_indices, TEST_JOINTS)
        solver, effector_task = setup_solver(robot, q_indices)

        # Set current poses as targets first
        for arm_name, arm_cfg in ARMS.items():
            T_current = robot.get_T_world_frame(arm_cfg["link_name"])
            effector_task[arm_name].T_world_frame = T_current

        # Get initial left EE pose
        T_init = robot.get_T_world_frame("left_arm_link6")
        init_xyz = T_init[:3, 3].copy()
        init_quat = tf.quaternion_from_matrix(T_init)

        # Apply delta
        target_xyz, target_quat = apply_delta_pose(init_xyz, init_quat, delta_xyz, np.zeros(3))
        T_target = tf.quaternion_matrix(target_quat)
        T_target[:3, 3] = target_xyz
        effector_task["left_arm"].T_world_frame = T_target

        for _ in range(100):
            solver.solve(True)
            robot.update_kinematics()

        T_actual = robot.get_T_world_frame("left_arm_link6")
        pos_err, rot_err = pose_error(T_actual, T_target)
        check(delta_label, pos_err, rot_err)

    # ── 6. Orientation deltas ───────────────────────────────────────────────
    print_header("Test 5: Orientation deltas (right_arm)")

    orientation_deltas = [
        ("Roll 10deg", np.array([np.radians(10), 0.0, 0.0])),
        ("Pitch 10deg", np.array([0.0, np.radians(10), 0.0])),
        ("Yaw 10deg", np.array([0.0, 0.0, np.radians(10)])),
    ]

    for delta_label, delta_rot in orientation_deltas:
        set_all_joints(robot, q_indices, TEST_JOINTS)
        solver, effector_task = setup_solver(robot, q_indices)

        for arm_name, arm_cfg in ARMS.items():
            T_current = robot.get_T_world_frame(arm_cfg["link_name"])
            effector_task[arm_name].T_world_frame = T_current

        T_init = robot.get_T_world_frame("right_arm_link6")
        init_xyz = T_init[:3, 3].copy()
        init_quat = tf.quaternion_from_matrix(T_init)

        target_xyz, target_quat = apply_delta_pose(init_xyz, init_quat, np.zeros(3), delta_rot)
        T_target = tf.quaternion_matrix(target_quat)
        T_target[:3, 3] = target_xyz
        effector_task["right_arm"].T_world_frame = T_target

        for _ in range(100):
            solver.solve(True)
            robot.update_kinematics()

        T_actual = robot.get_T_world_frame("right_arm_link6")
        pos_err, rot_err = pose_error(T_actual, T_target)
        # Orientation-only moves may have larger pos error near workspace limits
        check(delta_label, pos_err, rot_err, pos_tol=0.01)

    # ── 7. Simultaneous dual-arm deltas ─────────────────────────────────────
    print_header("Test 6: Simultaneous dual-arm deltas")

    set_all_joints(robot, q_indices, TEST_JOINTS)
    solver, effector_task = setup_solver(robot, q_indices)

    for arm_name, arm_cfg in ARMS.items():
        T_current = robot.get_T_world_frame(arm_cfg["link_name"])
        effector_task[arm_name].T_world_frame = T_current

    targets = {}
    for arm_name, arm_cfg in ARMS.items():
        T_init = robot.get_T_world_frame(arm_cfg["link_name"])
        init_xyz = T_init[:3, 3].copy()
        init_quat = tf.quaternion_from_matrix(T_init)

        delta_xyz = np.array([0.03, -0.02, 0.04])
        delta_rot = np.array([np.radians(5), np.radians(-5), np.radians(8)])
        target_xyz, target_quat = apply_delta_pose(init_xyz, init_quat, delta_xyz, delta_rot)

        T_target = tf.quaternion_matrix(target_quat)
        T_target[:3, 3] = target_xyz
        effector_task[arm_name].T_world_frame = T_target
        targets[arm_name] = T_target

    for _ in range(100):
        solver.solve(True)
        robot.update_kinematics()

    for arm_name in ARMS:
        T_actual = robot.get_T_world_frame(ARMS[arm_name]["link_name"])
        pos_err, rot_err = pose_error(T_actual, targets[arm_name])
        check(f"{arm_name} combined", pos_err, rot_err)

    # ── 8. Incremental stepping (simulates real-time VR loop) ───────────────
    print_header("Test 7: Incremental stepping (20 small deltas, 1 solve each)")

    set_all_joints(robot, q_indices, TEST_JOINTS)
    solver, effector_task = setup_solver(robot, q_indices)

    for arm_name, arm_cfg in ARMS.items():
        T_current = robot.get_T_world_frame(arm_cfg["link_name"])
        effector_task[arm_name].T_world_frame = T_current

    T_init = robot.get_T_world_frame("left_arm_link6")
    init_xyz = T_init[:3, 3].copy()
    init_quat = tf.quaternion_from_matrix(T_init)

    total_delta = np.array([0.05, 0.0, 0.0])
    num_steps = 20

    for step in range(num_steps):
        frac = (step + 1) / num_steps
        delta_xyz = total_delta * frac
        target_xyz, target_quat = apply_delta_pose(init_xyz, init_quat, delta_xyz, np.zeros(3))
        T_target = tf.quaternion_matrix(target_quat)
        T_target[:3, 3] = target_xyz
        effector_task["left_arm"].T_world_frame = T_target
        solver.solve(True)
        robot.update_kinematics()

    T_actual = robot.get_T_world_frame("left_arm_link6")
    pos_err, rot_err = pose_error(T_actual, T_target)
    check("left_arm 5cm X (20 steps)", pos_err, rot_err, pos_tol=0.005, rot_tol=2.0)

    # ── 9. Non-arm joints stay locked ───────────────────────────────────────
    print_header("Test 8: Non-arm joints stay locked after IK")

    set_all_joints(robot, q_indices, TEST_JOINTS)
    solver, effector_task = setup_solver(robot, q_indices)

    for arm_name, arm_cfg in ARMS.items():
        T_current = robot.get_T_world_frame(arm_cfg["link_name"])
        effector_task[arm_name].T_world_frame = T_current

    # Record initial non-arm joint angles
    arm_joint_names = set()
    for arm_cfg in ARMS.values():
        arm_joint_names.update(arm_cfg["joint_names"])
    initial_non_arm = {}
    for jn in robot.joint_names():
        if jn not in arm_joint_names:
            initial_non_arm[jn] = get_joint_angle(robot, q_indices, jn)

    # Move left arm
    T_init = robot.get_T_world_frame("left_arm_link6")
    init_xyz = T_init[:3, 3].copy()
    init_quat = tf.quaternion_from_matrix(T_init)
    target_xyz, target_quat = apply_delta_pose(
        init_xyz, init_quat, np.array([0.04, 0.0, 0.0]), np.zeros(3)
    )
    T_target = tf.quaternion_matrix(target_quat)
    T_target[:3, 3] = target_xyz
    effector_task["left_arm"].T_world_frame = T_target

    for _ in range(100):
        solver.solve(True)
        robot.update_kinematics()

    locked_ok = True
    for jn, init_val in initial_non_arm.items():
        val = get_joint_angle(robot, q_indices, jn)
        drift = abs(val - init_val)
        if drift > 0.05:
            print(f"  [FAIL] {jn} drifted: {init_val:.4f} -> {val:.4f} (delta={drift:.4f})")
            locked_ok = False
    if locked_ok:
        print("  [PASS] All non-arm joints stayed locked (drift < 0.05 rad).")
        n_pass += 1
    else:
        n_fail += 1

    # ── Summary ─────────────────────────────────────────────────────────────
    print_header(f"Summary: {n_pass} passed, {n_fail} failed")
    if n_fail == 0:
        print("  All tests PASSED. Placo IK pipeline is ready for VR teleop.\n")
    else:
        print("  Some tests FAILED. Review results above.\n")


if __name__ == "__main__":
    main()

from dataclasses import dataclass
from xrobotoolkit_teleop.common.xr_client import XrClient
from lerobot.teleoperators.config import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("lerobot_teleoperator_arx")
@dataclass
class ARXVRTeleopConfig(TeleoperatorConfig):
    """
    Configuration for ARX VR Teleoperator with dual R5 arms.

    Note: CAN configuration is NOT needed here because the teleoperator
    uses the robot's ZeroRPC bridge via set_robot_reference() instead of
    creating its own CAN connection.
    """

    # VR client
    xr_client: XrClient = None

    # Control frequency
    fps: int = 20

    # VR control scaling
    scale_factor: float = 0.8

    # Coordinate transformation (ZYX Euler angles in degrees)
    R_headset_world: list = None

    # 双臂安装朝向补偿: 绕 Z 轴的 yaw 角 (度)
    # 两臂相对安装时: 左臂 -90, 右臂 +90; 两臂同向时: 均为 0
    left_arm_yaw_comp_deg: float = 0.0
    right_arm_yaw_comp_deg: float = 0.0

    # Gripper trigger configuration
    trigger_reverse: bool = True
    trigger_threshold: float = 0.5
    close_position: float = -0.2
    open_position: float = 0.2

    # Placo IK config
    robot_urdf_path: str = "assets/r5_urdf/dual_R5a.urdf"
    servo_time: float = 0.017
    visualize_placo: bool = False

    # LIFT chassis toggle (set False when running arms-only without LIFT)
    enable_lift: bool = True

    # Chassis control scaling (only used when enable_lift=True)
    chassis_vx_scale: float = 2.0
    chassis_vy_scale: float = 2.0
    chassis_wz_scale: float = 4.0
    chassis_height_scale: float = 0.02
    chassis_mode: int = 2

    # Control mode
    control_mode: str = "vrteleop"

    # ---------------- Smoothing filters ----------------
    # VR pose-level smoothing (One-Euro on xyz, slerp-EMA on quat).
    # Applied on the raw VR controller pose BEFORE delta computation.
    enable_pose_filter: bool = True
    pose_min_cutoff: float = 1.0   # Hz, lower => more smoothing at low speeds
    pose_beta: float = 0.02        # higher => less lag at high speeds
    pose_d_cutoff: float = 1.0     # Hz, derivative low-pass cutoff
    quat_slerp_alpha: float = 0.5  # in [0, 1]; 1.0 disables quat smoothing

    # Joint-command smoothing (EMA + per-joint velocity clamp).
    # Applied on IK-solved joint angles BEFORE writing to target_*_q.
    enable_joint_filter: bool = True
    joint_ema_alpha: float = 0.6      # in [0, 1]; 1.0 disables EMA
    joint_max_velocity: float = 3.0   # rad/s; <= 0 disables velocity clamp

    def __post_init__(self):
        if self.R_headset_world is None:
            self.R_headset_world = [-90, 0, 90]

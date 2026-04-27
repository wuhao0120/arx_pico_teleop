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

    # Low-pass smoothing for VR delta pose.
    # 1.0 = no smoothing; lower values = smoother but more lag.
    position_filter_alpha: float = 1.0
    rotation_filter_alpha: float = 1.0

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

    def __post_init__(self):
        if self.R_headset_world is None:
            self.R_headset_world = [-90, 0, 90]

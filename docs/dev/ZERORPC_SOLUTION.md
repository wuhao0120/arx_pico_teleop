# ZeroRPC 跨进程通信方案

## 问题背景

ROS2 Jazzy 系统运行在 Python 3.12 环境，而 LeRobot 数据采集需要在 Python 3.10 的 `ur_data` conda 环境中运行。两者无法在同一 Python 解释器中同时运行，因为：

- ROS2.ROS2 的 C 扩展模块编译为 Python 3.12 ABI
- LeRobot 及其依赖需要 Python 3.10
- 直接导入会导致 `ImportError: dynamic module does not match interpreter`

## 整体架构

```
┌─────────────────────────────────────────────────────────────────────────┐
│                     ROS2 Bridge 服务 (Python 3.12)                  │
│                                                                   │
│  ┌─────────────────────────────────────────────────────────────────┐    │
│  │          ZeroRPC Server (0.0.0.0:5555)                      │    │
│  │                                                              │    │
│  │  ┌────────────────┐    ┌──────────────────────────┐        │    │
│  │  │   LiftBridge    │    │   R5DualArmBridge        │        │    │
│  │  └────────────────┘    └──────────────────────────┘        │    │
│  └─────────────────────────────────────────────────────────────────┘    │
│                         │                                       │
│                         │ ZeroRPC (zmq)                        │
│                         ▼                                       │
└─────────────────────────────────────────────────────────────────────────┘
                         │
                         │ (序列化: JSON/MessagePack)
                         ▼
┌─────────────────────────────────────────────────────────────────────────┐
│           LeRobot 数据采集 (ur_data, Python 3.10)                │
│                                                                   │
│  ┌─────────────────────────────────────────────────────────────────┐    │
│  │         ZeroRPC Client (localhost:5555)                      │    │
│  │                                                              │    │
│  │  get_full_state()  ->  远程调用 ->  返回状态               │    │
│  │  set_dual_joints() ->  远程调用 ->  发送命令               │    │
│  └─────────────────────────────────────────────────────────────────┘    │
│                         │                                       │
│                         ▼                                       │
│  ┌─────────────────────────────────────────────────────────────────┐    │
│  │         ARXLiftRobot (LeRobot Robot 基类)                  │    │
│  └─────────────────────────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## 1. 数据类型定义（两边通用）

```python
# common_data_types.py
# 同时被 Python 3.12 和 3.10 导入，使用纯 Python 数据结构

from dataclasses import dataclass, asdict
from typing import List, Dict, Any, Optional
import numpy as np

@dataclass
class ChassisState:
    """底盘状态"""
    height: float          # 米
    head_yaw: float       # 弧度
    head_pitch: float     # 弧度
    waist: float         # 腰部位置

@dataclass
class ArmState:
    """单臂状态"""
    joint_positions: List[float]      # 7 个关节
    joint_velocities: List[float]     # 7 个关节速度
    joint_currents: List[float]      # 7 个关节电流
    end_pose: List[float]           # [x, y, z, roll, pitch, yaw]
    gripper_position: float         # 夹爪位置

@dataclass
class FullRobotState:
    """完整机器人状态"""
    chassis: ChassisState
    left_arm: ArmState
    right_arm: ArmState
    timestamp: float

@dataclass
class ArmCommand:
(    """臂控制命令"""
    joint_positions: List[float]      # 7 维
    velocities: Optional[List[float]] = None
    mode: int = 0

@dataclass
class ChassisCommand:
    """底盘控制命令"""
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0
    height: Optional[float] = None

@dataclass
class RobotCommand:
    """完整机器人命令"""
    left_arm: ArmCommand
    right_arm: ArmCommand
    chassis: ChassisCommand

# 转换辅助函数
def numpy_to_list(obj):
    """递归将 numpy 数组转为列表（ZeroRPC 序列化需要）"""
    if isinstance(obj, np.ndarray):
        return obj.tolist()
    elif isinstance(obj, dict):
        return {k: numpy_to_list(v) for k, v in obj.items()}
    elif isinstance(obj, (list, tuple)):
        return [numpy_to_list(v) for v in obj]
    else:
        return obj

def state_to_dict(state: FullRobotState) -> dict:
    """将状态转为字典（序列化用）"""
    return asdict(state)

def dict_to_state(data: dict) -> FullRobotState:
    """从字典恢复状态"""
    chassis_data = data['chassis']
    chassis = ChassisState(**chassis_data)

    left_data = data['left_arm']
    left_arm = ArmState(**left_data)

    right_data = data['right_arm']
    right_arm = ArmState(**right)

    return FullRobotState(
        chassis=chassis,
        left_arm=left_arm,
        right_arm=right_arm,
        timestamp=data['timestamp']
    )
```

---

## 2. ZeroRPC 服务端 (Python 3.12)

```python
# ros2_bridge_server.py
# 运行在 ROS2 环境 (Python 3.12)

import logging
import time
import threading
import zerorpc
import numpy as np
from arx_lift2_ros2_bridge import ARXLift2Bridge
import rclpy
import sys
sys.path.insert(0, "/path/to/common")
from common_data_types import (
    ChassisState, ArmState, FullRobotState,
    ArmCommand, ChassisCommand, RobotCommand,
    numpy_to_list
)

logger = logging.getLogger(__name__)

class ARXLift2RPCServer:
    """ARX LIFT2 ZeroRPC 服务端"""

    def __init__(self, enable_lift=True, enable_arms=True, host="0.0.0.0", port=5555):
        self.enable_lift = enable_lift
        self.enable_arms = enable_arms
        self.host = host
        self.port = port

        # ROS2 初始化
        if not rclpy.ok():
            rclpy.init()

        # 创建 Bridge
        self.bridge = ARXLift2Bridge(
            lift_node_name="arx_lift_rpc_server",
            arms_node_name="arx_r5_dual_rpc_server",
            enable_lift=enable_lift,
            enable_arms=enable_arms
        )

        self._connected = False
        self._lock = threading.Lock()

    def connect(self, timeout=10.0) -> bool:
        """连接到机器人"""
        try:
            self.bridge.connect(timeout=timeout)
            self._connected = True
            logger.info(f"✓ Robot connected via RPC server")
            return True
        except Exception as e:
            logger.error(f"✗ Failed to connect: {e}")
            return False

    def is_connected(self) -> bool:
        """检查连接状态"""
        return self._connected

    def get_full_state(self) -> dict:
        """获取完整状态（RPC 方法）"""
        if not self._connected:
            raise RuntimeError("Robot not connected")

        with self._lock:
            state = self.bridge.get_full_state()

            # 转换为数据类
            chassis_state = ChassisState(
                height=float(state['chassis']['height']),
                head_yaw=float(state['chassis']['head_yaw']),
                head_pitch=float(state['chassis']['head_pitch']),
                waist=float(state['chassis']['waist'])
            )

            left_arm = ArmState(
                joint_positions=numpy_to_list(state['left_arm']['joint_positions']),
                joint_velocities=numpy_to_list(state['left_arm']['joint_velocities']),
                joint_currents=numpy_to_list(state['left_arm']['joint_currents']),
                end_pose=numpy_to_list(state['left_arm']['end_pose']),
                gripper_position=float(state['left_arm']['gripper'])
            )

            right_arm = ArmState(
                joint_positions=numpy_to_list(state['right_arm']['joint_positions']),
                joint_velocities=numpy_to_list(state['right_arm']['joint_velocities']),
                joint_currents=numpy_to_list(state['right_arm']['joint_currents']),
                end_pose=numpy_to_list(state['right_arm']['end_pose']),
                gripper_position=float(state['right_arm']['gripper'])
            )

            full_state = FullRobotState(
                chassis=chassis_state,
                left_arm=left_arm,
                right_arm=right_arm,
                timestamp=time.time()            )

            # 转换为字典返回（ZeroRPC 序列化）
            from dataclasses import asdict
            return asdict(full_state)

    def send_command(self, command_dict: dict) -> bool:
        """发送命令（RPC 方法）"""
        if not self._connected:
            raise RuntimeError("Robot not connected")

        with self._lock:
            # 从字典恢复命令对象
            chassis_cmd = ChassisCommand(**command_dict['chassis'])
            left_arm_cmd = ArmCommand(**command_dict['left_arm'])
            right_arm_cmd = ArmCommand(**command_dict['right_arm'])

            # 发送双臂命令
            if self.enable_arms:
                self.bridge.set_dual_joint_positions(
                    np.array(left_arm_cmd.joint_positions),
                    np.array(right_arm_cmd.joint_positions),
                    left_velocities=np.array(left_arm_cmd.velocities) if left_arm_cmd.velocities else None,
                    right_velocities=np.array(right_arm_cmd.velocities) if right_arm_cmd.velocities else None,
                    mode=left_arm_cmd.mode
                )

            # 发送底盘命令
            if self.enable_lift:
                if chassis_cmd.vx != 0.0 or chassis_cmd.vy != 0.0 or chassis_cmd.wz != 0.0:
                    self.bridge.set_chassis_velocity(chassis_cmd.vx, chassis_cmd.vy, chassis_cmd.wz)

                if chassis_cmd.height is not None:
                    self.bridge.set_chassis_height(chassis_cmd.height)

            return True

    def emergency_stop(self) -> bool:
        """紧急停止"""
        if self._connected:
            self.bridge.emergency_stop()
        return True

    def disconnect(self):
        """断开连接"""
        if self._connected:
            self.bridge.disconnect()
            self._connected = False

def main():
    logging.basicConfig(level=logging.INFO)

    # 创建 RPC 服务
    server = ARXLift2RPCServer(
        enable_lift=True,
        enable_arms=True,
        host="0.0.0.0",
        port=5555
    )

    # 连接机器人
    if not server.connect():
        logger.error("Failed to connect to robot")
        return

    # 启动 RPC 服务
    logger.info(f"Starting ZeroRPC server on tcp://{server.host}:{server.port}")
    rpc_server = zerorpc.Server(server, heartbeat=60)
    rpc_server.bind(f"tcp://{server.host}:{server.port}")

    try:
        rpc_server.run()
    except KeyboardInterrupt:
        logger.info("Shutting down...")
        server.disconnect()
        rpc_server.close()

if __name__ == "__main__":
    main()
```

---

## 3. ZeroRPC 客户端 (Python 3.10)

```python
# rpc_bridge_client.py
# 运行在 ur_data 环境 (Python 3.10)

import logging
import zerorpc
import numpy as np
from typing import Dict, Any, Optional
sys.path.insert(0, "/path/to/common")
from common_data_types import (
    ChassisState, ArmState, FullRobotState,
    ArmCommand, ChassisCommand, RobotCommand,
    dict_to_state
)

logger = logging.getLogger(__name__)

class ARXLift2RPCClient:
    """ARX LIFT2 ZeroRPC 客户端"""

    def __init__(self, host="localhost", port=5555, timeout=5.0):
        self.host = host
        self.port = port
        self.timeout = timeout
        self._client = None
        self._connected = False

    def connect(self) -> bool:
        """连接到 RPC 服务"""
        try:
            self._client = zerorpc.Client(
                f"tcp://{self.host}:{self.port}",
                timeout=self.timeout
            )
            # 测试连接
            self._client.is_connected()
            self._connected = True
            logger.info(f"✓ Connected to RPC server at {self.host}:{self.port}")
            return True
        except Exception as e:
            logger.error(f"✗ Failed to connect: {e}")
            self._connected = False
            return False

    def is_connected(self) -> bool:
        """检查连接状态"""
        if not self._connected:
            return False
        try:
            return self._client.is_connected()
        except:
            self._connected = False
            return False

    def get_full_state(self) -> FullRobotState:
        """获取完整状态"""
        if not self._connected:
            raise RuntimeError("RPC client not connected")

        state_dict = self._client.get_full_state()
        return dict_to_state(state_dict)

    def send_command(self, left_joints: np.ndarray,
                   right_joints: np.ndarray,
                   left_velocities: Optional[np.ndarray] = None,
                   right_velocities: Optional[np.ndarray] = None,
                   mode: int = 0,
                   chassis_vx: float = 0.0,
                   chassis_vy: float = 0.0,
                   chassis_wz: float = 0.0,
                   chassis_height: Optional[float] = None) -> bool:
        """发送命令"""
        if not self._connected:
            raise RuntimeError("RPC client not connected")

        command = RobotCommand(
            left_arm=ArmCommand(
                joint_positions=left_joints.tolist(),
                velocities=left_velocities.tolist() if left_velocities is not None else None,
                mode=mode
            ),
            right_arm=ArmCommand(
                joint_positions=right_joints.tolist(),
                velocities=right_velocities.tolist() if right_velocities is not None else None,
                mode=mode
            ),
            chassis=ChassisCommand(
                vx=chassis_vx,
                vy=chassis_vy,
                wz=chassis_wz,
                height=chassis_height
            )
        )

        from dataclasses import asdict
        return self._client.send_command(asdict(command))

    def emergency_stop(self) -> bool:
        """紧急停止"""
        if self._connected:
            return self._client.emergency_stop()
        return False

    def disconnect(self):
        """断开连接"""
        if self._client:
            self._client.close()
        self._connected = False
```

---

## 4. LeRobot Robot 类适配 (Python 3.10)

```python
# robots/arx/arx_lift_rpc.py
# 修改后的 ARXLift 类，使用 RPC 客户端替代直接 Bridge 导入

from lerobot.cameras import make_cameras_from_configs
from lerobot.utils.errors import DeviceNotConnectedError, DeviceAlreadyConnectedError
from lerobot.robots.robot import Robot
from .config_arx import ARXConfig
from rpc_bridge_client import ARXLift2RPCClient  # 新增

class ARXLift(Robot):
    """ARX LIFT robot using RPC client for Python 3.10 compatibility"""

    config_class = ARXConfig
    name = "arx_lift"

    def __init__(self, config: ARXConfig, rpc_host="localhost", rpc_port=5555):
        super().__init__(config)
        self.cameras = make_cameras_from_configs(config.cameras)
        self.cfg = config
        self._is_connected = False
        self._num_joints = config.num_joints

        # RPC 客户端替代直接 Bridge
        self.rpc_client = ARXLift2RPCClient(host=rpc_host, port=rpc_port)

        # 夹爪状态
        self._left_gripper_position = config.gripper_open
        self._right_gripper_position = config.gripper_open

    def connect(self) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self.name} is already connected.")

        logger.info("\n===== [ROBOT] Connecting to ARX LIFT2 via RPC =====")

        # 连接 RPC 服务
        if not self.rpc_client.connect():
            raise RuntimeError("Failed to connect to RPC server")

        # 连接相机
        logger.info("\n===== [CAM] Initializing Cameras =====")
        for cam_name, cam in self.cameras.items():
            cam.connect()
            logger.info(f"[CAM] {cam_name} connected")

        self.is_connected = True
        logger.info("===== [ROBOT] Connected Successfully =====\n")

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        """发送动作命令"""
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        if not self.cfg.debug:
            left_pos = np.array([action[f"left_joint_{i+1}.pos"] for i in range(self._num_joints)])
            right_pos = np.array([action[f"right_joint_{i+1}.pos"] for i in range(self._num_joints)])

            if "left_gripper_position" in action:
                self._left_gripper_position = action["left_gripper_position"]
                left_pos[6] = self._left_gripper_position
            if "right_gripper_position" in action:
                self._right_gripper_position = action["right_gripper_position"]
                right_pos[6] = self._right_gripper_position

            self.rpc_client.send_command(
                left_joints=left_pos,
                right_joints=right_pos,
                chassis_vx=action.get("chassis_vx", 0.0),
                chassis_vy=action.get("chassis_vy", 0.0),
                chassis_wz=action.get("chassis_wz", 0.0),
                chassis_height=action.get("chassis_height")
            )

        return action

    def get_observation(self) -> dict[str, Any]:
        """获取观测"""
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        obs_dict = {}

        # 通过 RPC 获取状态
        state = self.rpc_client.get_full_state()

        # 填充观测数据
        for i in range(self._num_joints):
            obs_dict[f"left_joint_{i+1}.pos"] = float(state.left_arm.joint_positions[i])
            obs_dict[f"left_joint_{i+1}.vel"] = float(state.left_arm.joint_velocities[i])
            obs_dict[f"left_joint_{i+1}.cur"] = float(state.left_arm.joint_currents[i])
            obs_dict[f"right_joint_{i+1}.pos"] = float(state.right_arm.joint_positions[i])
            obs_dict[f"right_joint_{i+1}.vel"] = float(state.right_arm.joint_velocities[i])
            obs_dict[f"right_joint_{i+1}.cur"] = float(state.right_arm.joint_currents[i])

        # TCP 姿态
        for i, axis in enumerate(["x", "y", "z", "roll", "pitch", "yaw"]):
            obs_dict[f"left_tcp_pose.{axis}"] = float(state.left_arm.end_pose[i])
            obs_dict[f"right_tcp_pose.{axis}"] = float(state.right_arm.end_pose[i])

        # 夹爪
        obs_dict["left_gripper_position"] = float(state.left_arm.gripper_position)
        obs_dict["right_gripper_position"] = float(state.right_arm.gripper_position)

        # 底盘
        obs_dict["chassis_height"] = float(state.chassis.height)
        obs_dict["chassis_head_yaw"] = float(state.chassis.head_yaw)
        obs_dict["chassis_head_pitch"] = float(state.chassis.head_pitch)

        # 相机
        for camara in self.cameras.items():
            obs_dict[cam_key] = cam.read()

        return obs_dict

    def disconnect(self) -> None:
        """断开连接"""
        if not self.is_connected:
            return

        self.rpc_client.disconnect()
        for cam in self.cameras.values():
            cam.disconnect()

        self.is_connected = False
        logger.info("===== All connections closed =====")

    # ... 其他方法保持不变 (calibrate, is_calibrated, configure, go_home 等) ...
```

---

## 5. 启动脚本

```bash
# start_rpc_server.sh
# 启动 ROS2 RPC 服务端 (Python 3.12 环境)

#!/bin/bash

# Source ROS2 环境
source /opt/ros/jazzy/setup.bash
source /home/arx/ARX_new/install/setup.bash

# 运行 RPC 服务
python3 /path/to/ros2_bridge_server.py
```

```bash
# arx_record_rpc.sh
# 启动数据采集 (Python 3.10 环境)

#!/bin/bash

# 激活 ur_data 环境
conda activate ur_data

# 运行数据采集（使用 RPC 版本的 ARXLift）
python3 -m arx_vr_data_collection.scripts.arx_record --use-rpc
```

---

## 6. 文件结构

```
arx_vr_data_collection/
├── common/
│   └── data_types.py           # 双方通用数据定义
├── ros2_bridge/
│   ├── arx_lift2_ros2_bridge.py
│   └── rpc_server.py           # [新增] RPC 服务端 (Python 3.12)
├── robots/arx/
│    arx_lift.py              # 原有（直接 Bridge）
│   ├── arx_lift_rpc.py       # [新增] RPC 版本 (Python 3.10)
│   ├── config_arx.py
│   └── __init__.py
├── teleoperators/arx/
│   ├── teleop_arx.py
│   └── config_teleop_arx.py
├── scripts/
│   ├── start_rpc_server.sh    # [新增] 启动服务端
│   ├── arx_record_rpc.sh    # [新增] 数据采集脚本
│   └── arx_record.py        # [修改] 支持 RPC 模式
├── rpc_bridge_client.py       # [新增] RPC 客户端
├── requirements.txt
└── docs/
    └── ZERORPC_SOLUTION.md   # 本文档
```

---

## 7. 通信流程

```
1. 启动阶段:
   终端1: bash start_rpc_server.sh    → Python 3.12 ROS2 + RPC Server
   终端2: bash arx_record_rpc.sh     → Python 3.10 + RPC Client

2. 数据采集循环:
   ┌─────────────────┐    get_full_state()    ┌─────────────────┐
   │  LeRobot (3.10) │ ────────────────────→ │  RPC Server     │
   │                 │ ←──────────────────── │  (3.12)        │
   └─────────────────┘    返回状态          └─────────────────┘
           │
           │ send_command(action)
           ▼
   ┌─────────────────┐    转发命令           ┌─────────────────┐
   │  RPC Server     │ ────────────────────→ │  ROS2 Topics    │
   │  (3.12)        │                     │  /joint_control  │
   └─────────────────┘                     └─────────────────┘
           │
           ▼
   ┌─────────────────┐
   │  硬件 (CAN)   │
   └─────────────────┘
```

---

## 8. 依赖安装

### 服务端 (Python 3.12 - ROS2 环境)

```bash
pip install zerorpc
```

### 客户端 (Python 3.10 - ur_data 环境)

```bash
pip install zerorpc
```

---

## 9. 方案优势

| 特性 | 说明 |
|------|------|
| **完全隔离** | ROS2 在 3.12 运行，LeRobot 在 3.10 运行，无版本冲突 |
| **透明通信** | 通过 RPC 实现远程调用，客户端几乎无感知 |
| **类型安全** | 通过 dataclass 定义双方通用数据结构 |
| **低延迟** | ZeroRPC 基于 ZeroMQ，延迟约 1-2ms |
| **跨网络** | 支持远程访问（可通过配置 host 连接远程机器人） |
| **故障恢复** | 支持 heartbeat，自动检测连接状态 |

---

## 10. 实施步骤

1. **创建通用数据类型模块** (`common/data_types.py`)
2. **实现 ZeroRPC 服务端** (`ros2_bridge/rpc_server.py`)
3. **实现 ZeroRPC 客户端** (`rpc_bridge_client.py`)
4. **创建 RPC 版本的 Robot 类** (`robots/arx/arx_lift_rpc.py`)
5. **修改数据采集脚本** (`scripts/arx_record.py` 支持 `--use-rpc` 参数)
6. **创建启动脚本** (`scripts/start_rpc_server.sh`, `scripts/arx_record_rpc.sh`)
7. **测试跨进程通信**
8. **集成到数据采集流程**

---

**文档创建时间**: 2026-02-27
**问题**: ROS2 Python 3.12 与 LeRobot Python 3.10 版本不兼容
**解决**: ZeroRPC 跨进程通信方案

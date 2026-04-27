# Placo IK Integration Summary

## Overview

将 Placo 逆运动学解算器集成到 ARX VR 遥操作器（`teleop_arx.py`），替换原来的空壳 IK 实现。现在 VR 控制器的末端增量位姿能被正确解算为关节角度，通过 ZeroRPC Bridge 发送给真实机器人。

## Commit

```
2e6574a Integrate Placo IK solver into ARX VR teleoperator
Branch: ur_zerorpc
```

---

## 修改的文件

### 1. `teleop_arx.py` — 核心改动

#### 新增 imports
- `import placo` — Placo IK 解算器
- `from pathlib import Path` — URDF 路径解析

#### 删除
- `quat_wxyz_to_rpy()` 函数 — 不再需要 RPY 转换，改用 Placo FK 的 4x4 变换矩阵

#### DEFAULT_MANIPULATOR_CONFIG 增加 `link_name`
```python
"left_arm":  {"link_name": "left_arm_link6",  ...}
"right_arm": {"link_name": "right_arm_link6", ...}
```

#### `__init__` 新增 Placo 状态
```python
self.placo_robot = None   # placo.RobotWrapper
self.solver = None         # placo.KinematicsSolver
self.effector_task = {}    # arm_name -> frame_task
```

#### 6 个新方法

| 方法 | 作用 |
|------|------|
| `_check_placo_setup()` | 加载 lift.urdf，创建 KinematicsSolver，构建 `_q_indices` 映射表 |
| `_check_endeffector_setup()` | 为每只手臂添加 frame task (weight=1.0) + manipulability task (weight=1e-3) |
| `_setup_joints_regularization()` | 用 soft joints task 锁定非手臂关节（wheels, head, catch, lift） |
| `_set_placo_joint(name, angle)` | 用 cos/sin 编码写入 continuous joint 到 state.q |
| `_get_placo_joint(name)` | 用 atan2 从 state.q 读取 continuous joint 角度 |
| `_sync_placo_from_bridge()` | 从 ZeroRPC Bridge 读取真实关节角 → 同步到 Placo 模型 |
| `_read_placo_arm_joints()` | 从 Placo 模型读取 IK 解算后的 6 个关节角度 |

#### `connect()` 改动
在 `_init_qpos()` 之前调用：
```python
self._check_placo_setup()
self._check_endeffector_setup()
self._setup_joints_regularization()
```

#### `_init_qpos()` 改动
- 通过 `_sync_placo_from_bridge()` 同步关节状态
- 用 Placo FK 设置初始 IK 目标（`effector_task[name].T_world_frame`）

#### `_update_from_vr()` 重写
旧流程：直接遍历手臂处理 → 底盘控制
新流程：
1. `_sync_placo_from_bridge()` — 同步真实关节到 Placo
2. 遍历手臂调用 `_process_arm_vr()` — 设置 IK 目标
3. `solver.solve(True)` + `update_kinematics()` — 一次解算双臂
4. `_read_placo_arm_joints()` — 读取解算结果写入 `target_left/right_q[:6]`
5. `_process_chassis_vr()` — 底盘控制（不变）

#### `_process_arm_vr()` 重写
旧逻辑（Grip 按下时）：
- 从 Bridge 读 EE pose (RPY 格式) → 转 quaternion → 存储
- IK 部分只有 `pass`（空壳）

新逻辑（Grip 按下时）：
- 从 **Placo FK** 读 EE pose (`get_T_world_frame` → 4x4 矩阵)
- 计算 VR 增量 (`_process_xr_pose` 不变)
- 用 `apply_delta_pose()` 得到目标 EE pose
- 设置 `effector_task[arm_name].T_world_frame`（IK 目标）

旧逻辑（Grip 松开时）：
- 从 Bridge 读当前关节角覆盖 target_q

新逻辑（Grip 松开时）：
- 重置 VR tracking 状态
- 设置 Placo 当前 EE pose 为 IK hold target
- 仍从 Bridge 读 target_q（为了保持 gripper index 6 的值）

### 2. `config_teleop_arx.py` — 新增 3 个配置项

```python
# Placo IK config
robot_urdf_path: str = "assets/lift_urdf/robot_description/lift.urdf"
servo_time: float = 0.017    # IK solver dt
visualize_placo: bool = False
```

### 3. `scripts/config/cfg_arx.yaml` — 新增 placo 配置段

```yaml
teleop:
  placo:
    robot_urdf_path: "assets/lift_urdf/robot_description/lift.urdf"
    servo_time: 0.017
    visualize_placo: false
```

---

## 新增的文件

### 4. `assets/lift_urdf/` — URDF + Meshes

```
assets/lift_urdf/
├── robot_description/
│   └── lift.urdf              # 双臂完整 URDF (22 joints)
└── meshes/
    └── lift/
        └── *.stl              # 17 个 mesh 文件
```

URDF 内 mesh 路径为 `../meshes/lift/xxx.stl`，从 `robot_description/lift.urdf` 出发能正确解析到 `meshes/lift/`。

### 5. `test_placo_ik.py` — 离线 IK 测试脚本

不需要连接硬件或 VR，纯 Placo 验证。16 个测试全部通过：

| 测试 | 内容 | 结果 |
|------|------|------|
| Test 1 | Joint set/get roundtrip (cos/sin 编码) | PASS |
| Test 2 | FK consistency (det(R) = 1.0) | PASS x2 |
| Test 3 | Hold position (100 steps) | PASS (0.13mm) |
| Test 4 | Position delta X/Y/Z/XYZ | PASS x4 (0.1~0.15mm) |
| Test 5 | Orientation delta Roll/Pitch/Yaw | PASS x3 |
| Test 6 | Dual-arm simultaneous delta | PASS x2 |
| Test 7 | Incremental stepping (20 steps, 模拟 VR 循环) | PASS (1.3mm) |
| Test 8 | Non-arm joints locked | PASS |

运行方式：
```bash
cd lerobot_data_collection/arx_vr_data_collection
conda run -n ur_data python test_placo_ik.py
```

---

## 关键技术发现与解决方案

### 1. Continuous Joint 的 cos/sin 编码问题

**问题**: lift.urdf 中手臂关节类型为 `continuous`（非 `revolute`）。Placo 对 continuous joints 在 `state.q` 中使用 `[cos(θ), sin(θ)]` 双参数表示，导致：
- `state.q` 长度 = 46（而非 22 个关节）
- `set_joint(name, value)` 把 `value` 直接写入 **cos 槽**，不做 cos/sin 转换
- `get_joint(name)` 返回 cos 槽的原始值（即 `cos(θ)`），不是角度 `θ`

**后果**: 直接用 `set_joint('left_arm_joint1', 0.3)` 会让 cos 槽 = 0.3（而非 cos(0.3)=0.955），导致旋转矩阵 scale 错误（det(R) ≠ 1.0），FK 输出完全错误。

**解决方案**: 自定义 `_set_placo_joint` / `_get_placo_joint` 方法：
```python
def _set_placo_joint(self, joint_name, angle):
    idx = self._q_indices[joint_name]
    state.q[idx]     = cos(angle)
    state.q[idx + 1] = sin(angle)

def _get_placo_joint(self, joint_name):
    idx = self._q_indices[joint_name]
    return atan2(state.q[idx + 1], state.q[idx])
```

`_q_indices` 在 `_check_placo_setup()` 中通过 probe 每个关节构建。

### 2. solver.solve(True) 后必须 update_kinematics()

**问题**: `solver.solve(True)` 更新了 `state.q`，但 `get_T_world_frame()` 依赖 FK 缓存。不调用 `update_kinematics()` 会返回旧值。

**解决方案**: 在 `_update_from_vr()` 中 solve 后立即调用：
```python
self.solver.solve(True)
self.placo_robot.update_kinematics()  # 必须！
```

### 3. joints_task 正则化目标值也受 cos/sin 影响

**问题**: `set_joints({'joint_wheel1': 0.0})` 对 continuous joint 会让 cos 槽 = 0（即角度 = ±π/2），而非锁定在角度 0。

**解决方案**: 使用零位构型的 `state.q` 值作为目标：
- Continuous joints: `q0[idx] = 1.0`（cos(0) = 1.0）
- Prismatic joints: `q0[idx] = 0.0`

### 4. Manipulability weight 需要针对 R5 手臂调参

**问题**: UR5e 参考实现用 `5e-2`，但 R5 手臂几何不同，导致 manipulability task 与 frame task 竞争，产生 ~7mm 稳态位置误差。

**解决方案**: 降低 manipulability weight 到 `1e-3`，稳态误差降至 < 0.15mm。

---

## lift.urdf 关节结构

```
[ 0] joint_lift          (prismatic)   ← 升降
[ 1] left_arm_joint1     (continuous)  ← 左臂 6 关节
[ 2] left_arm_joint2     (continuous)
[ 3] left_arm_joint3     (continuous)
[ 4] left_arm_joint4     (continuous)
[ 5] left_arm_joint5     (continuous)
[ 6] left_arm_joint6     (continuous)
[ 7] left_catch_joint1   (prismatic)   ← 左夹爪
[ 8] left_catch_joint2   (prismatic)
[ 9] right_arm_joint1    (continuous)  ← 右臂 6 关节
[10] right_arm_joint2    (continuous)
[11] right_arm_joint3    (continuous)
[12] right_arm_joint4    (continuous)
[13] right_arm_joint5    (continuous)
[14] right_arm_joint6    (continuous)
[15] right_catch_joint1  (prismatic)   ← 右夹爪
[16] right_catch_joint2  (prismatic)
[17] joint_h_1           (continuous)  ← head
[18] joint_h_2           (continuous)
[19] joint_wheel1        (continuous)  ← wheels
[20] joint_wheel2        (continuous)
[21] joint_wheel3        (continuous)
```

**EE link names**: `left_arm_link6`, `right_arm_link6`
**state.q 长度**: 46（因 continuous joints 用 cos/sin 双参数）

---

## 后续计划

1. **TCP Offset**: 在 URDF 中添加 fixed joint + TCP link（link6 X 轴正方向 ~7.5cm），使采集的 ee_pose 为夹爪末端位姿（而非法兰盘位姿），方便 VLA 模型泛化
2. **连接真机测试**: 连接 VR + 真实 LIFT 平台验证完整闭环
3. **数据采集验证**: 运行 record 流程确认 action 数据格式正确

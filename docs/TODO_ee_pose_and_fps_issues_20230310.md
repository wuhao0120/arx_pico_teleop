# TODO: EE Pose Action 记录 & 帧率问题

日期: 2026-03-10

---

## 1. 已完成: Action 中添加 ee_pose 记录

### 改动内容

在数据采集的 action 中新增了 12 个 ee_pose 字段（左右各 6 个: x, y, z, roll, pitch, yaw），
使训练模型时可以选择关节空间或笛卡尔空间作为 action space。

**改动文件：**

- `robots/arx/arx_lift.py` — `action_features` 属性中添加了 `left_tcp_pose.*` 和 `right_tcp_pose.*`
- `teleoperators/arx/teleop_arx.py` — `get_action()` 中从 Placo IK 目标变换矩阵提取 ee_pose

**技术细节：**

- 记录的是 **命令目标** ee_pose（Placo IK target `effector_task[arm_name].T_world_frame`），不是观测到的 ee_pose
- Euler 角采用 extrinsic XYZ（`R.from_matrix(...).as_euler('xyz')`），与 ARX SDK 的 `end_pos` 约定一致
- key 命名与 observation 侧完全一致（`left_tcp_pose.x` 等），LeRobot 框架通过 prefix 自动区分 `observation.*` 和 `action.*`
- `send_action()` 不受影响 — 它只读取 joint/gripper/chassis key，多余的 ee_pose key 被忽略
- Action shape 从 (20,) 变为 (32,)

### 验证方法

1. `bash start_recording.sh debug` 启动
2. 检查 `info.json` 中 action feature 的 shape 和 names
3. 录一个 episode 后确认 ee_pose 值非零且与 observation 中的 tcp_pose 在同一量级
4. 确认 record 模式下机器人行为不变

---

## 2. 发现的问题: 视频播放加速

### 现象

录制的 MP4 视频用播放器打开时明显加速（约 2 倍速）。

### 根本原因

录制循环的**实际帧率低于配置的 30fps**，但视频固定按 30fps 编码。

详细分析：

1. `record_loop`（lerobot_record.py）每帧需要执行：
   - 2 个 RealSense 相机读取
   - ZeroRPC 通信（get_observation → ROS2 Bridge）
   - Placo IK 求解
   - 异步写 PNG
   - 实际每帧耗时可能 50-100ms（~10-20fps）

2. `busy_wait(1/fps - dt_s)` 发现时间已超 33ms 时直接跳过，不等待

3. `add_frame()` 的 timestamp 是合成值 `frame_index / fps`（不是真实挂钟时间）：
   ```python
   # lerobot_dataset.py:1066
   timestamp = frame.pop("timestamp") if "timestamp" in frame else frame_index / self.fps
   ```

4. `encode_video_frames()` 固定按 `fps=30` 编码 MP4

5. 结果：实际 22 秒录制的 338 帧，被压缩到 11.2 秒的视频中

### 数据验证

```
Episode 0: 338 frames, video duration = 11.27s (实际录制可能 ~22s)
Episode 1: 423 frames, video duration = 14.10s (实际录制可能 ~28s)
配置: episode_time_sec = 60, fps = 30
```

### 解决方案（待实施）

**方案 A: 降低目标帧率**（最简单）
- 在 `cfg_arx.yaml` 中将 `fps` 从 30 降到实测可达帧率（如 15 或 20）
- 需要先测量实际帧率

**方案 B: 添加帧率监控**
- 在 record_loop 中加入实时帧率打印/告警
- 当实际帧率 < 目标帧率的 90% 时输出警告

**方案 C: 使用真实时间戳**（最正确但改动大）
- 在 add_frame 时传入真实挂钟 timestamp
- 需要修改 LeRobot 框架或在上层传入 timestamp

---

## 3. 待完成: EE Pose 直接控制模式

### 目标

添加直接发 ee_pose 给 ROS2 控制器的模式，使 replay/inference 时可以直接用笛卡尔空间 action。

### 计划（另开分支）

1. C++ 控制器侧：确认 ROS2 topic 是否已支持接收 ee_pose 命令
2. Bridge/RPC 侧：添加 `set_ee_pose(left_pose, right_pose)` RPC 方法
3. `arx_lift.py` 的 `send_action()` 中：根据 action 中是否包含 ee_pose key 来选择发送关节角还是笛卡尔坐标
4. 或者添加一个配置项 `action_space: "joint" | "cartesian"` 来显式选择

### 依赖

- 需要先确认 ARX R5 C++ 控制器的笛卡尔控制接口
- 当前改动（action 中记录 ee_pose）是前置条件，已完成

# ARX VR遥操作LeRobot数据采集操作手册

**版本**: v1.2
**日期**: 2026-03-12
**适用平台**: ARX R5双臂 + LIFT移动底盘
**支持VR设备**: Meta Quest3 / PICO 4 Ultra

---

## 1. 概述

本文档描述了使用VR设备遥操作ARX机器人，同时自动录制LeRobot格式演示数据集的完整操作流程。该流程支持：

- 双臂7自由度实时遥操作（6关节+夹爪）
- LIFT底盘全向移动+升降控制
- 2路RealSense相机图像同步采集（左手腕、右手腕）
- 自动保存为标准LeRobot数据集格式，可直接用于模仿学习训练

---

## 2. 前置准备

### 2.1 硬件检查清单

| 检查项 | 说明 |
|--------|------|
| [ ] R5双臂已通电 | 电源指示灯常亮 |
| [ ] LIFT底盘已通电 | 底盘电源开关打开 |
| [ ] CAN线缆连接正常 | 双臂+底盘共3路CAN线（can1/can3/can5） |
| [ ] RealSense相机已连接 | 左手腕、右手腕各1台，USB 3.0接口 |
| [ ] VR头显与电脑在同一5G WiFi | 延迟<70ms，无丢包 |
| [ ] 机器人周围有≥1m安全空间 | 无障碍物 |

### 2.2 软件环境

已提前配置好运行环境：

- 系统Python 3.12：ROS 2 Jazzy + 机器人控制器
- Conda环境 `ur_data`（Python 3.10）：LeRobot 0.3.4、XRoboToolkit、Placo、pyzmq
- 工作目录：`/home/arx/ARX_new/lerobot_data_collection/arx_vr_data_collection`

---

## 3. 详细操作流程

### 步骤 0：清理失败的历史数据（如有）

若上次录制异常中断，先删除不完整的数据集：

```bash
# 查看现有数据集
ls ~/.cache/huggingface/lerobot/deepcybo/

# 删除损坏的数据集（按需替换目录名）
rm -rf ~/.cache/huggingface/lerobot/deepcybo/arx_lift_task_20260311_v54
```

> **如何判断数据集是否损坏**：用 `python3 -c "import pyarrow.parquet as pq; pq.read_table('~/.cache/.../data/chunk-000/file-000.parquet')"` 读取，若报 `Parquet magic bytes not found in footer` 即为损坏。

---

### 步骤 1：启动 XRoboToolkit PC Service（PC端必须先做）

VR眼镜需要连接到PC端的 XRoboToolkit 服务进程，**必须在启动录制脚本之前启动**。

```bash
cd /home/arx/ARX_new/lerobot_data_collection/XRoboToolkit-Teleop-Sample-Python/dependencies/XRoboToolkit-PC-Service-Pybind/tmp/XRoboToolkit-PC-Service/RoboticsService/Package/debPack/
bash runService.sh
```

服务在后台运行 `RoboticsServiceProcess`，监听VR设备连接请求。

查看PC的IP地址（供VR眼镜手动连接）：

```bash
ip addr show | grep "inet " | grep -v 127.0.0.1
# 示例输出：inet 192.168.1.100/24 ...
```

---

### 步骤 2：操作VR眼镜连接PC

> PC和VR眼镜必须连接**同一个5G WiFi网络**。

#### PICO 4 Ultra

1. 开机进入主界面
2. 打开 **「XRoboToolkit 遥助手」** 应用
3. 允许所有权限申请（空间定位、存储等），确保6DoF定位功能正常
4. 在连接界面：
   - 自动发现：直接点击PC设备连接
   - 未发现：点击「手动连接」，输入步骤1查到的PC IP地址
5. 连接成功后**手柄指示灯变绿**

#### Meta Quest 3

1. 开机进入主界面
2. 打开 **「XRoboToolkit Teleop」** 应用
3. 首次启动时允许「空间定位」「存储」所有权限
4. 自动搜索或手动输入PC IP连接
5. 界面显示「✅ 已连接到PC」，保持应用在前台

#### 验证VR连接：

```bash
cd /home/arx/ARX_new/lerobot_data_collection/arx_vr_data_collection
conda run -n ur_data python3 scripts/tools/test_xr_connection.py
```

预期输出：连接状态已连接、头显姿态更新正常、网络延迟 < 70ms。

---

### 步骤 3：配置CAN接口（仅重启后首次运行需要）

```bash
# 方式1：系统服务（推荐）
sudo systemctl start arx-can-setup

# 方式2：手动配置
sudo /home/arx/ARX_new/src/drivers/can/scripts/arx_can1.sh  # 左臂
sudo /home/arx/ARX_new/src/drivers/can/scripts/arx_can3.sh  # 右臂
sudo /home/arx/ARX_new/src/drivers/can/scripts/arx_can5.sh  # 底盘
```

验证CAN状态（预期 `state UP`）：

```bash
ip link show can1 can3 can5
```

---

### 步骤 4：检查RealSense相机序列号

```bash
rs-enumerate-devices | grep -E "Serial Number|Camera Info"
```

记录输出的序列号，确认与配置文件中的一致。若不一致，更新配置文件（步骤5）。

异常排查：

- 相机未识别：重新插拔USB线，确保使用USB 3.0接口
- 权限不足：`sudo usermod -aG video $USER` 后重启

---

### 步骤 5：修改配置文件

```bash
cd /home/arx/ARX_new/lerobot_data_collection/arx_vr_data_collection
nano scripts/config/cfg_arx.yaml
```

关键字段说明：

```yaml
record:
  repo_id: "deepcybo/arx_lift_task"   # 数据集名称（用户名/任务名）
  task:
    num_episodes: 10                      # 需要录制的集数
    description: "任务描述（英文）"        # 例：Pick up the red cup
    episode_time_sec: 30                  # 每集最长录制时间（秒）

  cameras:
    left_wrist_cam_serial: "12345678"     # 替换为实际序列号
    right_wrist_cam_serial: "87654321"    # 替换为实际序列号

  robot:
    left_init_joints:  [0.0, -1.57, 1.57, -1.57, -1.57, 0.0, 0.0]  # 7个值
    right_init_joints: [0.0, -1.57, 1.57, -1.57, -1.57, 0.0, 0.0]  # 7个值
    init_height: 0.6                      # 底盘初始高度（米）

  debug: true    # 首次测试设为 true，正式录制改为 false
```

> 初始关节位置获取方法：
> ```bash
> conda run -n ur_data python3 scripts/tools/get_current_joints.py
> ```

---

### 步骤 6：Debug模式测试（首次运行必须执行）

配置文件中 `debug: true`，然后启动：

```bash
bash start_recording.sh debug
```

脚本自动弹出两个终端窗口，机器人**不会实际运动**。

用VR控制器测试以下功能：

| 操作 | 预期结果 |
|------|---------|
| 按住右手Grip | 终端打印右臂目标关节位置实时变化 |
| 按住左手Grip | 终端打印左臂目标关节位置实时变化 |
| 按扳机键 | 终端打印夹爪状态切换（开/关） |
| 推左摇杆 | 终端打印底盘速度指令 |
| 按A键 | 终端显示"保存episode"提示 |

测试全部正常后，将配置文件改为 `debug: false`。

---

### 步骤 7：启动正式录制

```bash
cd /home/arx/ARX_new/lerobot_data_collection/arx_vr_data_collection
bash start_recording.sh record
```

脚本自动在**两个独立终端**中启动所有组件：

#### 终端1（系统Python 3.12）- 机器人控制层

1. 加载 ROS 2 Jazzy 环境
2. 启动 LIFT 控制器：`ros2 run arx_lift_controller lift_controller`
3. 启动双臂控制器：`ros2 launch arx_r5_controller open_double_arm.launch.py`
4. 启动 RPC 服务端：`python3 ros2_bridge/arx_ros2_rpc_server.py`（监听4242端口）
5. 日志：`.log/controllers_YYYYMMDD_HHMMSS.log`

#### 终端2（ur_data conda Python 3.10）- 数据采集层

1. 验证所有依赖（lerobot、xrobotoolkit、pyzmq）
2. 启动数据采集主程序：`python3 scripts/core/run_record_arx.py`
3. 日志：`.log/data_collection_YYYYMMDD_HHMMSS.log`

> **不要手动启动LIFT和双臂控制器**，脚本会自动处理并检测重复启动。

---

### 步骤 8：VR录制操作

程序启动后，机器人自动移动到初始位置后进入就绪状态，按以下流程操作：

```
程序就绪（机器人在初始位置）
         ↓
【录制当前集】
  按住 Grip键 → 控制手臂跟随VR控制器运动
  按 扳机键   → 切换夹爪开合
  任务完成后  → 按右手 A键 → 保存当前episode
         ↓
机器人自动回到初始位置
         ↓
按右手 B键 → 确认，开始下一集录制
         ↓
（重复以上流程...）
         ↓
所有集录制完成 → 按左手 B键 → 停止录制，保存完整数据集
```

#### VR按键速查表

| 按键 | 右手控制器 | 左手控制器 |
|------|-----------|-----------|
| **Grip（按住）** | 激活右臂跟随VR移动 | 激活左臂跟随VR移动 |
| **扳机键** | 切换右夹爪开/关 | 切换左夹爪开/关 |
| **摇杆** | — | 控制底盘：前后=前进/后退，左右=平移，旋转=升降 |
| **A键** | 保存当前episode | 废弃当前episode，重新录制 |
| **B键** | 确认回位/开始下一集 | 停止所有录制并退出 |
| **Menu键** | — | **紧急停止**，立刻切断机器人动力 |

> 操作技巧：随时**松开Grip键**即可暂停手臂跟随，确保安全。

---

## 4. 录制完成后操作

### 4.1 验证数据完整性

```bash
conda activate ur_data
python3 -c "
import pyarrow.parquet as pq, glob, os
files = glob.glob(os.path.expanduser('~/.cache/huggingface/lerobot/deepcybo/arx_lift_task_*/data/chunk-000/file-000.parquet'))
for f in files:
    try:
        t = pq.read_table(f)
        print('OK  行数:', t.num_rows, '|', f)
    except Exception as e:
        print('ERR:', e, '|', f)
"
```

预期输出：`OK  行数: XXXX`，行数应与 `fps × 总录制秒数` 大致吻合。

### 4.2 查看数据集信息

```bash
cat ~/.cache/huggingface/lerobot/deepcybo/arx_lift_task_*/meta/info.json | python3 -m json.tool | grep -E "total_episodes|total_frames|fps"
```

### 4.3 数据集备份

```bash
tar -zcvf dataset_backup_$(date +%Y%m%d).tar.gz \
    ~/.cache/huggingface/lerobot/deepcybo/arx_lift_task_*
```

建议备份到外接存储或NAS，避免数据丢失。

---

## 5. 常见问题排查

| 问题现象 | 排查方向 | 解决方法 |
|----------|----------|----------|
| VR连接失败 | 网络/XRoboToolkit服务 | 1. 确认同一5G WiFi<br>2. 确认PC端 `runService.sh` 已启动<br>3. 重启VR端应用 |
| VR控制器无响应 | XRoboToolkit进程 | 重启 `runService.sh`，重启VR端应用 |
| 机器人不动 | CAN/debug配置 | 1. `ip link show can1 can3 can5` 确认状态UP<br>2. 配置文件 `debug: false`<br>3. 重启机器人电源 |
| 相机图像不显示 | USB连接/序列号 | 1. 重新插拔USB线（用USB 3.0口）<br>2. `rs-enumerate-devices` 确认序列号与配置一致 |
| 端口4242被占用 | 旧RPC进程未退出 | `lsof -i :4242` 找到PID后 `kill <PID>` |
| 录制结束数据损坏 | 程序异常中断导致parquet未写完 | 已修复（v1.2）：Ctrl+C或崩溃时会自动finalize |
| 录制卡顿 | 磁盘/CPU | 1. 确认磁盘剩余≥50G<br>2. 关闭不必要后台进程 |

### 查看日志

```bash
# 控制器日志（含LIFT/双臂启动状态）
tail -f /home/arx/ARX_new/lerobot_data_collection/arx_vr_data_collection/.log/controllers_*.log

# 数据采集日志
tail -f /home/arx/ARX_new/lerobot_data_collection/arx_vr_data_collection/.log/data_collection_*.log
```

---

## 6. 应急处理预案

| 紧急情况 | 处理步骤 |
|----------|----------|
| 机器人失控/碰撞 | 1. 立刻按**左手Menu键**紧急停止<br>2. 切断机器人电源<br>3. 检查硬件损坏情况 |
| VR控制无响应 | 1. 立刻**松开Grip键**，手臂停止跟随<br>2. 若机器人仍在运动，按Menu键<br>3. 检查VR网络连接 |
| 相机突然断开 | 1. 按右手A键停止当前episode（废弃）<br>2. 重新插拔相机USB线<br>3. 重启录制程序 |
| 程序异常崩溃 | 1. 检查 `.log/` 下最新日志<br>2. 确认RPC端口4242已释放<br>3. 确认机器人回到安全位置后重启 |

---

## 7. 安全注意事项

**强制安全规则**：

1. 首次运行务必先设置 `debug: true`，验证VR控制逻辑正确后再改为 `false`
2. 机器人运动时，操作人员必须站在**安全区域（≥1m）**，随时准备按紧急停止
3. 初始关节位置必须经过验证，确保机器人启动时不会碰撞周围物体
4. 录制过程中如出现异常，立即**松开Grip键**或按**左手Menu键**停止
5. 禁止在机器人运动时触碰机械臂和线缆

---

## 附录：数据集结构说明

数据集保存路径：
```
~/.cache/huggingface/lerobot/<user_name>/<task_name>_<YYYYMMDD>_v<N>/
├── data/chunk-000/file-000.parquet    # 关节状态、动作、时间戳
├── videos/
│   ├── observation.images.left_wrist_image/chunk-000/file-000.mp4
│   └── observation.images.right_wrist_image/chunk-000/file-000.mp4
└── meta/
    ├── info.json                      # 数据集元信息
    ├── stats.json                     # 统计信息
    └── episodes/chunk-000/file-000.parquet
```

版本号自动递增，不会覆盖历史数据。

数据字段：

- `action`（32维）：左右臂各7关节位置 + 左右TCP位姿各6维 + 左右夹爪 + 底盘4维
- `observation.state`（59维）：左右臂各7关节位置/速度/电流 + TCP位姿 + 夹爪 + 底盘状态
- `observation.images.*`：相机图像（视频格式，av1编码）

# VR 遥操作延迟优化 — 进展记录

日期: 2026-03-11

状态: 已完成第二轮复盘（`pyzmq + msgpack + ROS2 锁/快照优化` 后）。当前已确认 bridge 读路径显著改善，但卡顿仍在，主因已转移到上层控制架构、同步 RPC 热路径和线程/队列设计。

---

## 1. 当前问题范围

- 当前优先排查范围: 双臂 + 升降柱。
- 底盘平移/旋转暂时不作为主目标。
- 但当前控制链路里，`set_full_command()` 仍然会附带发送底盘零速度和高度命令，因此 lift/chassis 相关 publish 仍然在每帧参与时延。

---

## 2. 当前系统架构

```text
Terminal 1: Python 3.12 + ROS 2 Jazzy
┌───────────────────────────────────────────────┐
│  LIFT controller + R5 双臂 controller (ROS2)  │
│       ↕ ROS2 topics                           │
│  ARXLift2Bridge (ROS2 node, 缓存状态)         │
│       ↕                                       │
│  ArxROS2RPCServer (pyzmq REP + msgpack, 4242) │
└───────────────────────────────────────────────┘
          ↕ pyzmq REQ/REP over TCP (localhost)
┌───────────────────────────────────────────────┐
│  Terminal 2: Python 3.10 + conda ur_data      │
│                                               │
│  run_record_arx.py                            │
│    ├─ ARXLift (robot)                         │
│    │    └─ ArxROS2RPCClient (pyzmq REQ)       │
│    ├─ ARXVRTeleop (teleop)                    │
│    │    └─ VR 线程 (15Hz): XrClient + IK     │
│    └─ record_loop (15Hz 主循环)               │
│         ├─ robot.get_observation()            │
│         │   → bridge.get_full_state() [RPC]   │
│         │   → 并行读相机                       │
│         ├─ teleop.get_action()                │
│         │   → 读 VR 线程共享变量              │
│         │   → 读 robot._last_state 缓存       │
│         ├─ robot.send_action()                │
│         │   → bridge.set_full_command() [RPC] │
│         └─ dataset.add_frame()                │
└───────────────────────────────────────────────┘
```

两个 Python 环境（3.12 vs 3.10）当前通过 `pyzmq + msgpack` 跨进程通信；此前的 ZeroRPC 方案仅保留作历史背景。

---

## 3. 已完成的优化

### Step 1: 消除重复 RPC 调用
- **commit**: `ba7495e`
- `get_observation()` 缓存 `get_full_state()` 结果到 `robot._last_state`
- `get_action()` 优先使用缓存，不再重复调用 RPC
- fps 从 30 降到 15
- 结论: 主线程每帧从原来的“多次读状态 RPC”收敛为“1 次读状态 RPC + 1 次写命令 RPC”

### Step 2: 合并 RPC 发送命令
- **commit**: `4b683d6`
- bridge/server/client 三层新增 `set_full_command()`
- `send_action()` 从多次 RPC 合并为 1 次 RPC

### Step 3: 相机并行读取
- **commit**: `b8ad30b`
- `get_observation()` 中用 `ThreadPoolExecutor` 并行读取相机
- 结论: 相机耗时下降，但不是主瓶颈

### Step 4: VR 线程直接发 RPC（已回退）
- **commit**: `f90f490` → `3007497`
- 在 VR 线程中创建独立 `ArxROS2RPCClient`，IK 解算后直接发 `set_full_command`
- 结果: 灾难性。两个 client 在同一进程不同线程中使用 ZeroRPC/gevent，产生明显冲突
- VR 线程 RPC 平均 45-91ms，最高 818ms，实际帧率掉到 8-12Hz
- 结论: 不能在普通 Python 线程里直接调用 ZeroRPC

### 回退后有效架构
- VR 线程只做纯计算: `XrClient` 读取 + Placo IK
- 所有 RPC 统一回到主线程处理

### Step 5: 切换到 pyzmq + msgpack，并压缩 bridge 热路径
- **日期**: 2026-03-11
- server/client 从 ZeroRPC 切换到 `pyzmq REQ/REP + msgpack`
- server 写命令改为 worker 线程异步投递，避免 ROS2 publish 直接阻塞 REP 主循环
- bridge 新增 `get_full_state_serialized()`、双臂 `set_dual_joint_positions_fast()`、lift/arms snapshot fast path
- 结论: server 侧 `get_full_state` 已降到 `0.01/0.03ms`，`set_full_command` 平均 `1-2ms` 级；transport/gevent 已不再是当前第一瓶颈

---

## 4. 现有性能数据

> 注: 第 4-9 节主要记录 ZeroRPC 时代的第一轮排查和当时的判断。2026-03-11 已切换到 `pyzmq + msgpack`，因此这些结论只保留作对照；最新判断见第 10-12 节。

### 主循环测试结果（2026-03-11 13:58）

目标 15Hz，实际约 7-8Hz:

```text
[PERF main_loop] 100 frames | get_state=62.7/400.6ms | cameras=3.1/43.0ms | send_cmd=57.1/394.4ms
[PERF main_loop] 200 frames | get_state=48.5/457.6ms | cameras=8.0/52.4ms | send_cmd=48.7/318.3ms
[PERF main_loop] 300 frames | get_state=70.5/807.4ms | cameras=3.2/32.6ms | send_cmd=59.4/359.5ms
[PERF main_loop] 400 frames | get_state=69.2/376.9ms | cameras=3.2/46.0ms | send_cmd=59.0/527.8ms
```

### VR 线程测试结果

目标 15Hz，实际 15.0Hz:

```text
[PERF vr_thread] 200 frames | actual 15.0Hz | vr_input=0.1/0.8ms | ik_solve=0.4/0.7ms
[PERF vr_thread] 400 frames | actual 15.0Hz | vr_input=0.3/0.7ms | ik_solve=0.3/0.5ms
```

### 已确认结论

| 组件 | 平均耗时 | 最大耗时 | 结论 |
|------|---------|---------|------|
| `get_full_state` 端到端 RPC | 50-70ms | 807ms | 主瓶颈之一 |
| `set_full_command` 端到端 RPC | 50-60ms | 527ms | 主瓶颈之一 |
| 相机并行读取 | 3-8ms | 52ms | 次要 |
| VR 输入处理 | 0.1-0.4ms | 0.8ms | 不是瓶颈 |
| Placo IK | 0.3-0.4ms | 2.0ms | 不是瓶颈 |

**IK 不是问题。主线程的同步读写链路才是问题。**

---

## 5. 新发现（修正此前错误判断）

### 5.1 ZeroRPC 本身不是“固有 50-70ms”

对 localhost 做了独立 ZeroRPC loopback benchmark，服务端只返回小 payload，不接 ROS2/硬件：

```text
ping: avg=0.449ms p95=0.678ms p99=0.786ms max=1.040ms
get_full_state: avg=0.445ms p95=0.606ms p99=0.658ms max=0.687ms
set_full_command: avg=0.467ms p95=0.645ms p99=0.690ms max=0.754ms
msgpack(pack+unpack): avg=0.004ms p95=0.005ms max=0.019ms size=742B
```

修正后的结论:

- “ZeroRPC 固有基础开销就是 50-70ms” 这个判断是错误的。
- `msgpack` 序列化/反序列化也不是主要来源。
- 如果真实系统里仍看到 50-70ms，那么大头不在 wire transport，而在 **server 进入 bridge 之后的执行路径**。

### 5.2 heartbeat/timeout 不是当前主因

- 仓库检查时，client 代码仍显示 `zerorpc.Client(heartbeat=20, timeout=5)`，server 仍显示 `heartbeat=30`。
- 即使运行时你已经把 client heartbeat/timeout 去掉，这也不足以解释 50-70ms，因为上面的 loopback 基准已经说明 ZeroRPC 本身不是这个量级。
- 因此，“去掉 heartbeat/timeout 后就会恢复流畅” 不是当前最有力的解释。

### 5.3 真正更可疑的是 server/bridge/ROS2/controller 路径

ARX 当前链路比 Franka 厚得多：

- Franka server 暴露的方法基本是薄封装，直接转给机器人控制接口。
- ARX server 进入 `ARXLift2Bridge` 后，读状态会拼装 lift + 左臂 + 右臂多个快照；写命令会继续拆成双臂 publish、底盘速度 publish、高度 publish。
- 因此，Franka 即使跨两台主机还能快，并不能说明网络比本机通信更优，只能说明 **Franka 的 server/control path 更薄**。

### 5.4 主循环架构本身会放大卡顿

当前录制主循环是同步串行的：

1. `robot.get_observation()`
2. `teleop.get_action()`
3. `robot.send_action()`
4. `dataset.add_frame()`

优化后虽然只剩两次主线程 RPC，但依然是“先阻塞读，再阻塞写”。

15Hz 的预算只有 66.7ms。

- 如果 `get_state` 已经 50-70ms
- 再加 `send_cmd` 50-60ms

那么就算 transport 本身没问题，这个架构也注定跑不到 15Hz，主循环实际掉到 7-8Hz 是符合现象的。

### 5.5 当前代码仍有冗余发送

即使现在“不用底盘移动”，当前 `set_full_command()` 仍会在每帧做这些事：

- 发送双臂关节命令
- 发送底盘速度命令（通常是零速度）
- 发送升降柱高度命令

这意味着：

- 底盘虽然不作为当前控制目标，但底盘/lift 相关 publish 并没有退出控制链路
- 如果高度没有变化，却仍然每帧重复发 `set_height()`，这可能是额外负担

### 5.6 已确认一个底盘命令覆盖 bug，但它不是当前主问题

当前 bridge 的 `set_full_command()` 先调 `set_chassis_cmd(vx, vy, wz)`，随后又调 `set_height(height)`。

而 `set_height()` 内部会把 `chx/chy/chz` 重新写成 `0.0`。

结论:

- 如果将来重新启用底盘平移/旋转，这会导致底盘速度命令被后续高度消息覆盖。
- 这解释了此前“推摇杆后晚响应/一卡一卡”的底盘现象。
- 但在当前“只看双臂 + 升降柱”的范围里，它是已知 bug，不是当前主根因。

---

## 6. 修正后的根因优先级

### P1: server/bridge/ROS2/controller 执行路径过慢

当前最可疑的不是 ZeroRPC wire 本身，而是:

- `server.get_full_state() -> bridge.get_full_state()`
- `server.set_full_command() -> bridge.set_full_command()`
- 再往下的 ROS2 publish / controller 处理链路

### P2: 同步串行主循环放大控制时延

即使 RPC 本身未来能降到十几毫秒，当前“阻塞读 + 阻塞写”的结构仍然会把录制和控制耦合得很死。

### P3: 每帧附带发送 lift/chassis 相关命令

当前不用底盘移动，但 `set_full_command()` 仍然把底盘零速度和高度命令带进去，可能引入无效额外负担。

### P4: ZeroRPC/gevent 的线程问题只在特定用法下成立

- 结论仍成立: 多线程、多 client、普通 Python 线程直调 ZeroRPC 会出问题。
- 但它不是当前主线程单 client 架构下 50-70ms 平均延迟的充分解释。

---

## 7. 当前不再支持的旧结论

以下说法现在视为已被推翻或至少证据不足:

- “ZeroRPC 固有基础开销就是 50-70ms”
- “heartbeat + gevent 调度本身稳定吃掉 30-40ms”
- “直接替换成原生 ZMQ 就一定能解决当前问题”

现在更合理的说法是:

- 先把时延在 client / server / bridge / controller 之间拆清楚
- 再决定是继续优化 ARX server path，还是再换 transport

---

## 8. 下一步排查计划

### Step A: client 侧继续打点

目标: 确认端到端 RPC 耗时分布。

记录:

- `get_full_state` avg / p95 / p99 / max
- `set_full_command` avg / p95 / p99 / max
- 启动时打印实际 client 配置，确认运行时是否真的去掉了 heartbeat/timeout

### Step B: server 侧分段打点

目标: 把端到端 RPC 拆成 server 内部几段。

对 `get_full_state` 记录:

- `bridge_call`
- `serialize_response`
- `rpc_total`

对 `set_full_command` 记录:

- `bridge_call`
- `rpc_total`

### Step C: bridge 内部继续拆账

目标: 找到到底是哪个 bridge 子步骤慢。

对 `get_full_state` 记录:

- `lift_snapshot`
- `left_arm_snapshot`
- `right_arm_snapshot`

对 `set_full_command` 记录:

- `set_dual_joint_positions`
- `set_chassis_cmd`
- `set_height`

### Step D: 以“只用双臂 + 升降柱”为中心做测试矩阵

1. `mock_rpc`: 不接 ROS2/硬件，只测 RPC transport
2. `real_read_only`: 只测 `get_full_state`
3. `real_write_arms_only`: 只发双臂，不发底盘零速度，不发高度
4. `real_write_arms_plus_fixed_height`: 双臂 + 固定高度
5. `real_write_arms_plus_changing_height`: 双臂 + 变化高度
6. `real_read_write_loop`: 不开相机、不写 dataset，只保留主循环读写链路

### Step E: 根据结果再决定是否换通信层

决策规则:

- 如果 mock RPC 依然是亚毫秒，而 real RPC 很慢，先优化 server/bridge，不换 ZeroRPC
- 如果 bridge 内部很快，而 client 看到的端到端仍然很慢，再回头怀疑 gevent/调度
- 如果 `set_height()` 或额外底盘 publish 占了明显比例，先去掉冗余发送
- 只有在 server/bridge 已证明确实够快但端到端仍慢时，才进入 ZMQ 替换评估

---

## 9. 当前阶段的工作结论

一句话总结:

**当前“手臂 + 升降柱”卡顿，更像是 server/bridge/ROS2/control path + 同步主循环结构的问题，而不是 ZeroRPC wire 本身的问题。**

因此，下一阶段不应直接把精力放在“替换 ZeroRPC”上，而应先把 `get_full_state` 和 `set_full_command` 的 server 内部耗时拆清楚。

---

## 10. 第二轮复盘（pyzmq + ROS2 锁优化后）

### 10.1 当前实测数据

日志来源:

- `controllers_20260311_185733.log`
- `data_collection_20260311_185733.log`

主循环目标仍为 15Hz，但卡顿依旧明显:

```text
[PERF main_loop] 100 frames | get_state=33.6/446.8ms | cameras=7.8/49.9ms | send_cmd=35.6/273.8ms
[PERF main_loop] 200 frames | get_state=23.3/163.0ms | cameras=8.9/47.0ms | send_cmd=33.2/478.3ms
[PERF main_loop] 300 frames | get_state=31.4/232.0ms | cameras=6.6/58.0ms | send_cmd=31.8/387.2ms
[PERF main_loop] 400 frames | get_state=31.0/208.1ms | cameras=5.9/49.9ms | send_cmd=19.5/183.5ms
[PERF main_loop] 500 frames | get_state=30.8/402.3ms | cameras=4.6/40.5ms | send_cmd=22.3/167.2ms
[PERF main_loop] 600 frames | get_state=34.4/230.5ms | cameras=7.2/52.5ms | send_cmd=26.0/261.4ms
```

VR 线程依旧稳定 15Hz:

```text
[PERF vr_thread] 200 frames | actual 15.0Hz | vr_input=0.0/0.1ms | ik_solve=0.4/0.7ms
[PERF vr_thread] 400 frames | actual 15.0Hz | vr_input=0.2/1.3ms | ik_solve=0.4/0.7ms
[PERF vr_thread] 600 frames | actual 15.0Hz | vr_input=0.3/1.3ms | ik_solve=0.4/0.8ms
[PERF vr_thread] 800 frames | actual 15.0Hz | vr_input=0.3/0.9ms | ik_solve=0.4/0.7ms
```

server 侧 dispatch 已明显变快:

```text
[PERF server] 400 calls | get_full_state=0.01/0.03ms | set_full_command=1.10/67.28ms
[PERF server] 600 calls | get_full_state=0.01/0.03ms | set_full_command=1.92/86.12ms
[PERF server] 800 calls | get_full_state=0.01/0.03ms | set_full_command=1.44/83.58ms
[PERF server] 1000 calls | get_full_state=0.01/0.03ms | set_full_command=2.33/178.46ms
```

汇总后的当前判断:

| 组件 | 当前数据 | 结论 |
|------|---------|------|
| server `get_full_state` dispatch | `0.01/0.03ms` | bridge 快照读路径已经足够快，不是主瓶颈 |
| server `set_full_command` dispatch | 平均 `1-2ms`，最大 `35-178ms` | server 主线程明显改善，但写命令仍受线程调度影响 |
| main loop `get_state` | 平均 `23-34ms`，最大 `163-447ms` | 同步读状态热路径仍然偏重，且抖动大 |
| main loop `send_cmd` | 平均 `19-35ms`，最大 `167-478ms` | 同步写命令热路径仍然偏重，且抖动大 |
| cameras | 平均 `4.6-8.9ms` | 次要开销 |
| VR input / IK | `0.0-0.3ms` / `0.4ms` | 不是瓶颈 |

补充结论:

- 15Hz 帧预算只有 `66.7ms`
- 仅 `get_state + cameras + send_cmd` 三项平均就已经达到 `56-77ms`
- 峰值合计达到 `441-771ms`
- `dataset.add_frame()` 和 `Rerun` 显示不在上述 perf 统计内，但仍在主循环里同步执行

因此，当前卡顿已经不能再归因到“bridge 读状态慢”或“IK 慢”。

### 10.2 组件归因（第二轮）

- **ROS 控制器**: 当前证据最弱。控制器启动正常，首帧状态在启动后很快收到，日志里没有看到控制器层面的错误、超时或明显阻塞。
- **bridge**: 读路径已经明显优化，不是当前主责。写路径仍有改进空间，尤其是 `set_full_command()` 每帧同时发双臂、底盘速度和高度。
- **server/client**: 仍然是当前瓶颈之一。虽然 transport 从 ZeroRPC 换成 pyzmq 后有改善，但主循环每帧仍是“1 次同步读 RPC + 1 次同步写 RPC”，REQ/REP lockstep 会把抖动直接放大成控制延迟。
- **teleop**: IK 不是瓶颈，但 teleop 被绑定在 15Hz，而且 teleop 只负责生成目标，不掌控实际命令发送节奏，所以手感仍会发涩。
- **线程**: 是架构问题，不是简单“线程不够”。当前最关键的线程问题是两类: 一是 server 写命令采用 FIFO 队列，旧命令可能排队；二是 teleop VR 线程和主线程共享 `placo_robot/solver/target_*`，但没有显式同步。

### 10.3 当前已确认的主因

1. `record_loop` 同时承担 `control loop + dataset + Rerun/logging`，控制链路和录制/显示链路耦合过重。
2. 主循环每帧都做同步 `get_full_state()` 和同步 `set_full_command()`，两次 RPC 往返直接吃掉大部分 15Hz 预算。
3. server 的异步写命令虽然避免了 REP 主线程被 publish 卡死，但当前是“排队执行所有旧命令”，不是“只保留最新命令”，所以仍可能形成 backlog。
4. teleop/control 频率过低且不同步。现在 `teleop_fps = record_fps = 15Hz`，本身就不适合要求手感的遥操作。
5. `display: True`、`dataset.add_frame()`、相机读取、Rerun 写入都在控制热路径里，进一步放大抖动。

一句话结论:

**当前主因不是 ROS 控制器，也不是 IK；bridge 读路径也不是主责。真正的问题在于“录制主循环兼任控制主循环”的架构，以及围绕它形成的同步 RPC、FIFO 命令队列和线程共享状态设计。**

### 10.4 关于“一个进程起多个线程会不会更好”

- 如果只是把当前 client/server/recording 全塞进一个 Python 进程，再额外多开几个线程，通常不能根治问题；同步主循环、latest-command 缺失、录制/显示耦合仍然存在。
- 如果能在同一控制进程内去掉 RPC，让控制线程直接访问 bridge/latest snapshot，那么会更好；但当前项目被 Python 3.10（LeRobot）和 Python 3.12（ROS2 Jazzy）环境拆开，这不是零成本改造。
- 更合理的目标结构是:
  - `控制进程/线程`: 50-100Hz，只做 VR 输入、状态同步、命令下发
  - `录制/显示进程`: 15Hz，只做相机、dataset、Rerun

---

## 11. 更新后的根因优先级

### P1: 控制主循环和录制/显示链路耦合

- `record_loop()` 现在既是采集循环，又承担了真实控制循环的职责。
- 这会让任何相机、dataset、Rerun 抖动直接反映到控制延迟上。

### P2: 同步 RPC 热路径仍在每帧执行

- 当前每帧仍是 `get_full_state()` + `set_full_command()` 两次同步往返。
- transport 虽已切换到 pyzmq，但只要主循环仍是这种结构，抖动就会继续传到手感上。

### P3: server 写命令队列策略不对

- 当前 worker 线程执行的是 FIFO 队列。
- 对遥操作来说，更合理的是 `latest-only` 或按命令类型做合并，而不是忠实执行过时命令。

### P4: teleop/control 频率设计不合理

- `teleop_fps = record_fps = 15Hz`。
- 15Hz 适合录数据，不适合做有手感要求的遥操作闭环。

### P5: 线程共享状态存在竞争

- VR 线程和主线程共享 Placo 与 target 数据，没有显式锁或双缓冲。
- 它不是当前最大的耗时来源，但会放大 jitter、状态不一致和偶发卡顿。

### P6: ROS 控制器当前证据不足，不列为主因

- 在现有日志下，控制器更像是被上层节奏拖着走，而不是主动制造主要卡顿。

---

## 12. 下一步建议（按收益排序）

1. **解耦频率**: 增加 `teleop_fps`、`control_fps`、`record_fps` 三个独立配置，建议先试 `60 / 60 / 15`。
2. **改 server 命令队列**: 把 `_cmd_queue` 改成 `latest-only command slot` 或按方法名合并，优先只保留最新 `set_full_command`。
3. **把状态改成 latest snapshot**: 控制线程不要每帧同步 RPC 读状态，改成持续接收/读取最新状态缓存。
4. **先做 A/B 对照**:
   - 关闭 `display`
   - 暂停 `dataset.add_frame()`
   - 暂停相机采集
   - 只保留双臂控制
5. **最后再单独验 controller**: 如果上面都做完仍然卡，再去测 `publish -> hardware state feedback` 的闭环时延。

当前阶段的工程判断:

**继续单独抠 ROS2 锁，收益已经很有限；接下来最值得动刀的是控制架构、命令队列语义，以及 control/record 的解耦。**

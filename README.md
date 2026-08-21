# PICO 4 VR 遥操作插件

[![Python](https://img.shields.io/badge/Python-3.12+-3776AB?logo=python&logoColor=white)](https://www.python.org/downloads/)
[![LeRobot](https://img.shields.io/badge/LeRobot-0.6.x-FFD21E?logo=huggingface&logoColor=white)](https://github.com/huggingface/lerobot)
[![License](https://img.shields.io/badge/License-Apache--2.0-3377FF)](LICENSE)

面向 [LeRobot](https://github.com/huggingface/lerobot) 0.6.x 与 Seeed Studio reBot B601-DM（达妙电机）的 PICO 4 手柄**笛卡尔遥操作**插件。

- **自适应 6-DoF QP IK** —— 默认以 `gripper_end` 帧跟踪完整位姿；随归一化 `sigma_min` 下降连续增大阻尼并降低姿态权重，可切换纯位置模式
- **离合式激活** —— Grip 按住激活、松开立即冻结；启动、跟踪恢复或回位指令后须先完全松开 Grip 方可再次激活
- **latest-only 线程模型** —— TCP 接收、QP IK 工作线程与主控制循环互不阻塞，仅消费最新样本与最新 IK 结果
- **分层安全钳制** —— 关节限位、QP/整形器速度加速度约束、命令-反馈窗口、follower 相对目标；反馈异常进入 HOLD 冻结，连续故障受控退出并保留扭矩
- **双 VR 后端** —— XRoboToolkit V1（TCP，零额外依赖）与 Isaac Teleop + CloudXR（可选 extra，不提供按键输入）
- **CSV 记录与桌面分析** —— `--csv-log` 逐帧异步记录，`rebot-vr-csv-analyze` 本地可视化

## 要求

| 类别 | 要求 |
|---|---|
| 机械臂 | Seeed Studio reBot B601-DM |
| VR 设备 | PICO 4：XRoboToolkit（V1 后端） |
| 主机 | Linux；串口转 CAN 桥（默认 `/dev/ttyACM0`，damiao 协议，921600 baud） |
| Python | 3.12+ |
| LeRobot | `>=0.6.0,<0.7.0`（含 `rebot` extra，安装时自动引入） |

## 安全须知

> [!WARNING]
> - 上电前确认机械臂周围无障碍物；遥操期间人员靠近时随时准备松开 Grip（松开即冻结）。
> - 首次使用按[分阶段测试](#3-分阶段测试)从低倍率开始，确认映射方向与速度符合预期后再提速。
> - 默认退出即断电机扭矩（`--disable-torque-on-disconnect` 默认开启），**退出前请托住机械臂**；需保持使能时使用 `--no-disable-torque-on-disconnect`。
> - [参数表](#参数)“最大值”列为实机验证的安全上限：夹爪端点与扭矩比例由 CLI 强制校验，速度、加速度、fps 等上限不做 CLI 校验，请勿超过。

## 安装

本包需安装进 LeRobot 所在的虚拟环境（依赖其中的 `lerobot`）：

```bash
source /path/to/lerobot/.venv/bin/activate
pip install -e .          # 可编辑安装；亦可用 uv pip install -e .
```

## 快速开始

### 1. VR 数据自检（不连机械臂）

```bash
rebot-vr-print --backend xrobotoolkit_v1 --host 0.0.0.0 --port 63901 --hand right --rate 10
```

63901 端口同一时间只允许一个进程监听；自检结束后退出本命令再启动遥操。

### 2. 实机遥操

```bash
rebot-vr-teleoperate --robot-port /dev/ttyACM0 --backend xrobotoolkit_v1
```

启动后机械臂先以受限速度移动到 `--initial-q` 起始姿态（默认 `(0, 0.8, 0.8, 0, 0, 0)` rad，RS 参考系，q2/q3 自动换算达妙符号），随后进入遥操待命。先完全松开 Grip 一次，再按住 Grip 激活遥操。

### 3. 分阶段测试

| 阶段 | 目的 | 参数 |
|---|---|---|
| 1 | 仅位置（不跟踪姿态） | `--ik-mode position --position-scale 0.2 --max-joint-speed-rad-s 0.1` |
| 2 | 仅姿态 | `--position-scale 0 --orientation-scale 0.3 --max-joint-speed-rad-s 0.1` |
| 3 | 完整映射 | `--position-scale 1.0 --orientation-scale 1.0 --max-joint-speed-rad-s 0.4` |

### 4. CSV 记录与分析

`--csv-log` 指定路径后，由独立线程逐帧异步写入 CSV（不在控制循环内做磁盘 I/O）：32 列，含时间戳、主循环频率、遥操状态、IK 结果与原因、7 关节（q1–q6 + 夹爪）的 actual/target/command 位置、TCP 位置/姿态误差、`sigma_min`、condition number、QP 求解时间与 `dq` 范数。

```bash
rebot-vr-teleoperate --robot-port /dev/ttyACM0 --backend xrobotoolkit_v1 --csv-log logs/session.csv
```

事后分析使用本地桌面程序：

```bash
rebot-vr-csv-analyze logs/session.csv
```

左侧信号树共 43 路信号：21 路关节位置（7 关节 × actual/target/command）、14 路跟踪误差（actual−command 与 actual−target）、8 路诊断量（`control_loop_hz`、`ik_success`、TCP 位置误差（mm）与姿态误差、`sigma_min`、condition number、QP 求解时间、`dq` 范数），支持分组勾选与预设批量选择。绘图区：左键框选缩放、滚轮缩放、中键拖动平移、双击或右键复位、十字光标读帧；底部统计表给出可见窗口内每路信号的 N、min、max、mean、RMS、最大绝对值及其时刻；不同单位的曲线可启用“逐曲线归一化”比较趋势；时间轴下方色带标注遥操状态分段。

## 手柄按键

| 控制 | 行为 |
|---|---|
| Grip | 按住激活遥操（按压阈值 `--grip-press` 0.85），松开冻结并保持当前姿态（释放阈值 `--grip-release` 0.75） |
| Trigger | 夹爪开合：`0` → `--gripper-open-deg`，`1` → `--gripper-closed-deg`；有效 Tracking 建立后立即生效，不依赖 Grip |
| A / X | 平滑返回 `--initial-q` 起始姿态 |
| B / Y | 平滑返回六轴零点并闭合夹爪；Trigger 相对按下时刻移动 ≥ 0.05 后恢复开合控制 |

A/X 与 B/Y 仅 `xrobotoolkit_v1` 后端提供（Isaac 后端不含按键输入）；位姿映射当前固定右手控制器（`--hand left` 不可用）。回位指令及任何状态重置后，须先完全松开 Grip 才能再次激活。

`--gripper-open-deg`（默认 −180）与 `--gripper-closed-deg`（默认 0）是 Trigger 映射的两个端点，CLI 强制校验 `-270 ≤ open < closed ≤ 0`。移动到起始姿态期间夹爪保持实际反馈位置；进入 VR 主循环并取得新鲜 Tracking 后才应用 Trigger 映射。绕过 VR 直接验证夹爪标定时使用独立命令：

```bash
# 移动到开口测试点；q1-q6 持续保持实际反馈位置
rebot-gripper-test --robot-port /dev/ttyACM0 --target-deg -100 \
  --speed-deg-s 90 --acceleration-deg-s2 360 --relative-target-deg 10

# 回到标定闭合零点
rebot-gripper-test --robot-port /dev/ttyACM0 --target-deg 0
```

该工具不读取 PICO、Trigger 或 Grip；`--target-deg` 为电机角度，范围 `[−270, 0]`（−270 张开，0 闭合）。默认退出后保留电机扭矩以避免机械臂失去支撑；确认已支撑机械臂后才使用 `--disable-torque-on-disconnect`。

## 参数

`rebot-vr-teleoperate` 常用参数（完整参数：`rebot-vr-teleoperate --help`）：

| 参数 | 默认 | 最大值 | 说明 |
|---|---|---|---|
| `--position-scale` | `1.0` | — | 手柄位移 → TCP 位移倍率 |
| `--orientation-scale` | `1.0` | — | 手柄旋转 → 末端旋转倍率 |
| `--max-joint-speed-rad-s` | `5.5` | **5.5** | q1–q3 速度约束（rad/s）；起始位姿移动时作用于全部六轴 |
| `--max-joint-acceleration-rad-s2` | `20` | **20** | q1–q3 加速度约束（rad/s²）；起始位姿移动时作用于全部六轴 |
| `--wrist-speed-rad-s` | `12` | **12** | q4–q6 速度约束（rad/s） |
| `--wrist-acceleration-rad-s2` | `60` | **60** | q4–q6 加速度约束（rad/s²） |
| `--max-relative-target-deg` | `20` | **20** | 臂部 follower 相对目标钳制（deg） |
| `--wrist-relative-target-deg` | `20` | **20** | q4–q6 follower 相对目标钳制（deg） |
| `--gripper-relative-target-deg` | 跟随臂部 | **20** | 夹爪 follower 相对目标钳制（deg）；缺省跟随 `--max-relative-target-deg` |
| `--gripper-open-deg` | `-180` | — | Trigger=0 的开口端点；CLI 强制 `-270 ≤ open < closed ≤ 0` |
| `--gripper-closed-deg` | `0` | — | Trigger=1 与 B/Y 回零的闭合端点 |
| `--gripper-max-speed-deg-s` | `1200` | **1200** | 夹爪速度上限（°/s） |
| `--gripper-max-acceleration-deg-s2` | `5000` | 推荐 ≤50000 | 夹爪加速度上限（°/s²） |
| `--gripper-torque-ratio` | `0.2` | `1.0` | FORCE_POS 最大夹持力比例；CLI 强制 [0, 1] |
| `--fps` | `70` | ≤120 | 主循环频率（Hz） |
| `--qp-solver` | `scipy` | `scipy/osqp` | QP 后端；OSQP 需安装 `.[qp]` |
| `--ik-mode` | `pose` | `pose/position` | 完整位姿或纯 XYZ 任务 |
| `--qp-position-cost` | `20` | — | TCP 位置任务权重 |
| `--qp-orientation-cost` | `2` | — | 正常区域 TCP 姿态任务权重 |
| `--qp-orientation-cost-min` | `0.05` | — | 严重奇异区域姿态权重下限 |
| `--qp-position-gain` | `10` | — | 位置误差反馈增益（1/s），与目标线速度前馈相加 |
| `--qp-orientation-gain` | `8` | — | 姿态误差反馈增益（1/s），与目标角速度前馈相加 |
| `--arm-command-lookahead-ms` | `50` | — | q1–q3 POS_VEL 位置命令前视时间 |
| `--wrist-command-lookahead-ms` | `25` | — | q4–q6 POS_VEL 位置命令前视时间 |
| `--qp-damping`（别名 `--qp-damping-min`） | `1e-3` | — | 正常区域最小阻尼 |
| `--qp-damping-max` | `0.1` | — | 严重奇异区域最大阻尼 |
| `--singularity-threshold` | `0.08` | — | 开始自适应的归一化 `sigma_min` |
| `--singularity-critical-threshold` | `0.02` | — | 达到最大保护的归一化 `sigma_min` |
| `--singularity-characteristic-length-m` | `0.3` | — | Jacobian 线速度行的尺度归一化长度（m） |
| `--qp-smoothness-cost` | `0.05` | — | 速度连续性正则 |
| `--qp-posture-cost` | `0.05` | — | 回归 nominal 姿态正则 |
| `--joint-limit-margin-deg` | `2` | — | QP 关节限位内缩余量（deg） |
| `--qp-max-solve-time-ms` | `8` | — | 单次 QP 时间预算；超预算结果被丢弃 |
| `--feedback-fault-max-consecutive` | `5` | — | HOLD 连续故障帧数达到后受控退出 |
| `--csv-log` | 不记录 | — | 逐帧异步写入关节状态与 IK 诊断 CSV |
| `--status-rate` | `5` | — | 状态行输出频率（Hz） |

> [!IMPORTANT]
> “最大值”列为**实机验证的安全上限**，电机硬件空载上限见[最大速度与加速度](#最大速度与加速度)。CLI 仅强制校验 `--gripper-torque-ratio`（[0, 1]）与夹爪端点（`-270 ≤ open < closed ≤ 0`），不校验速度、加速度、fps 等上限；超出后电机物理上无法达到。

### 最大速度与加速度

B601-DM 电机硬件上限：

| 关节 | 电机 | 空载最大 | 额定 |
|---|---|---|---|
| q1–q3 | 达妙 DM-J4340P-2EC（40:1） | **5.5 rad/s**（315°/s） | 3.8 rad/s |
| q4–q6 | 达妙 DM-J4310-2EC（10:1） | **20.9 rad/s**（1200°/s） | 12.6 rad/s |
| 夹爪 | 达妙 DM-J4310-2EC（10:1） | **20.9 rad/s**（1200°/s） | 12.6 rad/s |

生产/采集推荐上限（保留余量）：

```bash
rebot-vr-teleoperate \
  --max-joint-speed-rad-s 5.5 --max-joint-acceleration-rad-s2 20 \
  --wrist-speed-rad-s 12 --wrist-acceleration-rad-s2 60 \
  --max-relative-target-deg 20
```

夹爪达到电机空载上限（1200°/s）且不放宽臂部保护：

```bash
rebot-vr-teleoperate \
  --gripper-max-speed-deg-s 1200 \
  --gripper-max-acceleration-deg-s2 20000 \
  --gripper-relative-target-deg 20
```

臂部与夹爪命令均依次经过三层钳制：QP/整形器速度与加速度约束 → controller 命令-反馈窗口（相对目标 × 0.9）→ follower 相对目标。相对目标对应等效速度上限 `相对目标 × fps`：默认 70 fps、20° 相对目标对应 1400°/s，满足 1200°/s。以 20000°/s² 加速到 1200°/s 需 36°，小于默认 180° 行程（−180° → 0°）。

> [!TIP]
> 相对目标须 ≥ 最大速度 ÷ fps，否则实际速度受限；加速度建议从低值分档上调（10 → 20 → 40 → 60），每档观察跟踪误差与跳变冲击。

## 工作原理

```text
PICO 手柄位姿（XRoboToolkit V1 TCP / Isaac CloudXR）
  → OpenXR → reBot 基座坐标变换
  → Grip 激活时锁存手柄与 gripper_end 位姿参考（初始目标 = 当前 FK 位姿）
  → Pinocchio LOCAL_WORLD_ALIGNED FK 与任务 Jacobian（线速度行按特征长度归一化）
  → SVD 监测归一化 sigma_min，smoothstep 插值阻尼与姿态权重
  → 目标 twist 前馈 + 误差反馈构成 QP 任务，盒约束求解 dq
  → q_actual + dq × lookahead 生成分轴目标（臂 50 ms / 腕 25 ms）
  → 关节限位、命令-反馈窗口、follower 相对目标三层钳制后 send_action()
```

- **命令生成**：ACTIVE 状态下 QP 已直接约束关节速度与加速度，命令按 `q_actual + dq × lookahead` 生成，不经过位置整形器；A/B 回位、起始位姿与夹爪使用速度/加速度位置整形。臂部固定达妙 `pos_vel` 模式；夹爪默认 `force_pos`（`--gripper-control-mode` 可选 `mit`）。臂部不使用 MIT 模式：无模型重力前馈时负载下存在稳态位置误差，起始位姿与 TCP 跟踪无法收敛。
- **线程模型**：V1 后端三个常驻线程——TCP 接收（`rebot-vr-v1`，原子替换最新样本）、QP 工作线程（`rebot-vr-qp`，latest-only；请求携带 generation/sequence/sample_id、实际反馈、上一速度、目标 twist 与 dt）、主控制线程（独占机器人反馈、状态机与唯一的 `send_action()` 调用）；`--csv-log` 启用时追加一个写盘线程；Isaac 后端无 TCP 接收线程。消费端仅采用当前 generation 的最新结果；Grip 捕获首帧不求解，QP 失败时保持上一完整六轴目标。
- **状态行**：按 `--status-rate`（默认 5 Hz）限频输出状态、`sigma_min`、condition number、当前阻尼与姿态权重、QP `dq`、求解时间与结果年龄、主循环实测频率、样本年龄、各阶段耗时与 TCP 实际/目标位姿，不逐控制周期打印。

**安全状态机**：

| 状态 | 含义 |
|---|---|
| `WAITING` | 尚未收到有效 Tracking |
| `IDLE` | Tracking 新鲜但未激活；Trigger 可独立控制夹爪 |
| `ACTIVE` | 遥操激活 |
| `STALE` | Tracking 超时或断连：保持当前位置，须松开 Grip 后才能再次激活 |
| `HOLD` | 反馈缺失/非有限/超限：冻结最后有效命令；连续 `--feedback-fault-max-consecutive`（默认 5）帧后受控退出并保留扭矩 |

设计细节见 [CONTROL_DESIGN.md](assest/docs/CONTROL_DESIGN.md) 与 [INVERSE_KINEMATICS_DESIGN.md](assest/docs/INVERSE_KINEMATICS_DESIGN.md)。

## LeRobot 插件集成

包名符合 `lerobot_teleoperator_*` 约定，导入即把 `rebot_vr` 注册进 LeRobot 的遥操器 registry。LeRobot 0.6 的通用 `teleoperate/record` 循环不向 teleoperator 提供机器人反馈，而本插件的 `get_action()` 在缺少新鲜反馈时直接抛出 `RuntimeError`（fail-closed），不退化为开环控制。实机一律使用 `rebot-vr-teleoperate`。

## 故障排除

| 现象 | 处理 |
|---|---|
| `command not found` | 激活 LeRobot 虚拟环境；或改用 `python -m lerobot_teleoperator_rebot_vr.teleoperate_real` |
| `Address already in use` | 关闭占用 63901 的其他进程（如仍在运行的 `rebot-vr-print`） |
| 按住 Grip 仍为 `idle` | 先完全松开 Grip 一次再重新按住 |
| `state=stale` | VR 数据流中断：检查 PICO 端发送与网络 |
| `state=hold` / 反馈异常 | 检查 CAN 连接与标定；连续故障帧后保留扭矩受控退出 |
| 退出后机械臂下坠 | 退出前托住机械臂；默认断开时关闭扭矩，可用 `--no-disable-torque-on-disconnect` 保持使能 |
| `lerobot-teleoperate --teleop.type=rebot_vr` 报 feedback 错误 | 预期行为：LeRobot 通用循环不提供反馈，请改用 `rebot-vr-teleoperate` |
| 实际速度达不到设定值 | 受相对目标限制：臂部增大 `--max-relative-target-deg`、夹爪增大 `--gripper-relative-target-deg`（须 ≥ 最大速度 ÷ fps） |

## 文档

- [控制设计](assest/docs/CONTROL_DESIGN.md) —— 线程模型、坐标映射、安全状态与反馈故障处理
- [逆解设计](assest/docs/INVERSE_KINEMATICS_DESIGN.md) —— 从 VR 样本到六轴命令的完整推导

## 许可证

[Apache-2.0](LICENSE)

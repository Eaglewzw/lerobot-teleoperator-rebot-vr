# PICO 4 VR 遥操作插件

[![Python](https://img.shields.io/badge/Python-3.12+-3776AB?logo=python&logoColor=white)](https://www.python.org/downloads/)
[![LeRobot](https://img.shields.io/badge/LeRobot-0.6.x-FFD21E?logo=huggingface&logoColor=white)](https://github.com/huggingface/lerobot)
[![License](https://img.shields.io/badge/License-Apache--2.0-3377FF)](LICENSE)

面向 [LeRobot](https://github.com/huggingface/lerobot) 0.6.x 与 Seeed Studio reBot B601-DM（达妙电机）的 PICO 4 手柄**笛卡尔遥操作**插件。

- **自适应 6-DoF QP IK** —— 默认以 `gripper_end` 帧跟踪完整位姿；随归一化 `sigma_min` 下降连续增大阻尼并降低姿态权重，可切换纯位置模式
- **离合式激活** —— Grip 按住激活、松开立即冻结；启动、跟踪恢复或回位指令后须先完全松开 Grip 方可再次激活
- **latest-only 线程模型** —— TCP 接收、QP IK 工作线程与主控制循环互不阻塞，仅消费最新样本与最新 IK 结果
- **分层安全钳制** —— 关节限位、QP/整形器速度加速度约束、命令-反馈窗口、follower 相对目标；反馈异常进入 HOLD 冻结，连续故障受控退出并保留扭矩
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

常用参数与默认值见[参数说明](assest/docs/PARAMETERS.md)；完整参数：`rebot-vr-teleoperate --help`。速度与加速度的实机安全上限、电机硬件上限见该文档的[最大速度与加速度](assest/docs/PARAMETERS.md#最大速度与加速度)一节。

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

- [参数说明](assest/docs/PARAMETERS.md) —— `rebot-vr-teleoperate` 参数默认值、实机安全上限与电机硬件上限
- [控制设计](assest/docs/CONTROL_DESIGN.md) —— 线程模型、坐标映射、安全状态与反馈故障处理
- [逆解设计](assest/docs/INVERSE_KINEMATICS_DESIGN.md) —— 从 VR 样本到六轴命令的完整推导

## 许可证

[Apache-2.0](LICENSE)

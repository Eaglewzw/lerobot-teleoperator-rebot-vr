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
> - 上电前确认机械臂周围无障碍物；遥操期间人员靠近时随时准备松开 Grip。
> - 首次使用按[分阶段测试](#3-分阶段测试)从低倍率开始，确认映射方向与速度符合预期后再提速。
> - 默认退出即断电机扭矩（`--disable-torque-on-disconnect` 默认开启），**退出前请托住机械臂**；需保持使能时使用 `--no-disable-torque-on-disconnect`。
> - [参数表](assest/docs/PARAMETERS.md)“最大值”列为实机验证的安全上限：夹爪端点与扭矩比例由 CLI 强制校验，速度、加速度、fps 等上限不做 CLI 校验，请勿超过。

## 安装

本包需安装进 LeRobot 所在的虚拟环境：

```bash
source /path/to/lerobot/.venv/bin/activate
pip install -e .          # 可编辑安装；亦可用 uv pip install -e .
```

## 快速开始

### 1. VR 数据自检（不连机械臂）

```bash
# 63901 端口同一时间只允许一个进程监听；自检结束后退出本命令再启动遥操。
rebot-vr-print --backend xrobotoolkit_v1 --host 0.0.0.0 --port 63901 --hand right --rate 10
```


### 2. 实机遥操

```bash
# 起始姿态(0, 0.8, 0.8, 0, 0, 0)rad
rebot-vr-teleoperate --robot-port /dev/ttyACM0 --backend xrobotoolkit_v1
```


### 3. 分阶段测试

| 阶段 | 目的 | 参数 |
|---|---|---|
| 1 | 仅位置（不跟踪姿态） | `--ik-mode position --position-scale 0.2` |
| 2 | 仅姿态 | `--position-scale 0 --orientation-scale 1.0` |
| 3 | 完整映射 | `--position-scale 1.0 --orientation-scale 1.0` |

### 4. CSV 记录与分析


```bash
rebot-vr-teleoperate --robot-port /dev/ttyACM0 --backend xrobotoolkit_v1 --csv-log logs/session.csv

rebot-vr-csv-analyze logs/session.csv
```


## 手柄按键

| 控制 | 行为 |
|---|---|
| Grip | 按住激活遥操，松开冻结并保持当前姿态 |
| Trigger | 夹爪开合：`0` → `--gripper-open-deg`，`1` → `--gripper-closed-deg`|
| A / X | 平滑返回 `--initial-q` 起始姿态 |
| B / Y | 平滑返回六轴零点并闭合夹爪；Trigger 相对按下时刻移动 ≥ 0.05 后恢复开合控制 |

A/X 与 B/Y 仅 `xrobotoolkit_v1` 后端提供；位姿映射当前固定右手控制器（`--hand left` 不可用）。回位指令及任何状态重置后，须先完全松开 Grip 才能再次激活。

`--gripper-open-deg`（默认 −180）与 `--gripper-closed-deg`（默认 0）是 Trigger 映射的两个端点，CLI 强制校验 `-270 ≤ open < closed ≤ 0`。移动到起始姿态期间夹爪保持实际反馈位置；进入 VR 主循环并取得新鲜 Tracking 后才应用 Trigger 映射。绕过 VR 直接验证夹爪标定时使用独立命令：

```bash
# 移动到开口测试点；q1-q6 持续保持实际反馈位置
rebot-gripper-test --robot-port /dev/ttyACM0 --target-deg -100 \
  --speed-deg-s 90 --acceleration-deg-s2 360 --relative-target-deg 10

# 回到标定闭合零点
rebot-gripper-test --robot-port /dev/ttyACM0 --target-deg 0
```

该工具不读取 PICO、Trigger 或 Grip；`--target-deg` 为电机角度，范围 `[−270, 0]`（−270 张开，0 闭合）。默认退出后保留电机扭矩以避免机械臂失去支撑；确认已支撑机械臂后才使用 `--disable-torque-on-disconnect`。


## LeRobot 插件集成

包名符合 `lerobot_teleoperator_*` 约定，导入即把 `rebot_vr` 注册进 LeRobot 的遥操器 registry。LeRobot 0.6 的通用 `teleoperate/record` 循环不向 teleoperator 提供机器人反馈，而本插件的 `get_action()` 在缺少新鲜反馈时直接抛出 `RuntimeError`（fail-closed），不退化为开环控制。实机一律使用 `rebot-vr-teleoperate`。

## 文档

- [参数说明](assest/docs/PARAMETERS.md) —— `rebot-vr-teleoperate` 参数默认值、实机安全上限与电机硬件上限
- [控制设计](assest/docs/CONTROL_DESIGN.md) —— 线程模型、坐标映射、安全状态与反馈故障处理
- [逆解设计](assest/docs/INVERSE_KINEMATICS_DESIGN.md) —— 从 VR 样本到六轴命令的完整推导

## 许可证

[Apache-2.0](LICENSE)

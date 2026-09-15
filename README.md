# reBot VR Teleoperation

基于 PICO 4 的 reBot B601-DM VR 遥操作插件。

<div align="center">

[![LeRobot](https://img.shields.io/badge/LeRobot-0.6.x-FFD21E?logo=huggingface&logoColor=white)](https://github.com/huggingface/lerobot)
![PICO 4](https://img.shields.io/badge/PICO-4-1675D1.svg)
![QP IK](https://img.shields.io/badge/IK-6--DoF_QP-orange.svg)
[![Python](https://img.shields.io/badge/Python-3.12+-3776AB?logo=python&logoColor=white)](https://www.python.org/downloads/)
[![License](https://img.shields.io/badge/License-Apache--2.0-3377FF)](LICENSE)

</div>

本项目面向 Seeed Studio reBot B601-DM（达妙电机），通过 LeRobot、VR 相对位姿映射与闭环 QP IK，实现六轴机械臂和夹爪的实机遥操作。

## 项目简介

系统接收 PICO 4 手柄位姿，将其映射为机械臂 TCP 目标，并结合实时关节反馈生成安全、连续的六轴控制命令。主要功能包括：

- **自适应 6-DoF QP IK**：支持位置、完整位姿跟踪和奇异位形自适应。
- **Grip 离合控制**：按住跟随，松开保持，恢复跟踪时防止突跳。
- **分层安全保护**：提供关节限位、命令整形、相对目标和异常反馈保护。
- **双控制模式**：默认使用 `POS_VEL`，可选带重力前馈的实验性 MIT 模式。
- **运行诊断**：记录关节、IK 和延迟数据，并生成 CSV 统计。

## 要求

| 项目 | 要求 |
|---|---|
| 机械臂 | Seeed Studio reBot B601-DM |
| VR | PICO 4 + 支持 XRoboToolkit V1 Tracking 协议的发送端 |
| 主机 | Linux，达妙串口转 CAN，默认 921600 baud |
| Python | 3.12+ |
| LeRobot | `>=0.6.0,<0.7.0`，包含 `rebot` extra |

## 安全提示

> [!WARNING]
> 程序默认在启动后移动到初始姿态，即使尚未按下 Grip。首次运行应降低速度并托住机械臂，确认各关节方向和零位正确。默认断开时关闭电机扭矩，退出前务必支撑机械臂。不要在反馈异常或电机未全部在线时绕过安全检查。

## 安装

在插件根目录执行，使用 Python 3.12+ 虚拟环境：

```bash
uv venv --python 3.12 .venv
source .venv/bin/activate
uv pip install -e .
rebot-vr-teleoperate --help
```

如果已有 LeRobot 环境，也可以将插件安装到该环境中（按实际位置替换路径）：

```bash
uv pip install \
  --python ~/Python/lerobot/.venv/bin/python \
  -e .

source ~/Python/lerobot/.venv/bin/activate
rebot-vr-teleoperate --help
```

基础安装包含 LeRobot reBot 支持、NumPy、SciPy、Pinocchio 和 PyYAML。默认 QP 后端为 SciPy；需要 OSQP 或开发测试依赖时，在已激活的环境中执行：

```bash
uv pip install -e '.[qp]'       # 可选 OSQP 后端
uv pip install -e '.[test]'     # pytest 测试依赖
```

## 启动前自检

### 1. 检查七个电机

扫描时不要同时运行遥操程序或其他占用串口的软件。

```bash
motorbridge-cli scan \
  --vendor damiao \
  --transport dm-serial \
  --serial-port /dev/ttyACM0 \
  --serial-baud 921600 \
  --start-id 1 \
  --end-id 7 \
  --feedback-base 0x10 \
  --timeout-ms 1000
```

正常结果应找到 ID 1–7 共七个电机。推荐使用 `/dev/serial/by-id/...` 稳定路径代替可能变化的 `/dev/ttyACM0`。

### 2. 按需标定零点

首次使用或零点需要重新标定时，停止其他串口程序，执行并按照标定工具提示操作：

```bash
lerobot-calibrate \
  --robot.type=rebot_b601_follower \
  --robot.port=/dev/ttyACM0 \
  --robot.id=rebot_b601_vr
```

标定时的 `robot.id` 应与遥操使用的 `--robot-id` 一致，默认均为 `rebot_b601_vr`。

### 3. 检查 VR 数据

主机监听 TCP 端口，将 PICO 发送端指向主机可达的 IP 和端口 `63901`；`0.0.0.0` 是主机监听地址，不是发送端的连接目标。以下命令只检查 VR 数据，不连接机械臂：

```bash
rebot-vr-print \
  --host 0.0.0.0 \
  --port 63901 \
  --hand right \
  --rate 10
```

确认能够持续收到 Tracking 数据后退出；端口 63901 同一时间只能由一个程序监听。

## 实机遥操

```bash
# POS_VEL 位置速度模式（默认，加载 config/pos_vel.yaml）
rebot-vr-teleoperate \
  --robot-port /dev/ttyACM0 \
  --motor-control-mode pos_vel

# MIT 模式（加载 config/mit.yaml）
rebot-vr-teleoperate \
  --robot-port /dev/ttyACM0 \
  --motor-control-mode mit
```

两种模式共用 VR 映射和 QP IK，命令行参数可覆盖 YAML。MIT 为实验模式，首次运行请托住机械臂并使用低增益验证。

### 启动与退出

启动默认移动到初始姿态，`--no-move-to-initial` 跳过。`Ctrl+C` 停止跟踪，q1–q6 回零后断开电机；再次 `Ctrl+C` 跳过回零直接断开。`--duration` 限时运行，退出默认失能电机。

### 手柄按键

| 按键 | 行为 |
| --- | --- |
| Grip | 按住激活位姿跟踪，松开保持 |
| Trigger | 夹爪开合（松开=-180°, 按到底=0°），独立于 Grip |
| A / X | 返回初始姿态 |
| B / Y | 返回六轴零点，闭合夹爪 |

启动或跟踪中断后需先完全松开 Grip 再按住以重新激活。

## 配置

默认加载 `config/pos_vel.yaml`，`--motor-control-mode mit` 时加载 `config/mit.yaml`。命令行覆盖 YAML。

| 常用参数 | 默认值 |
| --- | --- |
| `--hand` (left/right) | right |
| `--position-scale` / `--orientation-scale` | 1.0 |
| `--ik-mode` (pose/position) | pose |
| `--qp-solver` (scipy/osqp) | scipy |
| `--max-joint-speed-rad-s` / `--wrist-speed-rad-s` | 5.5 / 12.0 |
| `--fps` | 90 |
| `--stale-timeout` | 0.2 s |

完整参数见 `--help`，默认值以 YAML 为准。

`--ik-mode position` 只允许 q1-q3 参与位置 IK，QP 对 q4-q6 施加零速度硬约束，
因此腕部保持 Grip 激活时的反馈姿态。`pose` 模式仍使用全部六轴跟踪完整 TCP 位姿。
两种模式都在 QP 和 MIT 最终发送层根据剩余限位距离限制制动速度，不会通过放宽腕部
软件限位掩盖越界。

### MIT 模式（实验性）

```
τ = Kp×(q_cmd − q_fb) + Kd×(v_cmd − v_fb) + τ_gravity
```

MIT 为实验模式，需实机标定重力前馈。详见[控制设计](assest/docs/CONTROL_DESIGN.md)。

### MIT 分轴调参

`rebot-mit-tune` 不启动 VR 或 QP，只让指定关节相对当前姿态做小幅、平滑、可重复的往返运动：

```bash
rebot-mit-tune \
  --robot-port /dev/ttyACM0 \
  --joint q1 \
  --step-deg 5 \
  --kp 20 \
  --kd 5 \
  --csv-log logs/mit_tuning/q1-kp20-kd5.csv
```

只有现场输入 `RUN` 后程序才连接并使能电机；连接后锁存当前姿态并开始测试。退出时保持当前反馈片刻并失能全部电机。详细流程和数据字段见 [MIT 分轴调参](assest/docs/MIT_TUNING.md)。

## 诊断

```bash
rebot-vr-teleoperate --csv-log logs/session.csv            # 逐帧关节/IK/延迟
rebot-vr-teleoperate --csv-log logs/session.csv --motor-diagnostics  # 含电机 I/O
```

## 夹爪测试

```bash
rebot-gripper-test --robot-port /dev/ttyACM0 --target-deg -100
rebot-gripper-test --robot-port /dev/ttyACM0 --target-deg 0
```

## 测试

```bash
python -m pytest -q
```

## 文档

- [系统架构](assest/docs/ARCHITECTURE.md)
- [核心模块 API](assest/docs/API_REFERENCE.md)
- [控制设计](assest/docs/CONTROL_DESIGN.md)
- [MIT 分轴调参](assest/docs/MIT_TUNING.md)
- [逆解设计](assest/docs/INVERSE_KINEMATICS_DESIGN.md)
- [参数说明](assest/docs/PARAMETERS.md)

## 许可证

[Apache-2.0](LICENSE)

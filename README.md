# reBot VR Teleoperation

基于 PICO 4、LeRobot 和闭环 IK 的 reBot B601-DM 六轴机械臂实机遥操作插件，支持全身 QP、分离式 IK、夹爪控制和运行诊断。

<div align="center">

[![LeRobot](https://img.shields.io/badge/LeRobot-0.6.x-FFD21E?logo=huggingface&logoColor=white)](https://github.com/huggingface/lerobot)
![PICO 4](https://img.shields.io/badge/PICO-4-1675D1.svg)
![IK](https://img.shields.io/badge/IK-QP_%2B_Split-orange.svg)
[![Python](https://img.shields.io/badge/Python-3.12+-3776AB?logo=python&logoColor=white)](https://www.python.org/downloads/)
[![License](https://img.shields.io/badge/License-Apache--2.0-3377FF)](LICENSE)

</div>

<p align="center">
  <img src="assest/cut_30s.gif" width="640" alt="reBot VR 遥操作演示">
</p>

## 功能

- 🤖 **6‑DoF QP IK**：支持完整位姿或纯位置跟踪，并根据奇异程度调整姿态权重和阻尼。
- 🎯 **分离式 IK**：q1–q3 跟踪 joint4 轴心位置，q4–q6 只跟随手柄相对旋转，减少肩部转动引起的腕部补偿。
- ✋ **Grip 离合控制**：按住跟随，松开保持，重新激活时不会产生目标突跳。
- 🛡️ **分层安全保护**：包含关节限位、速度与加速度约束、相对目标钳制和反馈异常保护。
- 🔀 **双控制模式**：`POS_VEL` 位速模式和 `MIT` 位置-速度模式，用户按需自行选择。
- 📊 **运行诊断**：可记录关节、IK 和链路延迟数据。

## 运行要求

| 项目 | 要求 |
| --- | --- |
| 机械臂 | Seeed Studio reBot B601-DM（达妙电机） |
| VR | PICO 4 + XRoboToolkit V1 Tracking 发送端 |
| 主机 | Linux，达妙串口转 CAN，默认 921600 baud |
| Python | 3.12+ |
| LeRobot | `>=0.6.0,<0.7.0`，包含 `rebot` extra |
| 环境工具 | [uv](https://docs.astral.sh/uv/getting-started/installation/)；安装 PICO APK 时还需要 `adb` |

## 安全提示

> [!WARNING]
> 程序默认在启动后自动移动到初始姿态，即使尚未按下 Grip。首次运行必须托住机械臂，并使用下文的低速命令检查关节方向、零位和限位。程序退出时默认关闭电机扭矩，机械臂可能失去支撑；不要在电机未全部在线或反馈异常时绕过安全检查。

## 安装

在项目根目录创建 Python 3.12+ 环境：

```bash
uv venv --python 3.12 .venv
source .venv/bin/activate
uv pip install -e .
rebot-vr-teleoperate --help
```

已有 LeRobot 环境时，可直接安装到该环境（按实际路径调整）：

```bash
uv pip install \
  --python ~/Python/lerobot/.venv/bin/python \
  -e .

source ~/Python/lerobot/.venv/bin/activate
```

基础安装使用 SciPy QP 后端。OSQP 和测试依赖按需安装：

```bash
uv pip install -e '.[qp]'       # OSQP 后端
uv pip install -e '.[test]'     # pytest
```

## PICO 发送端

仓库提供 `assest/reBot.apk`。在 PICO 4 中启用开发者模式和 USB 调试，连接主机后安装：

```bash
adb install -r assest/reBot.apk
```

当前 APK 的 SHA-256：

```text
d8e10b85babe5cbf94389903398e228a3639f3f77183e98582a8cc303410b714
```

在 PICO 发送端填写主机的局域网 IP 和端口 `63901`。`0.0.0.0` 只用于主机监听，不能作为 PICO 的目标地址。

## 首次安全运行

### 1. 检查七个电机

扫描时不要运行遥操程序或其他占用串口的软件：

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

结果必须包含 ID 1–7 共七个电机。正式使用时，建议用 `/dev/serial/by-id/...` 替代可能变化的 `/dev/ttyACM0`。

### 2. 标定零点

首次使用或零点发生变化时，停止其他串口程序并执行：

```bash
lerobot-calibrate \
  --robot.type=rebot_b601_follower \
  --robot.port=/dev/ttyACM0 \
  --robot.id=rebot_b601_vr
```

标定和遥操必须使用相同的 `robot.id`；默认值为 `rebot_b601_vr`。

### 3. 检查 VR 数据

以下命令只检查 Tracking 数据，不连接机械臂：

```bash
rebot-vr-print \
  --host 0.0.0.0 \
  --port 63901 \
  --hand right \
  --rate 10
```

确认数据持续更新后退出。端口 `63901` 同一时间只能由一个程序监听。

### 4. 低速验证

托住机械臂，完全松开 Grip 后启动。此命令跳过自动初始姿态和退出回零，将速度、加速度、映射比例与相对目标窗口限制在较低水平：

```bash
rebot-vr-teleoperate \
  --robot-port /dev/ttyACM0 \
  --motor-control-mode pos_vel \
  --no-move-to-initial \
  --no-return-to-zero-on-exit \
  --position-scale 0.25 \
  --orientation-scale 0.25 \
  --max-joint-speed-rad-s 0.5 \
  --max-joint-acceleration-rad-s2 1.0 \
  --wrist-speed-rad-s 0.5 \
  --wrist-acceleration-rad-s2 1.0 \
  --max-relative-target-deg 5 \
  --wrist-relative-target-deg 5
```

通过小幅手柄动作确认各关节的运动方向、零位和限位正确后，再使用正式配置。退出仍会默认失能电机，操作过程中需要持续支撑机械臂。

## 日常遥操

```bash
# POS_VEL：默认模式，加载 config/pos_vel.yaml
rebot-vr-teleoperate \
  --robot-port /dev/ttyACM0 \
  --motor-control-mode pos_vel

# MIT：实验模式，加载 config/mit.yaml
rebot-vr-teleoperate \
  --robot-port /dev/ttyACM0 \
  --motor-control-mode mit

# POS_VEL + split：加载独立的分离式 IK profile
rebot-vr-teleoperate \
  --robot-port /dev/ttyACM0 \
  --motor-control-mode pos_vel \
  --control-config config/pos_vel_split.yaml

# MIT + split：MIT 参数不变，使用独立的分离式 IK profile
rebot-vr-teleoperate \
  --robot-port /dev/ttyACM0 \
  --motor-control-mode mit \
  --control-config config/mit_split.yaml
```

MIT 模式应先通过分轴调参验证增益和重力前馈，再进行 VR 遥操作。

### 手柄按键

| 按键 | 行为 |
| --- | --- |
| Grip | 按住激活位姿跟踪，松开保持 |
| Trigger | 夹爪开合；松开为 -180°，按到底为 0° |
| A / X | 返回配置的初始姿态 |
| B / Y | 返回六轴零点并闭合夹爪 |

启动或 Tracking 中断后，需要先完全松开 Grip，再按住以重新激活跟踪。

### 启动与退出

- 默认启动：移动到配置的 `initial_q`；使用 `--no-move-to-initial` 跳过。
- MIT 启动及退出回零发送整形后的轨迹位置，目标速度为 0，使用位置 PD 和重力补偿跟随；启动六轴统一使用 `max_joint_speed_rad_s` 和 `max_joint_acceleration_rad_s2`，退出回零另受 `exit_zero_speed_rad_s` 和 `exit_zero_acceleration_rad_s2` 限制。
- 第一次 `Ctrl+C`：停止跟踪，q1–q6 回零后断开电机。
- 第二次 `Ctrl+C`：跳过回零，直接断开电机。
- `--no-return-to-zero-on-exit`：退出时不回零；`--duration` 可限制运行时间。
- 断开时默认失能电机，退出前必须支撑机械臂。

## 配置

不指定 `--control-config` 时，程序根据 `--motor-control-mode` 加载兼容默认
`pos_vel.yaml` 或 `mit.yaml`，两者都保持 `pose` IK。需要明确组合时使用以下完整
profile，避免只切换 `ik_mode` 却继续沿用另一种解算方式的滤波、增益和运动限制：

| 电机控制 | pose QP | split IK |
| --- | --- | --- |
| POS_VEL | `config/pos_vel_pose.yaml` | `config/pos_vel_split.yaml` |
| MIT | `config/mit_pose.yaml` | `config/mit_split.yaml` |

`*_pose.yaml` 与当前兼容默认等价。两个 `*_split.yaml` 是保守的实机调试起点，
不是硬件极限或已经完成标定的最终参数。命令行显式参数的优先级仍高于 YAML。

兼容默认（pose）参数对比如下：

| 关键配置 | POS_VEL | MIT |
| --- | ---: | ---: |
| 配置文件 | `config/pos_vel.yaml` | `config/mit.yaml` |
| q1–q3 速度 / 加速度 | 5.5 rad/s / 20 rad/s² | 2.0 rad/s / 6 rad/s² |
| q4–q6 速度 / 加速度 | 12 rad/s / 60 rad/s² | 2.0 rad/s / 6 rad/s² |
| 臂部 / 腕部相对目标窗口 | 20° / 20° | 10° / 10° |
| 位置滤波 / 死区 | 关闭 / 0 m | 4 Hz / 0.015 m |
| QP 位置 / 姿态增益 | 10 / 8 | 4 / 2 |
| 控制状态 | 推荐 | 实验性 |

两种兼容默认均使用右手、`pose` IK、SciPy QP、90 Hz 主循环和 0.2 s 数据超时。实际值以所选 YAML 和 `rebot-vr-teleoperate --help` 为准。

自定义配置建议从对应的“电机控制 + IK”profile 复制，文件内的
`motor_control_mode` 必须和命令行一致：

```bash
cp config/mit_split.yaml config/my_mit_split.yaml

rebot-vr-teleoperate \
  --motor-control-mode mit \
  --control-config config/my_mit_split.yaml \
  --robot-port /dev/ttyACM0
```

`pose` 使用六轴跟踪 TCP 位姿；`position` 仅用 q1–q3 跟踪 TCP 位置并锁定腕部；`split` 用 q1–q3 跟踪 joint4 轴心位置，q4–q6 跟随手柄相对旋转。CLI 仍允许 `--ik-mode split` 覆盖，但实机使用建议选择完整 split profile，不要只覆盖这一项。详细的参数归属、调参顺序与 MIT 控制原理见[运行参数](assest/docs/PARAMETERS.md)和[控制设计](assest/docs/CONTROL_DESIGN.md)。

### MIT 分轴调参

`rebot-mit-tune` 不启动 VR 或 QP，只让指定关节在当前姿态附近进行小幅往返运动：

```bash
rebot-mit-tune \
  --robot-port /dev/ttyACM0 \
  --joint q1 \
  --step-deg 5 \
  --kp 20 \
  --kd 5 \
  --csv-log logs/mit_tuning/q1-kp20-kd5.csv
```

程序只会在现场输入 `RUN` 后连接并使能电机。完整流程见 [MIT 分轴调参](assest/docs/MIT_TUNING.md)。

## 诊断与工具

```bash
# 逐帧关节、IK 和延迟数据
rebot-vr-teleoperate --csv-log logs/session.csv

# 夹爪动作测试
rebot-gripper-test --robot-port /dev/ttyACM0 --target-deg -100
rebot-gripper-test --robot-port /dev/ttyACM0 --target-deg 0
```

## 常见问题

| 现象 | 检查项 |
| --- | --- |
| 串口打不开 | 停止其他占用进程，检查用户串口权限，优先使用 `/dev/serial/by-id/...` |
| 扫描不足七个电机 | 检查供电、CAN 接线、波特率和 ID 1–7 |
| 收不到 Tracking | PICO 应连接主机真实局域网 IP；确认处于同一网络，端口 `63901` 未被占用或拦截 |
| Grip 按下后不跟踪 | 先完全松开 Grip 再按住，并确认 Tracking 数据未超时 |
| OSQP 不可用 | 安装 `uv pip install -e '.[qp]'`，或继续使用默认 SciPy 后端 |
| 退出后机械臂下坠 | 默认行为是断开时失能电机；退出前必须托住机械臂 |

## 测试

```bash
python -m pytest -q
```

## 详细文档

- [系统架构](assest/docs/ARCHITECTURE.md)
- [核心模块 API](assest/docs/API_REFERENCE.md)
- [控制设计](assest/docs/CONTROL_DESIGN.md)
- [MIT 分轴调参](assest/docs/MIT_TUNING.md)
- [逆解设计](assest/docs/INVERSE_KINEMATICS_DESIGN.md)
- [参数说明](assest/docs/PARAMETERS.md)

## 许可证

[Apache-2.0](LICENSE)

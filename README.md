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

默认启动参考姿态为 `[0.0, 0.8, 0.8, 0.0, 0.0, 0.0]`，单位为弧度。`--initial-q` 使用 RS 参考约定，程序会将 q2、q3 符号转换为 DM 约定，默认对应 DM 关节角 `[0.0, -0.8, -0.8, 0.0, 0.0, 0.0]`。

如需跳过启动回位，可添加 `--no-move-to-initial`。正常启动后，先完全松开 Grip，再按住进行遥操。退出前支撑机械臂，然后按 `Ctrl+C`；也可用 `--duration 30` 设置运行时长（秒）。

启动反馈停更诊断：当前默认仅在移动到初始姿态期间，将每轴主动反馈请求限制为最多 20 Hz（`--initial-feedback-request-hz 20`），两处读取共用限频；运动发送、接收轮询和缓存读取继续按原频率执行。启动结束、异常或中断后恢复原行为，连接配置、VR 跟随、退出回零及失能不受此限频影响。设为 `0` 可恢复原始不限频行为。该选项不验证反馈新鲜度；实机 20 Hz 对照仍复现后四轴停更，不能视为故障修复，详见[测试记录](assest/docs/MOTOR_SHUTDOWN.md)。

正常按一次 `Ctrl+C` 后，程序先停止 VR 跟随，将六个机械臂关节缓慢返回已标定的 `0°`，夹爪保持回零开始时的位置；反馈满足到位条件后，由 VR 工程管理失能与断开。回零不重新标定电机零点。再次按 `Ctrl+C` 可中止回零并进入断开流程。

退出回零默认轨迹速度上限为 `0.5 rad/s`、加速度上限为 `1.0 rad/s²`，可通过 `--exit-zero-speed-rad-s` 和 `--exit-zero-acceleration-rad-s2` 设置，同时遵守更低的机械臂/腕部限制。POS_VEL 电机速度参数也会临时降低。到位容差、总超时和停滞超时复用 `--initial-move-tolerance-deg`、`--initial-move-timeout`、`--initial-stall-timeout`。回零失败或中止后仍执行断开，是否失能由 `--disable-torque-on-disconnect` 决定。

反馈异常、其他程序异常、`SIGTERM` 和 `--duration` 到时退出不自动回零；主循环持续反馈故障仍使用既有 HOLD 退出策略，启动异常则按 `disable_torque_on_disconnect` 决定是否失能。添加 `--no-return-to-zero-on-exit` 可关闭 Ctrl+C 回零。

失能由本工程的 `ManagedFollower` 直接调用已有 MotorBridge 电机对象，绕过 LeRobot 的 `disconnect()`；不修改 LeRobot 源码，不另开串口。默认对七轴分轮发送，未确认轴最多 3 轮，每次失能/反馈请求后间隔 20 ms，每轮观察反馈 100 ms，然后调用 `close_bus()` 排空传输并释放句柄。退出不调用 `clear_error()`。参数为 `--disable-attempts`、`--disable-interval-s`、`--disable-feedback-wait-s`，也可在模式 YAML 的 `safety` 中设置。

终端逐轴报告发送次数、缓存状态和 `confirmed=YES/NO`；已检查的 MotorBridge 0.3.9 和 0.5.3 的 `get_state()` 均缺少反馈接收时间戳/计数，因此即使缓存为 DISABLED 也只能报告 NO（未确认），不能保证解决已有的后四轴失能异常。设置 `--csv-log /tmp/run.csv` 会额外写入 `/tmp/run_shutdown.json`。设计、退出策略、0.5.3 环境升级与测试说明见 [失能与断开管理](assest/docs/MOTOR_SHUTDOWN.md)。

对照参数 `--no-disable-request-feedback` 只取消退出阶段的主动反馈请求，保留对应等待间隔、七轴连接及接收轮询；默认仍开启请求。2026-09-09 实测中，此对照仍出现后四轴绿灯，不能作为修复方案。实测日志、缓存停止变化的时间及分析边界已记录在上述文档中。

### 手柄按键

| 按键 | 行为 |
|---|---|
| Grip | 按住激活机械臂位姿跟踪，松开保持姿态 |
| Trigger | 控制夹爪开合；默认松开为 -180°，按到底为 0° |
| A / X | 返回初始姿态 |
| B / Y | 返回六轴零点并闭合夹爪 |

启动或跟踪中断后，必须先完全松开一次 Grip 才能重新激活。

夹爪 Trigger 控制独立于 Grip，有新鲜 Tracking 时即可生效；松开 Grip 不会停用夹爪。B / Y 的回零动作不等同于重新标定电机零点。

## 配置与调参

默认读取 [POS_VEL 配置](config/pos_vel.yaml)；指定 `--motor-control-mode mit` 时读取 [MIT 配置](config/mit.yaml)。显式命令行参数优先于 YAML 配置。自定义配置通过 `--control-config` 指定，其 `motor_control_mode` 必须与命令行选择一致：

```bash
rebot-vr-teleoperate \
  --motor-control-mode mit \
  --control-config config/mit.yaml \
  --robot-port /dev/ttyACM0
```

| 常用参数 | 作用 |
|---|---|
| `--hand left` / `right` | 选择手柄，默认右手 |
| `--position-scale` / `--orientation-scale` | 位移 / 旋转映射倍率，默认均为 1.0 |
| `--ik-mode pose` / `position` | 跟踪完整位姿 / 仅位置，默认 `pose` |
| `--qp-solver scipy` / `osqp` | QP 后端；OSQP 需安装 `qp` extra |
| `--max-joint-speed-rad-s` / `--wrist-speed-rad-s` | q1–q3 / q4–q6 速度上限 |
| `--fps` | 主循环目标频率，默认 90 Hz，实际频率取决于运行耗时 |
| `--stale-timeout` | VR 数据失效阈值，默认 0.2 秒 |

完整选项见 `rebot-vr-teleoperate --help`，模式默认值以对应 YAML 为准。

### MIT 增益与重力前馈

MIT 控制近似为：

```text
扭矩 = Kp × 位置误差 + Kd × 速度误差 + 前馈扭矩
```

- `Kp` 控制位置刚度：增大后跟随更紧，过大可能震荡；减小后更柔软，但位置误差可能增大。
- `Kd` 控制速度阻尼：增大通常有助于抑制过冲；过小可能导致摆动。
- `--mit-kp` 和 `--mit-kd` 均按 q1–q6 顺序接收六个数值。
- 重力前馈由 Pinocchio 和动力学 URDF 计算，通过 `--mit-gravity-scale` 调整倍率、`--mit-gravity-ramp-s` 设置渐入时间。

MIT 模式的重力前馈仍需实机标定。首次验证应托住机械臂，使用较低增益和重力倍率；可用 `--position-scale 0 --orientation-scale 0 --no-move-to-initial` 关闭 VR 位姿映射并跳过启动回位。这些选项仍会使能电机，Trigger 和回位按键仍可触发动作。详细说明见[参数说明](assest/docs/PARAMETERS.md)与[控制设计](assest/docs/CONTROL_DESIGN.md)。

## 运行日志

记录控制和延迟数据：

```bash
rebot-vr-teleoperate \
  --robot-port /dev/ttyACM0 \
  --csv-log logs/session.csv
```

逐帧数据写入 `logs/session.csv`，关闭日志时生成 `logs/session_latency_summary.csv`。日志包含关节反馈与命令、IK 状态及延迟诊断；默认不记录 CSV。

排查电机停住、突然运动或跟随异常时，在上述命令后添加 `--motor-diagnostics`，额外生成 `_motor_io.csv` 和 `_motor_summary.json`，记录最终电机 API 参数、原始缓存状态及统计。真实 CAN 接收时间/计数当前不可用；使用方法和字段限制见[电机诊断说明](assest/docs/MOTOR_DIAGNOSTICS.md)。

## 夹爪测试

绕过 VR 单独验证夹爪：

```bash
rebot-gripper-test --robot-port /dev/ttyACM0 --target-deg -100
rebot-gripper-test --robot-port /dev/ttyACM0 --target-deg 0
```

测试期间六个机械臂关节会保持在实际反馈位置。

安装测试依赖后，在项目根目录运行：

```bash
python -m pytest -q
```

## 文档

- [系统架构](assest/docs/ARCHITECTURE.md)
- [核心模块 API](assest/docs/API_REFERENCE.md)
- [电机发送与反馈诊断](assest/docs/MOTOR_DIAGNOSTICS.md)
- [参数说明](assest/docs/PARAMETERS.md)
- [控制设计](assest/docs/CONTROL_DESIGN.md)
- [逆解设计](assest/docs/INVERSE_KINEMATICS_DESIGN.md)

## 许可证

[Apache-2.0](LICENSE)

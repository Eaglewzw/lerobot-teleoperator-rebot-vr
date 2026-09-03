# reBot VR Teleoperation

### 基于 PICO 4的reBot B601-DM VR 遥操作插件

<div align="center">

[![LeRobot](https://img.shields.io/badge/LeRobot-0.6.x-FFD21E?logo=huggingface&logoColor=white)](https://github.com/huggingface/lerobot)
![PICO 4](https://img.shields.io/badge/PICO-4-1675D1.svg)
![QP IK](https://img.shields.io/badge/IK-6--DoF_QP-orange.svg)
[![Python](https://img.shields.io/badge/Python-3.12+-3776AB?logo=python&logoColor=white)](https://www.python.org/downloads/)
[![License](https://img.shields.io/badge/License-Apache--2.0-3377FF)](LICENSE)

</div>

> 本项目是面向 Seeed Studio reBot B601-DM（达妙电机）的 PICO 4 VR 遥操作插件，通过 LeRobot、VR 位姿映射与闭环 QP IK，实现低延迟、安全可控的全六轴笛卡尔遥操作，适用于实机控制、算法验证与遥操作研究。

---

## 项目简介

系统接收 PICO 4 手柄位姿，将其映射为机械臂 TCP 目标，并结合实时关节反馈生成安全、连续的六轴控制命令。主要功能包括：

- **自适应 6-DoF QP IK**：支持位置、完整位姿跟踪和奇异位形自适应。
- **Grip 离合控制**：按住跟随，松开保持，恢复跟踪时防止突跳。
- **分层安全保护**：提供关节限位、命令整形、相对目标和异常反馈保护。
- **双控制模式**：默认使用 `POS_VEL`，可选带重力前馈的实验性 MIT 模式。
- **运行诊断**：记录关节、IK 和延迟数据，并生成 CSV 统计。

---

## 要求

| 项目 | 要求 |
|---|---|
| 机械臂 | Seeed Studio reBot B601-DM |
| VR | PICO 4 + XRoboToolkit V1 |
| 主机 | Linux，达妙串口转 CAN，默认 921600 baud |
| Python | 3.12+ |
| LeRobot | `>=0.6.0,<0.7.0`，包含 `rebot` extra |

## 安全提示

> [!WARNING]
> 首次运行应降低速度并托住机械臂，确认各关节方向和零位正确。默认断开时关闭电机扭矩，退出前务必支撑机械臂。不要在反馈异常或电机未全部在线时绕过安全检查。

## 安装

在插件根目录执行：

```bash
uv pip install \
  --python ~/Python/lerobot/.venv/bin/python \
  -e .

source ~/Python/lerobot/.venv/bin/activate
rebot-vr-teleoperate --help
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

### 2. 检查 VR 数据

```bash
rebot-vr-print \
  --backend xrobotoolkit_v1 \
  --host 0.0.0.0 \
  --port 63901 \
  --hand right \
  --rate 10
```

确认能够持续收到 Tracking 数据后退出；端口 63901 同一时间只能由一个程序监听。

## 实机遥操

```bash
rebot-vr-teleoperate \
  --robot-port /dev/ttyACM0 \
  --backend xrobotoolkit_v1
```

程序默认先移动到 RS 参考姿态 `(0, 0.8, 0.8, 0, 0, 0) rad`，其中 q2/q3 会转换为 B601-DM 的符号约定。

建议按以下顺序验证：

```bash
# 仅位置
rebot-vr-teleoperate --robot-port /dev/ttyACM0 --ik-mode position

# 仅姿态
rebot-vr-teleoperate --robot-port /dev/ttyACM0 --position-scale 0

# 完整位姿
rebot-vr-teleoperate --robot-port /dev/ttyACM0
```

## 手柄按键

| 按键 | 行为 |
|---|---|
| Grip | 按住激活遥操，松开保持姿态 |
| Trigger | 控制夹爪开合 |
| A / X | 返回初始姿态 |
| B / Y | 返回六轴零点并闭合夹爪 |

启动或跟踪中断后，必须先完全松开一次 Grip 才能重新激活。

## 日志与实验模式

记录控制和延迟数据：

```bash
rebot-vr-teleoperate \
  --robot-port /dev/ttyACM0 \
  --csv-log logs/session.csv
```

MIT 模式包含动力学重力前馈，属于实机待标定功能。首次测试必须托住机械臂、关闭 VR 位姿映射并使用较低重力倍率；详细参数和测试流程见[参数说明](assest/docs/PARAMETERS.md)与[控制设计](assest/docs/CONTROL_DESIGN.md)。

## 夹爪测试

绕过 VR 单独验证夹爪：

```bash
rebot-gripper-test --robot-port /dev/ttyACM0 --target-deg -100
rebot-gripper-test --robot-port /dev/ttyACM0 --target-deg 0
```

测试期间六个机械臂关节会保持在实际反馈位置。

## 文档

- [架构与 API](assest/docs/ARCHITECTURE.md)
- [参数说明](assest/docs/PARAMETERS.md)
- [控制设计](assest/docs/CONTROL_DESIGN.md)
- [逆解设计](assest/docs/INVERSE_KINEMATICS_DESIGN.md)

## 许可证

[Apache-2.0](LICENSE)

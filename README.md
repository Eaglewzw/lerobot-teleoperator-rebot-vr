# reBot B601-DM PICO 4 VR 遥操作插件

[![Python](https://img.shields.io/badge/Python-3.12+-3776AB?logo=python&logoColor=white)](https://www.python.org/downloads/)
[![LeRobot](https://img.shields.io/badge/LeRobot-0.6.x-FFD21E?logo=huggingface&logoColor=white)](https://github.com/huggingface/lerobot)
[![License](https://img.shields.io/badge/License-Apache--2.0-3377FF)](LICENSE)

面向 LeRobot 0.6.x、Seeed Studio reBot B601-DM（达妙电机）和 PICO 4，提供低延迟、反馈闭环且安全优先的全六轴笛卡尔 VR 遥操作。

- **自适应 6-DoF QP IK**：将手柄位姿实时映射到机械臂 TCP，支持仅位置和完整位姿跟踪；接近奇异位形时自动调整阻尼与姿态权重，提高求解稳定性。
- **Grip 离合控制**：按住 Grip 才激活跟随，松开后立即停止映射并保持当前姿态；启动或跟踪中断后需要先完全松开，避免恢复时发生突跳。
- **分层安全保护**：对命令执行关节限位、速度与加速度整形及相对目标约束；反馈缺失、非有限或越界时进入 HOLD，持续异常则受控退出。
- **双电机控制路径**：默认采用稳定的 `POS_VEL` 位置速度模式，也可显式启用带 Pinocchio 重力前馈的实验性 MIT 模式，夹爪保持独立控制。
- **运行诊断与分析**：可将关节状态、VR 样本、IK 结果和各阶段延迟逐帧写入 CSV，并在退出时生成延迟统计，便于实机调参与故障定位。

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

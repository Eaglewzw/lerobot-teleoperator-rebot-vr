# PICO 4 VR 遥操作插件

[![Python](https://img.shields.io/badge/Python-3.12+-3776AB?logo=python&logoColor=white)](https://www.python.org/downloads/)
[![LeRobot](https://img.shields.io/badge/LeRobot-0.6.x-FFD21E?logo=huggingface&logoColor=white)](https://github.com/huggingface/lerobot)
[![License](https://img.shields.io/badge/License-Apache--2.0-3377FF)](LICENSE)

面向 [LeRobot](https://github.com/huggingface/lerobot) 0.6.x 与 Seeed Studio reBot B601-DM（达妙电机）的 PICO 4 手柄**笛卡尔遥操作**插件。

- **自适应 6-DoF QP IK** —— TCP 位姿跟随手柄；接近奇异时自动增大阻尼、降低姿态权重，避免求解失败，也可切换纯位置模式
- **离合式激活** —— 按住 Grip 激活，松开即冻结；启动或中断后须先完全松开一次再激活，防止机械臂突跳
- **latest-only 线程模型** —— VR 接收、QP IK、主控制循环各一个线程，只消费最新数据，互不阻塞
- **分层安全保护** —— 速度/加速度整形、关节限位、相对目标钳制；反馈异常进入 HOLD 冻结，连续故障受控退出并保持扭矩
- **CSV 记录与分析** —— `--csv-log` 逐帧写入关节与 IK 诊断，`rebot-vr-csv-analyze` 本地绘图分析

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

# 在本包根目录下执行（即 pyproject.toml 所在目录）
pip install -e .  

# 验证包已安装
pip show lerobot_teleoperator_rebot_vr

# 验证 LeRobot 能发现插件
python -c "
from lerobot_teleoperator_rebot_vr.config_rebot_vr import RebotVRTeleopConfig
print('插件注册名:', RebotVRTeleopConfig.plugin_name)
print('安装成功')
"

# 验证命令行工具可用
rebot-vr-teleoperate --help
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
| 1 | 仅位置（不跟踪姿态） | `--ik-mode position --position-scale 1.0` |
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
| Trigger | 夹爪开合：`0` → `open`，`1` → `closed`|
| A / X | 返回 `--initial-q` 起始姿态 |
| B / Y | 返回六轴零点并闭合夹爪 |

`--gripper-open-deg`（默认 −180）与 `--gripper-closed-deg`（默认 0）是 Trigger 映射的两个端点，CLI 强制校验 `-270 ≤ open < closed ≤ 0`。移动到起始姿态期间夹爪保持实际反馈位置；进入 VR 主循环并取得新鲜 Tracking 后才应用 Trigger 映射。绕过 VR 直接验证夹爪标定时使用独立命令：

```bash
# 移动到开口测试点；q1-q6 持续保持实际反馈位置
rebot-gripper-test --robot-port /dev/ttyACM0 --target-deg -100 \
  --speed-deg-s 90 --acceleration-deg-s2 360 --relative-target-deg 10

# 回到标定闭合零点
rebot-gripper-test --robot-port /dev/ttyACM0 --target-deg 0
```



## 文档

- [参数说明](assest/docs/PARAMETERS.md) —— `rebot-vr-teleoperate` 参数默认值、实机安全上限与电机硬件上限
- [控制设计](assest/docs/CONTROL_DESIGN.md) —— 线程模型、坐标映射、安全状态与反馈故障处理
- [逆解设计](assest/docs/INVERSE_KINEMATICS_DESIGN.md) —— 从 VR 样本到六轴命令的完整推导

## 许可证

[Apache-2.0](LICENSE)

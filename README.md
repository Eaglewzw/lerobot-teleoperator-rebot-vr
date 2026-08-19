# PICO 4 VR 遥操作插件

[![Python](https://img.shields.io/badge/Python-3.12+-3776AB?logo=python&logoColor=white)](https://www.python.org/downloads/)
[![LeRobot](https://img.shields.io/badge/LeRobot-0.6.x-FFD21E?logo=huggingface&logoColor=white)](https://github.com/huggingface/lerobot)
[![License](https://img.shields.io/badge/License-Apache--2.0-3377FF)](LICENSE)

面向 [LeRobot](https://github.com/huggingface/lerobot) 0.6.x 与 Seeed Studio reBot B601-DM（达妙电机）的 PICO 4 手柄**笛卡尔遥操作**插件。独立 pip 包，不修改 LeRobot 任何源码。

- **6-DoF 笛卡尔映射** —— 手柄位移 → 腕部中心位置 IK（q1–q3），手柄旋转 → 闭式姿态解（q4–q6），同一 VR 帧原子更新六轴目标
- **离合式死人手开关** —— Grip 按住激活、松开立即冻结；激活前必须先完全松开一次，防止手柄积累运动瞬间注入
- **三线程 latest-only 架构** —— TCP 接收 / 异步 IK / 主控制循环互不阻塞，永远只消费最新样本与最新 IK 结果
- **分层安全防护** —— 软限位、跳支保护、速度/加速度整形、follower 相对目标裁剪；反馈异常进入 HOLD 冻结，连续故障受控退出并保留扭矩
- **双 VR 后端** —— XRoboToolkit V1（TCP，零额外依赖）与 Isaac Teleop + CloudXR（可选 extra）
- **LeRobot 插件** —— 符合 `lerobot_teleoperator_*` 自动发现约定，导入即注册 `--teleop.type=rebot_vr`



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
> - 机械臂上电前确认周围无障碍物；遥操过程中人员靠近时随时准备松手（松开 Grip 即冻结）。
> - **首次使用务必按[分阶段测试](#3-分阶段测试)从低倍率开始**，确认映射方向与速度符合预期后再提速。
> - 默认退出即断电机扭矩（`--disable-torque-on-disconnect` 默认开启），**退出前请托住机械臂**；如需保持使能可加 `--no-disable-torque-on-disconnect`。
> - [参数表](#参数)中的"最大值"列为实机验证的安全上限，CLI 不做强制校验，请勿随意超过。

## 安装

本包需安装进 LeRobot 所在的虚拟环境（依赖其中的 `lerobot`）：

```bash
source /path/to/lerobot/.venv/bin/activate   # 激活 LeRobot 虚拟环境（按实际路径）
pip install -e .                             # 仓库目录下可编辑安装；亦可用 uv pip install -e .
```

## 快速开始

### 1. VR 数据自检（不连机械臂）

```bash
rebot-vr-print --backend xrobotoolkit_v1 --host 0.0.0.0 --port 63901 --hand right --rate 10
```

> [!NOTE]
> 63901 端口同一时间只能有一个进程监听；自检完成后请先退出本命令再启动遥操。

### 2. 实机遥操

```bash
rebot-vr-teleoperate --robot-port /dev/ttyACM0 --backend xrobotoolkit_v1
```

启动后机械臂先**限速移动到起始姿态**，随后进入遥操待命；**先完全松开 Grip 一次**，再按住 Grip 激活遥操。速度与加速度调节见[参数](#参数)，首次使用请先按下一节的分阶段流程操作。

### 3. 分阶段测试

| 阶段 | 目的 | 参数 |
|---|---|---|
| 1 | 仅位置（姿态锁定） | `--position-scale 0.2 --orientation-scale 0 --max-joint-speed-rad-s 0.1` |
| 2 | 仅姿态 | `--position-scale 0 --orientation-scale 0.3 --max-joint-speed-rad-s 0.1` |
| 3 | 完整映射 | `--position-scale 1.0 --orientation-scale 1.0 --max-joint-speed-rad-s 0.4` |

## 手柄按键

| 控制 | 行为 |
|---|---|
| Grip | 按住激活遥操，松开冻结并保持当前姿态（离合） |
| Trigger | 夹爪开合（`0` 张开 → `1` 闭合），不依赖 Grip |
| A / X | 平滑返回起始姿态（右手 / 左手） |
| B / Y | 平滑返回六轴零点（右手 / 左手） |

## 参数

`rebot-vr-teleoperate` 常用参数（完整参数：`rebot-vr-teleoperate --help`）：

| 参数 | 默认 | 最大值 | 说明 |
|---|---|---|---|
| `--position-scale` | `1.0` | — | 手柄位移 → 腕部中心位移倍率 |
| `--orientation-scale` | `1.0` | — | 手柄旋转 → 末端旋转倍率 |
| `--max-joint-speed-rad-s` | `5.5` | **5.5** | q1–q3 速度上限（rad/s） |
| `--max-joint-acceleration-rad-s2` | `20` | **20** | q1–q3 加速度上限（rad/s²） |
| `--wrist-speed-rad-s` | `12` | **12** | q4–q6 速度上限（rad/s） |
| `--wrist-acceleration-rad-s2` | `60` | **60** | q4–q6 加速度上限（rad/s²） |
| `--max-relative-target-deg` | `20` | **20** | 臂部 follower 相对目标保护（deg） |
| `--wrist-relative-target-deg` | `20` | **20** | q4–q6 follower 相对目标保护（deg） |
| `--gripper-relative-target-deg` | `20` | **20** | 夹爪 follower 相对目标保护（deg） |
| `--gripper-max-speed-deg-s` | `1200` | **1200** | 夹爪速度上限（°/s） |
| `--gripper-max-acceleration-deg-s2` | `5000` | 推荐 ≤50000 | 夹爪加速度上限（°/s²） |
| `--gripper-torque-ratio` | `0.2` | `1.0` | 夹爪最大夹持力比例 |
| `--fps` | `60` |  ≤120 | 主循环频率 |

> [!IMPORTANT]
> 上表最大值为**实机验证的安全上限**；电机硬件空载上限见[最大速度与加速度](#最大速度与加速度)。CLI 不强制最大值，超过后电机物理上跑不动。

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

夹爪跑满电机空载上限（1200°/s），且不放宽臂部保护：

```bash
rebot-vr-teleoperate \
  --gripper-max-speed-deg-s 1200 \
  --gripper-max-acceleration-deg-s2 20000 \
  --gripper-relative-target-deg 20
```

夹爪速度受两层串联钳制：`--gripper-max-speed-deg-s`（控制器整形与电机 FORCE_POS 速度限）和 `--gripper-relative-target-deg` × fps（follower 每周期相对裁剪）。相对目标小于 最大速度 ÷ fps（60 fps 下 1200°/s 需 ≥ 20）时，速度会被钳在 相对目标 × fps。`--gripper-relative-target-deg` 未设置时跟随 `--max-relative-target-deg`，也可单独设置；放宽夹爪不影响臂部保护。20000°/s² 加速到 1200°/s 需要 v²/2a = 36°，夹爪行程 270° 足够跑满。

> [!TIP]
> `--max-relative-target-deg` 应 ≥ 最大速度 °/s ÷ fps，否则会掐死实际速度；加速度建议从低值分档上调（10 → 20 → 40 → 60），每档观察状态行中的 `wrist_clip_deg` 与跳变冲击。

## 工作原理

```text
PICO 手柄位姿（XRoboToolkit V1 TCP / Isaac CloudXR）
  → XR → B601 坐标转换
  → Grip 按下时建立腕部中心位置 + 夹爪姿态参考（离合相对映射）
  → q1–q3 异步位置 IK（腕部中心）+ q4–q6 绝对闭式姿态解
  → 同一 VR 帧原子更新六轴目标
  → 限位、跳支保护、速度/加速度整形
  → LeRobot RebotB601Follower.send_action()
```

**线程模型**：V1 TCP 接收线程原子替换最新样本；IK worker 以 latest-only 方式求解（新请求到达时旧请求被丢弃）；主控制线程独占机器人反馈、状态机与唯一的 `send_action()` 调用。

**安全状态机**：

| 状态 | 含义 |
|---|---|
| `WAITING` | 尚未收到有效 Tracking |
| `IDLE` | Tracking 新鲜但 Grip 未激活；Trigger 仍可控制夹爪 |
| `ACTIVE` | 遥操激活 |
| `STALE` | 样本超时或断连：保持当前位置，要求重新释放 Grip |
| `HOLD` | 反馈缺失/非有限/超限：冻结最后有效命令；连续 5 帧后受控退出并保留扭矩 |

设计细节见 [CONTROL_DESIGN.md](docs/CONTROL_DESIGN.md) 与 [INVERSE_KINEMATICS_DESIGN.md](docs/INVERSE_KINEMATICS_DESIGN.md)。

## LeRobot 插件集成

包名符合 `lerobot_teleoperator_*` 约定，导入即把 `rebot_vr` 注册进 LeRobot 的遥操器 registry。但 LeRobot 0.6 的通用 `teleoperate/record` 循环**不会向 teleoperator 发送机器人反馈**，而本插件的笛卡尔控制闭环依赖反馈保证安全，因此该路径会**直接报错失败**（fail-closed），不会退化为开环关节控制。实机请一律使用 `rebot-vr-teleoperate`。


## 故障排除

| 现象 | 处理 |
|---|---|
| `command not found` | 激活 LeRobot 虚拟环境；或改用 `python -m lerobot_teleoperator_rebot_vr.teleoperate_real` |
| `Address already in use` | 关闭占用 63901 的其他进程（如仍在运行的 `rebot-vr-print`） |
| 按住 Grip 仍为 `idle` | 先完全松开 Grip 一次再重新按住 |
| `state=stale` | VR 数据流中断：检查 PICO 端发送与网络 |
| `state=hold` / 反馈异常 | 检查 CAN 连接与标定；连续 5 帧后保留扭矩受控退出 |
| 退出后机械臂下坠 | 退出前托住机械臂；默认断开时关闭扭矩，可用 `--no-disable-torque-on-disconnect` 保持使能 |
| `lerobot-teleoperate --teleop.type=rebot_vr` 报 feedback 错误 | 预期行为：LeRobot 通用循环不提供反馈，请改用 `rebot-vr-teleoperate` |
| 实际速度达不到设定值 | 相对目标被钳：臂部调大 `--max-relative-target-deg`、夹爪调大 `--gripper-relative-target-deg`（均应 ≥ 最大速度 °/s ÷ fps） |



## 文档

- [控制设计](docs/CONTROL_DESIGN.md) —— 线程模型、坐标映射、安全状态与反馈故障处理
- [逆解设计](docs/INVERSE_KINEMATICS_DESIGN.md) —— 从 VR 样本到六轴命令的完整推导
- [腕部求解验证报告](docs/vr_wrist_test/TEST_REPORT.md) —— 闭环仿真跳变统计与 q4–q6 解算对比

## 许可证

[Apache-2.0](LICENSE)

# PICO 4 VR 遥操作插件

[![Python](https://img.shields.io/badge/Python-3.12+-3776AB?logo=python&logoColor=white)](https://www.python.org/downloads/)
[![LeRobot](https://img.shields.io/badge/LeRobot-0.6.x-FFD21E?logo=huggingface&logoColor=white)](https://github.com/huggingface/lerobot)
[![License](https://img.shields.io/badge/License-Apache--2.0-3377FF)](LICENSE)

面向 [LeRobot](https://github.com/huggingface/lerobot) 0.6.x 与 Seeed Studio reBot B601-DM（达妙电机）的 PICO 4 手柄**笛卡尔遥操作**插件。
- **自适应 6-DoF QP IK** —— 默认以 `gripper_end` 跟踪完整位姿；接近奇异位形时连续增加阻尼并降低姿态权重，也可切换为纯位置 IK
- **离合式开关** —— Grip 按住激活、松开立即冻结；激活前必须先完全松开一次，防止手柄积累运动瞬间注入
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
> - [参数表](#参数)中的"最大值"列为实机验证的安全上限，CLI 不做强制校验，请勿超过。

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
| 1 | 仅位置（不跟踪姿态） | `--ik-mode position --position-scale 0.2 --max-joint-speed-rad-s 0.1` |
| 2 | 仅姿态 | `--position-scale 0 --orientation-scale 0.3 --max-joint-speed-rad-s 0.1` |
| 3 | 完整映射 | `--position-scale 1.0 --orientation-scale 1.0 --max-joint-speed-rad-s 0.4` |

### 4. CSV 桌面分析

遥操进程逐帧记录电机反馈、QP 目标、最终命令和 IK 诊断：

```bash
rebot-vr-teleoperate \
  --robot-port /dev/ttyACM0 \
  --backend xrobotoolkit_v1 \
  --csv-log logs/session.csv
```

实验结束后启动本地桌面分析器（不启动 Web 服务、不使用浏览器）：

```bash
rebot-vr-csv-analyze logs/session.csv
```

分析器可任意选择并叠加七轴 `actual/target/command`、actual-command/actual-target
误差、TCP 位置/姿态误差、`sigma_min`、condition number、QP 求解耗时和 `dq` 范数。
鼠标框选或滚轮可放大时间段，中键拖动可平移，十字光标可查看具体帧；底部表格显示
当前窗口内每条曲线的 min/max/mean/RMS、最大绝对值及其发生时刻。不同单位的曲线可启用
“逐曲线归一化”后比较变化趋势。

## 手柄按键

| 控制 | 行为 |
|---|---|
| Grip | 按住激活遥操，松开冻结并保持当前姿态（离合） |
| Trigger | 夹爪开合（`0` 张开 → `1` 闭合），不依赖 Grip；有效 Tracking 建立后立即生效 |
| A / X | 平滑返回起始姿态（右手 / 左手） |
| B / Y | 平滑返回六轴零点并闭合夹爪（右手 / 左手）；Trigger 再次主动移动后恢复开合控制 |

`--gripper-open-deg` 和 `--gripper-closed-deg` 是 Trigger 映射的两个端点。移动到 `initial_q` 期间夹爪保持实际反馈位置；进入 VR 主循环并取得新鲜 Tracking 后才应用 Trigger 映射。需要绕过 VR、直接验证夹爪标定和实际开合位置时，使用独立命令：

```bash
# 直接移动到开口测试点；q1-q6 持续保持实际反馈位置
rebot-gripper-test \
  --robot-port /dev/ttyACM0 \
  --target-deg -100 \
  --speed-deg-s 90 \
  --acceleration-deg-s2 360 \
  --relative-target-deg 10

# 直接回到标定闭合零点
rebot-gripper-test --robot-port /dev/ttyACM0 --target-deg 0
```

该工具不读取 PICO、Trigger 或 Grip。目标和反馈单位均为电机角度（deg），合法范围 `[-270, 0]`；默认退出后保留电机扭矩，避免机械臂失去支撑。确认已支撑机械臂后，才使用 `--disable-torque-on-disconnect`。

## 参数

`rebot-vr-teleoperate` 常用参数（完整参数：`rebot-vr-teleoperate --help`）：

| 参数 | 默认 | 最大值 | 说明 |
|---|---|---|---|
| `--position-scale` | `1.0` | — | 手柄位移 → `gripper_end` TCP 位移倍率 |
| `--orientation-scale` | `1.0` | — | 手柄旋转 → 末端旋转倍率 |
| `--max-joint-speed-rad-s` | `5.5` | **5.5** | q1–q6 前三轴速度约束（rad/s） |
| `--max-joint-acceleration-rad-s2` | `20` | **20** | q1–q6 前三轴加速度约束（rad/s²） |
| `--wrist-speed-rad-s` | `12` | **12** | q1–q6 后三轴速度约束（rad/s） |
| `--wrist-acceleration-rad-s2` | `60` | **60** | q1–q6 后三轴加速度约束（rad/s²） |
| `--max-relative-target-deg` | `20` | **20** | 臂部 follower 相对目标保护（deg） |
| `--wrist-relative-target-deg` | `20` | **20** | q4–q6 follower 相对目标保护（deg） |
| `--gripper-relative-target-deg` | `20` | **20** | 夹爪 follower 相对目标保护（deg） |
| `--gripper-open-deg` | `-180` | `>-270` | Trigger=0 的开口端点；负值绝对值越小，开口越小 |
| `--gripper-closed-deg` | `0` | `0` | Trigger=1 与 B/Y 回零的闭合端点 |
| `--gripper-max-speed-deg-s` | `1200` | **1200** | 夹爪速度上限（°/s） |
| `--gripper-max-acceleration-deg-s2` | `5000` | 推荐 ≤50000 | 夹爪加速度上限（°/s²） |
| `--gripper-torque-ratio` | `0.2` | `1.0` | 夹爪最大夹持力比例 |
| `--fps` | `70` |  ≤120 | 主循环频率 |
| `--qp-solver` | `scipy` | `scipy/osqp` | QP 后端；OSQP 需安装 `.[qp]` |
| `--ik-mode` | `pose` | `pose/position` | 完整位姿或纯 XYZ 任务 |
| `--qp-position-cost` | `20` | — | TCP 位置任务权重（高于姿态） |
| `--qp-orientation-cost` | `2` | — | 正常区域 TCP 姿态任务权重 |
| `--qp-orientation-cost-min` | `0.05` | — | 严重奇异区域姿态最低权重 |
| `--qp-position-gain` | `10` | — | 位置误差反馈增益（1/s），与目标线速度前馈相加 |
| `--qp-orientation-gain` | `8` | — | 姿态误差反馈增益（1/s），与目标角速度前馈相加 |
| `--arm-command-lookahead-ms` | `50` | — | q1-q3 POS_VEL 位置命令前视时间 |
| `--wrist-command-lookahead-ms` | `25` | — | q4-q6 POS_VEL 位置命令前视时间 |
| `--qp-damping-min` / `--qp-damping` | `1e-3` | — | 正常区域最小阻尼；旧参数名保留兼容 |
| `--qp-damping-max` | `0.1` | — | 严重奇异区域最大阻尼 |
| `--singularity-threshold` | `0.08` | — | 开始自适应的归一化 `sigma_min` |
| `--singularity-critical-threshold` | `0.02` | — | 达到最大保护的归一化 `sigma_min` |
| `--singularity-characteristic-length-m` | `0.3` | — | 6D Jacobian 线速度行的尺度归一化长度 |
| `--qp-smoothness-cost` | `0.05` | — | 速度连续性正则 |
| `--qp-posture-cost` | `0.05` | — | 回归 nominal 姿态正则 |
| `--joint-limit-margin-deg` | `2` | — | QP 关节限位内缩余量 |
| `--qp-max-solve-time-ms` | `8` | — | 单次 QP 时间预算 |
| `--csv-log` | 不记录 | — | 逐帧异步写入关节状态和 IK 诊断 CSV |

> [!IMPORTANT]
> 上表最大值为**实机验证的安全上限**；电机硬件空载上限见[最大速度与加速度](#最大速度与加速度)。CLI 不强制校验最大值，超出后电机物理上无法达到。

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

夹爪达到电机空载上限（1200°/s），且不放宽臂部保护：

```bash
rebot-vr-teleoperate \
  --gripper-max-speed-deg-s 1200 \
  --gripper-max-acceleration-deg-s2 20000 \
  --gripper-relative-target-deg 20
```

夹爪速度受两层串联钳制：`--gripper-max-speed-deg-s`（控制器整形与电机 FORCE_POS 速度限）和 `--gripper-relative-target-deg` × fps（follower 每周期相对裁剪）。相对目标小于 最大速度 ÷ fps（60 fps 下 1200°/s 需 ≥ 20）时，实际速度将被限制为 相对目标 × fps。`--gripper-relative-target-deg` 未设置时跟随 `--max-relative-target-deg`，也可单独设置；放宽夹爪不影响臂部保护。20000°/s² 加速到 1200°/s 需要 v²/2a = 36°，夹爪行程 270° 足以加速至满速。

> [!TIP]
> `--max-relative-target-deg` 应 ≥ 最大速度 °/s ÷ fps，否则会限制实际速度；加速度建议从低值分档上调（10 → 20 → 40 → 60），每档观察实际跟踪误差与跳变冲击。

## 工作原理

```text
PICO 手柄位姿（XRoboToolkit V1 TCP / Isaac CloudXR）
  → XR → B601 坐标转换
  → Grip 按下时建立 `gripper_end` TCP 位置/旋转参考，并原子锁存实际六轴姿态
  → Pinocchio LOCAL_WORLD_ALIGNED FK/Jacobian
  → 归一化任务 Jacobian SVD 奇异性监测
  → 目标 twist 前馈 + 误差反馈的自适应差分 QP IK
  → 同一 VR 帧原子更新六轴目标
  → ACTIVE 分轴 POS_VEL 前视 + 反馈窗口约束；回位路径速度/加速度整形
  → LeRobot RebotB601Follower.send_action()（机械臂 POS_VEL，夹爪默认 FORCE_POS）
```

实机入口将六个机械臂关节配置为达妙 `pos_vel` 模式。ACTIVE 中 QP 已直接约束关节速度
与加速度，控制器用 `q_actual + dq * lookahead` 生成分轴前视位置，不再经过会在短目标处
反复清零速度的第二个加速度整形器；命令仍受关节限位、controller command-feedback
窗口和 follower 相对目标三层保护。A/B 回位、启动姿态和夹爪继续使用位置整形。夹爪默认
使用 `force_pos`。当前不使用零扭矩前馈的 MIT 模式，因为负载下的稳态位置误差会妨碍
启动姿态和 TCP 跟踪。

**线程模型**：V1 TCP 接收线程原子替换最新样本；全六轴 QP worker 以 latest-only 方式求解（请求携带 generation/sequence/sample_id、实际反馈、上一速度、目标 twist 和 dt）；主控制线程独占机器人反馈、状态机与唯一的 `send_action()` 调用。Grip 捕获首帧不求解；成功结果的 `dq` 生成反馈基准上的有限前视位置，QP 失败时保持上一完整六轴目标。

状态行按 `--status-rate` 限频输出 `sigma_min`、condition number、当前 damping、当前 orientation weight、目标 twist、QP `dq`、求解时间、结果年龄、主循环实测 Hz、Tracking 样本年龄、反馈读取/命令发送/整帧工作耗时以及 TCP 实际/目标位置，不在每个控制周期刷日志。

**安全状态机**：

| 状态 | 含义 |
|---|---|
| `WAITING` | 尚未收到有效 Tracking |
| `IDLE` | Tracking 新鲜但 Grip 未激活；Trigger 仍可独立控制夹爪 |
| `ACTIVE` | 遥操激活 |
| `STALE` | 样本超时或断连：保持当前位置，要求重新释放 Grip |
| `HOLD` | 反馈缺失/非有限/超限：冻结最后有效命令；连续 5 帧后受控退出并保留扭矩 |

设计细节见 [CONTROL_DESIGN.md](assest/docs/CONTROL_DESIGN.md) 与 [INVERSE_KINEMATICS_DESIGN.md](assest/docs/INVERSE_KINEMATICS_DESIGN.md)。

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
| 实际速度达不到设定值 | 受相对目标限制：臂部增大 `--max-relative-target-deg`、夹爪增大 `--gripper-relative-target-deg`（均应 ≥ 最大速度 °/s ÷ fps） |



## 文档

- [控制设计](assest/docs/CONTROL_DESIGN.md) —— 线程模型、坐标映射、安全状态与反馈故障处理
- [逆解设计](assest/docs/INVERSE_KINEMATICS_DESIGN.md) —— 从 VR 样本到六轴命令的完整推导

## 许可证

[Apache-2.0](LICENSE)

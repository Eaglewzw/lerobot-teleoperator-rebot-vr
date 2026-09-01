# PICO 4 VR 遥操作插件

[![Python](https://img.shields.io/badge/Python-3.12+-3776AB?logo=python&logoColor=white)](https://www.python.org/downloads/)
[![LeRobot](https://img.shields.io/badge/LeRobot-0.6.x-FFD21E?logo=huggingface&logoColor=white)](https://github.com/huggingface/lerobot)
[![License](https://img.shields.io/badge/License-Apache--2.0-3377FF)](LICENSE)

面向 [LeRobot](https://github.com/huggingface/lerobot) 0.6.x 与 Seeed Studio reBot B601-DM（达妙电机）的 PICO 4 手柄**笛卡尔遥操作**插件。

- **自适应 6-DoF QP IK** —— TCP 位姿跟随手柄；接近奇异时自动增大阻尼、降低姿态权重，避免求解失败，也可切换纯位置模式
- **离合式激活** —— 按住 Grip 激活，松开即冻结；启动或中断后须先完全松开一次再激活，防止机械臂突跳
- **latest-only 线程模型** —— VR 接收、QP IK、主控制循环各一个线程，只消费最新数据，互不阻塞
- **分层安全保护** —— 速度/加速度整形、关节限位、相对目标钳制；反馈异常进入 HOLD 冻结，连续故障受控退出并保持扭矩
- **CSV 记录与分析** —— `--csv-log` 逐帧写入关节、IK 诊断和分段延迟，并在退出时生成延迟统计
- **双电机控制路径** —— 默认稳定的 `POS_VEL`，也可显式启用带关节速度目标与 Pinocchio 重力前馈的六轴 MIT；夹爪独立控制

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
  motorbridge-cli scan \
    --vendor damiao \
    --transport dm-serial \
    --serial-port /dev/serial/by-id/usb-HDSC_CDC_Device_00000000050C-if00 \
    --serial-baud 921600 \
    --start-id 1 \
    --end-id 7 \
    --feedback-base 0x10 \
    --timeout-ms 1000


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

### 4. MIT 实验模式

默认仍使用 `--motor-control-mode pos_vel`。MIT 只接管 q1-q6；第七个夹爪电机仍按
`--gripper-control-mode force_pos` 独立发送。首次测试必须托住机械臂，先关闭 VR 位移和
姿态映射，验证六个关节在多个位姿下的重力方向，再逐级提高速度与增益：

```bash
rebot-vr-teleoperate \
  --robot-port /dev/ttyACM0 \
  --backend xrobotoolkit_v1 \
  --motor-control-mode mit \
  --no-move-to-initial \
  --position-scale 0 \
  --orientation-scale 0 \
  --mit-kp 70 70 70 12 12 12 \
  --mit-kd 4 4 4 1 1 1 \
  --mit-torque-limit-nm 27 27 27 7 7 7 \
  --mit-gravity-scale 0.2 \
  --csv-log logs/mit-hold.csv
```

默认跟踪增益为前三轴 `Kp=70、Kd=4`，腕部三轴 `Kp=12、Kd=1`。参考工程的重力补偿
锁定控制器使用六轴 `Kp=8、Kd=1`，基础可拖动示例使用 `Kp=2、Kd=1`；两者适合柔顺保持，
但不足以作为带负载启动和 VR 目标跟踪的默认值。确认六轴补偿方向均正确后，
再按 `0.2 → 0.5 → 1.0` 提高重力倍率。MIT 命令为
`q_des, dq_des, Kp, Kd, tau_g`。`tau_g` 由打包的固定末端六轴动力学
URDF 计算并直接生效；默认力矩限幅等于 URDF effort 上限。该模型与参考控制工程使用同一
组惯性数据，`end_link` 质量为 `0.45 kg`，代表完整固定末端组件。若更换夹爪或负载，必须通过
`--mit-dynamics-urdf` 提供更新后的六轴惯性模型。`--mit-torque-limit-nm` 只限制前馈
扭矩项，不是电机内部 PD 总输出的硬扭矩限制。MIT 当前属于实机待标定功能，不应直接
使用高速参数开始测试。

### 5. CSV 记录与延迟分析


```bash
rebot-vr-teleoperate \
  --robot-port /dev/ttyACM0 \
  --backend xrobotoolkit_v1 \
  --csv-log logs/session.csv
```

运行结束后会得到两个文件：

- `logs/session.csv`：逐控制周期的关节、IK 诊断、原始单调时间戳和分段延迟。
- `logs/session_latency_summary.csv`：各延迟指标的样本数、均值、最小值、P50、P95、P99 和最大值。

主要延迟列的边界如下：

| CSV 列 | 测量范围 |
|---|---|
| `vr_decode_ms` | TCP 字节到达 → Tracking JSON 校验并发布到 latest-only 槽 |
| `latest_sample_wait_ms` | 样本发布 → 主控制循环读取该样本 |
| `tracking_receive_to_pickup_ms` | TCP 字节到达 → 主循环读取样本 |
| `feedback_read_ms` | 主循环调用 `robot.get_observation()` 的耗时 |
| `fk_ms` / `pose_mapping_ms` | Pinocchio FK / VR 相对位姿映射耗时 |
| `ik_sample_to_submit_ms` | 原始 Tracking 到达 → QP 请求提交 |
| `ik_queue_wait_ms` | QP 请求提交 → worker 开始求解 |
| `qp_solve_time_ms` / `ik_worker_total_ms` | 求解器内部耗时 / worker 完整求解耗时 |
| `ik_result_wait_ms` | worker 完成 → 主线程消费结果 |
| `command_shaping_ms` | 六轴与夹爪命令整形耗时 |
| `send_action_ms` | follower 限幅、反馈保护和七电机串口发送调用总耗时 |
| `ik_receive_to_send_ms` | 产生本次已消费 QP 结果的 Tracking 到达 → 串口发送返回 |
| `command_to_next_feedback_ms` | 上一周期串口发送返回 → 下一周期反馈读取完成 |

所有 PC 链路延迟均使用 `time.monotonic_ns()`，不会把 PICO 的 `timeStampNs`
当作 PC 时钟。当前达妙协议没有返回“电机收到命令”的硬件时间戳，因此
`send_action_ms` 的终点是 PC 串口写入调用返回；`command_to_next_feedback_ms`
是下一次可观测反馈的周期估计，不是电机侧确认时间。

MIT 模式下 CSV 还会写入六轴 `mit_desired_velocity_*_rad_s`、动力学模型原始
`mit_gravity_*_nm` 与实际限幅后的 `mit_feedforward_*_nm`，用于检查重力方向和饱和。


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

包源码按领域分为 `control/`、`ik/`、`vr/`、`runtime/`、`diagnostics/` 和 `tools/`。
控制周期编排位于 `control/controller.py`，Pinocchio/QP 位于 `ik/`，Tracking 与坐标映射
位于 `vr/`，CSV 记录分析位于 `diagnostics/`。包根目录只保留 LeRobot 插件配置、注册和
兼容入口；旧的 `cartesian_controller`、`kinematics` 等核心模块导入路径仍可使用。

## 许可证

[Apache-2.0](LICENSE)

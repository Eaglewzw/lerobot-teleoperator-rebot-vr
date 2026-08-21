# rebot-vr-teleoperate 参数说明

`rebot-vr-teleoperate` 常用参数如下；完整参数见 `rebot-vr-teleoperate --help`。

| 参数 | 默认 | 最大值 | 说明 |
|---|---|---|---|
| `--position-scale` | `1.0` | — | 手柄位移 → TCP 位移倍率 |
| `--orientation-scale` | `1.0` | — | 手柄旋转 → 末端旋转倍率 |
| `--max-joint-speed-rad-s` | `5.5` | **5.5** | q1–q3 速度约束（rad/s）；起始位姿移动时作用于全部六轴 |
| `--max-joint-acceleration-rad-s2` | `20` | **20** | q1–q3 加速度约束（rad/s²）；起始位姿移动时作用于全部六轴 |
| `--wrist-speed-rad-s` | `12` | **12** | q4–q6 速度约束（rad/s） |
| `--wrist-acceleration-rad-s2` | `60` | **60** | q4–q6 加速度约束（rad/s²） |
| `--max-relative-target-deg` | `20` | **20** | 臂部 follower 相对目标钳制（deg） |
| `--wrist-relative-target-deg` | `20` | **20** | q4–q6 follower 相对目标钳制（deg） |
| `--gripper-relative-target-deg` | 跟随臂部 | **20** | 夹爪 follower 相对目标钳制（deg）；缺省跟随 `--max-relative-target-deg` |
| `--gripper-open-deg` | `-180` | — | Trigger=0 的开口端点；CLI 强制 `-270 ≤ open < closed ≤ 0` |
| `--gripper-closed-deg` | `0` | — | Trigger=1 与 B/Y 回零的闭合端点 |
| `--gripper-max-speed-deg-s` | `1200` | **1200** | 夹爪速度上限（°/s） |
| `--gripper-max-acceleration-deg-s2` | `5000` | 推荐 ≤50000 | 夹爪加速度上限（°/s²） |
| `--gripper-torque-ratio` | `0.2` | `1.0` | FORCE_POS 最大夹持力比例；CLI 强制 [0, 1] |
| `--fps` | `70` | ≤120 | 主循环频率（Hz） |
| `--qp-solver` | `scipy` | `scipy/osqp` | QP 后端；OSQP 需安装 `.[qp]` |
| `--ik-mode` | `pose` | `pose/position` | 完整位姿或纯 XYZ 任务 |
| `--qp-position-cost` | `20` | — | TCP 位置任务权重 |
| `--qp-orientation-cost` | `2` | — | 正常区域 TCP 姿态任务权重 |
| `--qp-orientation-cost-min` | `0.05` | — | 严重奇异区域姿态权重下限 |
| `--qp-position-gain` | `10` | — | 位置误差反馈增益（1/s），与目标线速度前馈相加 |
| `--qp-orientation-gain` | `8` | — | 姿态误差反馈增益（1/s），与目标角速度前馈相加 |
| `--arm-command-lookahead-ms` | `50` | — | q1–q3 POS_VEL 位置命令前视时间 |
| `--wrist-command-lookahead-ms` | `25` | — | q4–q6 POS_VEL 位置命令前视时间 |
| `--qp-damping`（别名 `--qp-damping-min`） | `1e-3` | — | 正常区域最小阻尼 |
| `--qp-damping-max` | `0.1` | — | 严重奇异区域最大阻尼 |
| `--singularity-threshold` | `0.08` | — | 开始自适应的归一化 `sigma_min` |
| `--singularity-critical-threshold` | `0.02` | — | 达到最大保护的归一化 `sigma_min` |
| `--singularity-characteristic-length-m` | `0.3` | — | Jacobian 线速度行的尺度归一化长度（m） |
| `--qp-smoothness-cost` | `0.05` | — | 速度连续性正则 |
| `--qp-posture-cost` | `0.05` | — | 回归 nominal 姿态正则 |
| `--joint-limit-margin-deg` | `2` | — | QP 关节限位内缩余量（deg） |
| `--qp-max-solve-time-ms` | `8` | — | 单次 QP 时间预算；超预算结果被丢弃 |
| `--feedback-fault-max-consecutive` | `5` | — | HOLD 连续故障帧数达到后受控退出 |
| `--csv-log` | 不记录 | — | 逐帧异步写入关节状态与 IK 诊断 CSV |
| `--status-rate` | `5` | — | 状态行输出频率（Hz） |

> [!IMPORTANT]
> “最大值”列为**实机验证的安全上限**，电机硬件空载上限见[最大速度与加速度](#最大速度与加速度)。CLI 仅强制校验 `--gripper-torque-ratio`（[0, 1]）与夹爪端点（`-270 ≤ open < closed ≤ 0`），不校验速度、加速度、fps 等上限；超出后电机物理上无法达到。

## 最大速度与加速度

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

夹爪达到电机空载上限（1200°/s）且不放宽臂部保护：

```bash
rebot-vr-teleoperate \
  --gripper-max-speed-deg-s 1200 \
  --gripper-max-acceleration-deg-s2 20000 \
  --gripper-relative-target-deg 20
```

臂部与夹爪命令均依次经过三层钳制：QP/整形器速度与加速度约束 → controller 命令-反馈窗口（相对目标 × 0.9）→ follower 相对目标。相对目标对应等效速度上限 `相对目标 × fps`：默认 70 fps、20° 相对目标对应 1400°/s，满足 1200°/s。以 20000°/s² 加速到 1200°/s 需 36°，小于默认 180° 行程（−180° → 0°）。

> [!TIP]
> 相对目标须 ≥ 最大速度 ÷ fps，否则实际速度受限；加速度建议从低值分档上调（10 → 20 → 40 → 60），每档观察跟踪误差与跳变冲击。

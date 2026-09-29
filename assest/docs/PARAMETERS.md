# 运行参数

[架构](ARCHITECTURE.md) · [控制流程](CONTROL_DESIGN.md) · [QP](INVERSE_KINEMATICS_DESIGN.md)

默认值来自 [pos_vel_split.yaml](../../config/pos_vel_split.yaml) 和 [mit_split.yaml](../../config/mit_split.yaml)。IK 只有 split 一种（q1–q3 位置 QP + q4–q6 腕部闭式解）。核对日期：2026-09-29。

## 配置优先级

默认模式 pos_vel。CLI 显式值覆盖 YAML；Python 配置类不自动读取 YAML。完整选项见 `rebot-vr-teleoperate --help`。

```bash
# 自动加载对应模式的配置
rebot-vr-teleoperate --motor-control-mode pos_vel
rebot-vr-teleoperate --motor-control-mode mit

# 指定配置；文件内 motor_control_mode 必须与所选模式一致
rebot-vr-teleoperate --motor-control-mode mit --control-config config/mit_split.yaml
```

速度、加速度、相对目标和 fps 要求有限且大于零，没有固定校验上限。默认值不代表硬件或安全极限。

## 组合 profile 与参数归属

| 电机控制 | 配置（split IK） |
|---|---|
| POS_VEL | `pos_vel_split.yaml` |
| MIT | `mit_split.yaml` |

参数按职责分为三组：

- 电机层：`mit_kp`、`mit_kd`、前馈力矩上限和重力参数。
- 解算与映射层：滤波、死区、QP 增益/代价、奇异性参数。
- 安全与下发层：速度、加速度、命令-反馈窗口和前视。profile 可以给出更保守的值，但不能把它们当作硬件安全上限。

split profile 的初始策略是保持已经验证的 MIT 电机层参数，只降低或平滑 IK 目标：

| 参数 | POS_VEL split | MIT split | 目的 |
|---|---:|---:|---|
| 位置 / 姿态滤波 | 4 / 4 Hz | 4 / 4 Hz | 抑制 joint4 目标与腕姿抖动 |
| 位置 / 姿态死区 | 0.005 m / 0.5° | 0.008 m / 0.5° | 避免静止噪声持续驱动 |
| q1–q3 位置增益 | 6 | 4 | joint4 位置闭环 |
| q4–q6 腕部增益 | 4 | 2 | 相对腕姿闭环 |
| q1–q3 速度 / 加速度 | 3 / 10 | 2 / 6 | 保守起调值，rad/s 与 rad/s² |
| q4–q6 速度 / 加速度 | 4 / 15 | 2 / 6 | 保守起调值，rad/s 与 rad/s² |

这些是实机小幅动作的起点，不是最终标定结果。建议先固定 MIT 参数，依次调 q1–q3 位置回路、腕部增益/滤波、速度/加速度和前视；只有 CSV 中 IK 目标已经平滑，而反馈仍振荡或明显滞后时，才检查所用遥操 YAML 中各轴的 `mit_kp` / `mit_kd`，并在可靠支撑及低速条件下逐项验证。

## 位姿映射与运动限制

| 参数 | POS_VEL 默认 | MIT 默认 | 含义 |
|---|---|---|---|
| `--position-scale` | 1 | 1 | 手柄相对位移倍率 |
| `--orientation-scale` | 1 | 1 | 手柄相对旋转倍率 |
| `--position-filter-hz` | 0 | 4 | 位置低通截止频率，Hz；0 关闭 |
| `--orientation-filter-hz` | 0 | 0 | 姿态低通截止频率，Hz；0 关闭 |
| `--position-deadband-m` | 0 | 0.015 | 位置死区，m |
| `--orientation-deadband-deg` | 0 | 0 | 姿态死区，deg |
| `--max-joint-speed-rad-s` | 5.5 | 2 | q1–q3 速度限制，rad/s；启动移动用于全部六轴 |
| `--max-joint-acceleration-rad-s2` | 20 | 6 | q1–q3 加速度限制，rad/s²；启动移动用于全部六轴 |
| `--wrist-speed-rad-s` | 12 | 2 | q4–q6 速度限制，rad/s |
| `--wrist-acceleration-rad-s2` | 60 | 6 | q4–q6 加速度限制，rad/s² |
| `--max-relative-target-deg` | 20 | 10 | q1–q3 命令相对反馈的最大距离，deg |
| `--wrist-relative-target-deg` | 20 | 10 | q4–q6 命令相对反馈的最大距离，deg |
| `--arm-command-lookahead-ms` | 50 | 50 | ACTIVE 中 q1–q3 的位置前视时间，ms |
| `--wrist-command-lookahead-ms` | 25 | 25 | ACTIVE 中 q4–q6 的位置前视时间，ms |

腕部参数为 None 时回退到臂部值。前视对两种模式的 ACTIVE 有效，不用于启动/退出回零。控制器使用 0.9 倍反馈窗口；窗口 × fps 不能作为电机最大速度。

## QP 与奇异性

| 参数 | POS_VEL 默认 | MIT 默认 | 含义 |
|---|---|---|---|
| `--qp-solver` | scipy | scipy | scipy 或 osqp；OSQP 需安装 `.[qp]` |
| `--qp-position-cost` | 20 | 20 | 位置任务权重 |
| `--qp-position-gain` | 10 | 4 | 位置误差到目标线速度的增益，1/s |
| `--qp-orientation-gain` | 8 | 2 | q4–q6 腕部目标误差增益，1/s（名称沿用，split 的腕部增益） |
| `--qp-damping` | 0.001 | 0.001 | 最小阻尼；别名 `--qp-damping-min` |
| `--qp-damping-max` | 0.1 | 0.1 | 最大阻尼 |
| `--qp-smoothness-cost` | 0.05 | 0.05 | 相邻 QP 速度差的代价 |
| `--qp-posture-cost` | 0.01 | 0.01 | 偏离 nominal 关节姿态的代价 |
| `--singularity-threshold` | 0.08 | 0.08 | 开始自适应的归一化最小奇异值 |
| `--singularity-critical-threshold` | 0.02 | 0.02 | 达到最大阻尼的阈值 |
| `--singularity-characteristic-length-m` | 0.3 | 0.3 | Jacobian 线速度行归一化长度，m |
| `--joint-limit-margin-deg` | 2 | 2 | 关节限位内缩余量，deg |
| `--qp-max-solve-time-ms` | 8 | 8 | 求解时间预算，ms；超时结果不采用 |

要求 `0 ≤ critical < threshold`、`0 ≤ damping_min ≤ damping_max`。直接求解后检查耗时，超时丢弃。split 的 q1–q3 位置 QP 使用位置代价、位置增益、阻尼、平滑、关节姿态正则、奇异性阈值与限位余量；q4–q6 闭式分解使用 `qp_orientation_gain` 作为腕部误差增益。

## MIT 参数

六个数依次对应 q1–q6；夹爪独立配置。

| 参数 | 默认 | 代码校验 / 含义 |
|---|---|---|
| `--mit-kp` | 25 30 30 10 10 10 | 每轴 [0, 500]；位置误差增益 |
| `--mit-kd` | 5 5 4 0.5 0.5 0.5 | 每轴 [0, 5]；速度误差增益 |
| `--mit-torque-limit-nm` | 27 27 27 7 7 7 | 每轴大于 0，且不超过上述固定上限；仅限制前馈扭矩 |
| `--mit-gravity-scale` | 1 | [0, 2]；重力前馈倍率 |
| `--mit-gravity-ramp-s` | 1.5 | 非负；重力前馈渐入时间，s；0 立即应用 |
| `--mit-dynamics-urdf` | 内置模型 | 动力学 URDF 路径；不同于 IK 的 `--urdf` |

`mit_torque_limit_nm` 仅限制前馈，不限制电机 PD 与前馈的总扭矩。

## 启动与退出回零

| 参数 | POS_VEL 默认 | MIT 默认 | 含义 |
|---|---|---|---|
| `--initial-q` | 0 0.8 0.8 0 0 0 | 0 0.8 0.8 0.2 0 0 | RS 参考姿态，rad；下发前 q2/q3 取反 |
| `--move-to-initial` | 开启 | 开启 | `--no-move-to-initial` 跳过启动移动 |
| `--initial-move-tolerance-deg` | 2 | 3 | 六轴到位误差容差，deg |
| `--initial-move-timeout` | 30 | 30 | 启动/回零移动总超时，s |
| `--initial-stall-timeout` | 5 | 5 | 误差长时间未充分改善的超时，s |
| `--initial-feedback-request-hz` | 20 | 20 | 启动阶段每个电机主动请求反馈的频率上限；0 不限频 |
| `--return-to-zero-on-exit` | 开启 | 开启 | 正常 Ctrl+C 时尝试回零；第二次 Ctrl+C 中止 |
| `--exit-zero-speed-rad-s` | 0.5 | 0.5 | 退出回零速度上限，rad/s |
| `--exit-zero-acceleration-rad-s2` | 1 | 1 | 退出回零加速度上限，rad/s² |

启动六轴使用臂部速度/加速度；退出取退出值、臂部值、腕部值的最小值。MIT 两阶段均为位置整形、零目标速度、重力前馈。反馈请求限频不改变主循环频率；退出条件见 [控制流程](CONTROL_DESIGN.md)。

## 夹爪

| 参数 | 两种模式默认 | 含义 |
|---|---|---|
| `--gripper-control-mode` | force_pos | force_pos 或 mit，独立于六轴模式 |
| `--gripper-open-deg` | -180 | Trigger=0 的电机角度端点 |
| `--gripper-closed-deg` | 0 | Trigger=1 和 B/Y 闭合端点 |
| `--gripper-max-speed-deg-s` | 1200 | 速度限制，deg/s |
| `--gripper-max-acceleration-deg-s2` | 5000 | 加速度限制，deg/s² |
| `--gripper-relative-target-deg` | None | 回退到臂部相对目标窗口，deg |
| `--gripper-torque-ratio` | 0.2 | force_pos 扭矩比例，[0, 1] |
| `--gripper-mit-kp` | 8 | MIT 位置增益，[0, 500]；仅 MIT 使用 |
| `--gripper-mit-kd` | 0.3 | MIT 速度阻尼，[0, 5]；仅 MIT 使用 |

要求 `-270 ≤ open < closed ≤ 0`，单位是电机角度。启动保持反馈；主循环收到新鲜 Tracking 后按 Trigger 更新。`mit_split.yaml` 当前显式选择夹爪 MIT；该模式发送整形后的位置、零目标速度、配置的 `gripper_mit_kp/kd` 和零前馈力矩，`gripper_torque_ratio` 不生效。速度与加速度参数仍用于上位机位置目标整形。

## 安全、运行与记录

| 参数 | 两种模式默认 | 含义 |
|---|---|---|
| `--grip-press` / `--grip-release` | 0.60 / 0.40 | 激活/释放阈值；恢复后必须先释放 |
| `--stale-timeout` | 0.2 | PC 接收时间判断的 Tracking 超时，s |
| `--feedback-fault-max-consecutive` | 5 | 连续反馈异常达到该帧数时请求退出 |
| `--feedback-fault-settle-time` | 0.25 | 持续反馈故障退出前的保持阶段，s |
| `--disable-torque-on-disconnect` | 开启 | 普通退出尝试失能；持续反馈故障路径保留扭矩 |
| `--fps` | 90 | 主循环目标频率，Hz；不保证实测达到 |
| `--duration` | 0 | 主循环持续时间，s；0 持续运行 |
| `--status-rate` | 5 | 控制台状态输出频率，Hz |
| `--csv-log` | None | 主循环逐帧 CSV；相对路径以启动时工作目录为准 |

CSV 字段与限制见 [API](API_REFERENCE.md)。当前不支持 `--backend`、`--motor-diagnostics`。

# 控制流程

[架构](ARCHITECTURE.md) · [QP](INVERSE_KINEMATICS_DESIGN.md) · [参数](PARAMETERS.md)

## 关节与单位

TCP 为 URDF 的 `gripper_end`，包含夹爪安装偏置。

| 关节 | 电机名 |
|---|---|
| q1 | `shoulder_pan` |
| q2 | `shoulder_lift` |
| q3 | `elbow_flex` |
| q4 | `wrist_flex` |
| q5 | `wrist_yaw` |
| q6 | `wrist_roll` |
| 夹爪 | `gripper`，独立于六轴 IK |

内部臂部用 rad、rad/s、rad/s²，TCP 用 m，力矩用 N·m。LeRobot `.pos` 和夹爪整形用 deg。

## VR 映射

Tracking pose 为 `[x,y,z,qx,qy,qz,qw]`。默认坐标方向：XR +X → 基座 -Y，+Y → +Z，+Z → -X。

Grip 激活时锁存 VR 与实际任务点位姿。pose/position 的任务点是 TCP；split 的位置任务点是 joint4 轴心，姿态参考仍取 TCP。忽略滤波、死区时：

```text
p_target = p_tcp_ref + position_scale * R * (p_vr - p_vr_ref)
Delta_R = R * (R_vr * R_vr_ref.T) * R.T
R_target = Exp(orientation_scale * Log(Delta_R)) * R_tcp_ref
```

已转为基座坐标的 VRFrame 不再重复转换。滤波每样本执行一次：dt 优先取递增的源时间戳，否则取 PC 接收时间；新鲜度始终按 PC 接收时间判断。

## 状态与 Grip

| 状态 | 行为 |
|---|---|
| WAITING | 等待首次有效 Tracking |
| IDLE | Tracking 新鲜，Grip 未激活；可执行 A/B 回位 |
| ACTIVE | 相对位姿控制，提交/消费 QP |
| STALE | Tracking 缺失或超时，保持并清除参考 |
| HOLD | 反馈非法或越界，沿用已有保持命令 |

当前 YAML：Grip ≥ 0.60 激活，≤ 0.40 释放，中间区域保持原状态。启动、重连、Tracking 恢复和反馈故障恢复后必须先释放。

激活首帧同步实测关节角、命令和 nominal，清零速度历史；下一样本开始 QP。`IK=HOLD(reason)` 仅表示求解失败，控制状态仍可能是 ACTIVE。

## IK 与电机下发

IK 只有 split 一种：q1–q3 跟踪 joint4 轴心位置，q4–q6 跟随相对腕姿。

split 从 URDF FK 推导腕部轴序和符号，不硬编码欧拉轴交换。每个 VR 样本先完成 q1–q3 位置 QP 和 q4–q6 闭式分解，再原子发布六轴结果；腕部超限会裁剪并报告 `wrist_clip_deg`。该模式允许 TCP 随腕部长度产生位置圆弧，也允许肩部转动带动末端朝向。

ACTIVE 不重复执行通用位置整形。POS_VEL 由接受的 QP 速度生成前视位置；MIT 先限制最终速度变化、复核限位制动，再重建活动轴位置：

```text
q_des = q_actual + dq_des * lookahead
tau_ff = clip(gravity_scale * ramp * g(q_actual), ±torque_limit)
电机扭矩 = Kp*(q_des-q_actual) + Kd*(dq_des-dq_actual) + tau_ff
```

默认前视 q1–q3 为 50 ms、q4–q6 为 25 ms。`torque_limit` 只限制前馈项。制动钳制优先于普通加速度连续性，实机仍可能超调。

MIT 无新速度超过 `max(3/fps, 0.03)` 秒，或消费到失败结果时，减速到零；非 ACTIVE 或反馈故障立即清零目标速度。

## 回位与夹爪

新鲜 Tracking 下，夹爪独立于 Grip：

```text
gripper_goal = open_deg + Trigger * (closed_deg - open_deg)
```

默认 Trigger=0 为 -180°，Trigger=1 为 0°；角度含义依赖零点标定。

夹爪 `force_pos` 下发送位置、速度上限和 `gripper_torque_ratio`。夹爪
`mit` 下发送整形后的位置、零目标速度、`gripper_mit_kp/kd` 和零前馈
力矩；此时 `gripper_torque_ratio` 不使用，但夹爪速度/加速度仍限制上位机
位置目标的变化率。

| 操作 | 六轴 | 夹爪 |
|---|---|---|
| 启动 | 移动到 initial_q | 每帧保持反馈位置 |
| A/X 上升沿 | 回 initial_q | 不强制闭合 |
| B/Y 上升沿 | 回零，同帧优先于 A/X | 闭合；Trigger 变化 ≥ 0.05 后恢复接管 |
| 正常 Ctrl+C | 尝试回零 | 保持回零开始时的位置 |

启动与退出回零均用位置整形；MIT 目标速度为 0，保留重力前馈。启动使用臂部速度/加速度；退出取退出参数、臂部、腕部三者最小值。到位要求命令与反馈连续满足容差。

## 故障与退出

主循环反馈异常进入 HOLD；连续默认 5 帧后短暂保持，抛出 `PersistentFeedbackFault`，保留扭矩并断开通信。

启动/退出回零使用独立异常路径：反馈错误、越界、停滞或超时直接抛错。普通退出默认失能。

仅第一次正常 Ctrl+C 且无反馈故障时尝试回零；第二次 Ctrl+C 中止。时长结束、SIGTERM 和异常退出不触发该流程。回零失败仍继续断开。

失能会重试。底层缺少反馈接收时间戳/计数时，缓存 DISABLED 不能严格确认失能。

实现：[位姿映射](../../src/lerobot_teleoperator_rebot_vr/vr/pose_mapping.py)、[主控制器](../../src/lerobot_teleoperator_rebot_vr/control/controller.py)、[MIT](../../src/lerobot_teleoperator_rebot_vr/control/mit.py)、[启动/回零](../../src/lerobot_teleoperator_rebot_vr/runtime/safety.py)。

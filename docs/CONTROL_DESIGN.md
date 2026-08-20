# reBot B601-DM PICO 4 全六轴 TCP QP 控制设计

生产路径唯一使用 `gripper_end` TCP 的全六轴差分 QP IK。六个关节共同参与 TCP 位置和姿态跟踪；不存在腕部中心位置 IK、闭式腕部解或部分关节回退路径。

## 控制流程

```text
VR tracking sample
  -> XR 到机器人坐标变换
  -> Grip 激活时记录 p_tcp_ref/R_tcp_ref 与 VR 参考
  -> 相对离合映射生成目标 TCP 位姿
  -> 实际反馈 q_actual 的 gripper_end FK/Jacobian
  -> 六轴差分 QP 求解 dq
  -> 完整 q_next 原子应用
-> 有限值/限位/速度/加速度/follower 反馈保护
  -> send_action()
```

实机执行模式：`rebot-vr-teleoperate` 将六个机械臂关节配置为达妙 `pos_vel` 模式。
上层发送经过 QP 和速度/加速度整形的位置目标，LeRobot follower 同时发送分轴
`pos_vel_velocity` 速度上限；夹爪默认保持 `force_pos`。参数整定阶段不使用零扭矩
前馈的 MIT 模式，避免负载稳态误差影响启动姿态和 TCP 跟踪。

Grip 激活时记录 `p_tcp_ref`、`R_tcp_ref`、`p_vr_ref`、`R_vr_ref`。目标为：

```text
p_target = p_tcp_ref + position_scale * R_XR_TO_BASE * (p_vr - p_vr_ref)
R_target = Exp(orientation_scale * Log(R_XR_TO_BASE *
          (R_vr * R_vr_ref.T) * R_XR_TO_BASE.T)) * R_tcp_ref
```

## QP 与异步机制

每个请求携带 `generation`、`sequence`、`sample_id`、目标 TCP 位姿、`q_actual`、
`dq_previous`、`dt`、`q_nominal` 和提交时间。QP 变量是六轴关节速度 `dq`：

```text
min ||Wp(Jp*dq - ep/dt)||² + ||Wo(Jo*dq - eo/dt)||²
  + λd||dq||² + λs||dq-dq_previous||²
  + λq||q_actual + dt*dq-q_nominal||²
```

其中位置权重默认高于姿态权重，姿态误差为 `Log(R_target R_actual.T)`。约束包括：

```text
q_lower + margin <= q_actual + dt*dq <= q_upper - margin
-dq_max <= dq <= dq_max
-ddq_max*dt <= dq-dq_previous <= ddq_max*dt
```

目标是软约束，因此不可达目标会平滑饱和。QP 异常、超时、非有限结果、旧 generation、旧 sequence 或旧 sample_id 均不能应用；控制器保持上一完整六轴目标。

QP 在线性化时使用实际反馈，成功结果是绝对下一步目标
`q_next = q_actual + dq*dt`。不得再把 `dq*dt` 累加到上一完整目标，否则反馈延迟和
Tracking 噪声会形成目标积分器并驱使腕部持续运动。异步请求的 `dt` 使用相邻 QP 提交
间隔（上限 50 ms），避免 worker 周期低于控制循环时额外损失加速度。

每次 Grip 进入 ACTIVE 都把当前实际六轴角同时写入 `q_goal`、`q_command` 和
`q_nominal`，并把上一命令速度清零。捕获参考的首帧不提交 QP，从下一 Tracking 样本
开始求解。因此静止 Grip 时 TCP 误差和姿态正则误差同时为零，也不会继承 IDLE/home
阶段的旧命令或速度。

## 线程与状态

V1/Isaac 接收、latest-only QP worker 和主控制线程分离。主线程独占反馈读取、状态机和 `send_action()`。WAITING、IDLE、ACTIVE、STALE、HOLD 状态机、Grip 重新释放、Tracking 超时、A/B 回位和反馈故障保持原有安全语义。

## 安全层

QP 约束保证算法输出可执行；外层仍执行最终有限值检查、软件限位、速度/加速度整形、command-feedback 相对目标保护和 follower 发送。反馈异常进入 HOLD，禁止使用上一命令冒充实际反馈继续计算。

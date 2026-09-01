# reBot B601-DM PICO 4 全六轴 TCP QP 控制设计

生产路径使用 `gripper_end` TCP 的六轴差分 QP IK。默认 `pose` 模式共同跟踪 TCP
位置和姿态；`position` 模式只跟踪 TCP 位置。两者都以六个关节为变量，不存在腕部
中心位置 IK、闭式腕部解或部分关节回退路径。

## 控制流程

```text
VR tracking sample
  -> XR 到机器人坐标变换
  -> Grip 激活时记录 p_tcp_ref/R_tcp_ref 与 VR 参考
  -> 相对离合映射生成目标 TCP 位姿
  -> 实际反馈 q_actual 的 gripper_end FK/Jacobian
  -> 任务 Jacobian SVD 与连续奇异性自适应
  -> 目标 twist 前馈 + 误差反馈的六轴差分 QP 求解 dq
  -> ACTIVE 分轴前视位置原子应用
  -> 有限值/限位/command-feedback/follower 反馈保护
  -> send_action()
```

实机默认将六个机械臂关节配置为达妙 `pos_vel` 模式。显式设置
`--motor-control-mode mit` 后，插件发送六轴 `q_des/dq_des/Kp/Kd/tau_g`；`tau_g` 由
Pinocchio 读取独立的固定末端动力学 URDF 计算并经过倍率、可选渐入和分轴前馈限幅。默认
前三轴默认 `Kp=70、Kd=4`，腕部默认 `Kp=12、Kd=1`；重力倍率为 1、渐入为 0。参考工程
的 `Kp=8、Kd=1` 用于柔顺锁定，不作为带负载目标跟踪的默认值。默认前馈
限幅为 URDF effort `[27,27,27,7,7,7] N*m`。ACTIVE
中两种模式都使用 QP `dq` 生成的分轴前视位置目标；MIT 速度目标由最终限幅后的
`(q_command-q_actual)/lookahead` 得到。新 QP 结果到达时它等于该结果的 `dq`，没有新
结果时会随位置误差收敛而自然降到零，避免复用旧的非零速度。A/B 回位和非 ACTIVE 六轴
目标仍使用速度/加速度位置整形，MIT 速度目标在这些状态为零。

启动姿态只整形 q1-q6，夹爪命令原样跟随反馈；进入 VR 主循环取得新鲜 Tracking 后，
Trigger 目标才参与夹爪整形。第七个夹爪电机不进入六轴动力学模型，默认始终保持独立
`force_pos`。动力学 `end_link` 的惯性必须覆盖完整固定末端组件；更换夹爪或负载后需
更新模型。默认保持 `pos_vel`，MIT 必须在托住机械臂的条件下先验证重力力矩方向。

Grip 激活时记录 `p_tcp_ref`、`R_tcp_ref`、`p_vr_ref`、`R_vr_ref`。目标为：

```text
p_target = p_tcp_ref + position_scale * R_XR_TO_BASE * (p_vr - p_vr_ref)
R_target = Exp(orientation_scale * Log(R_XR_TO_BASE *
          (R_vr * R_vr_ref.T) * R_XR_TO_BASE.T)) * R_tcp_ref
```

## QP 与异步机制

每个请求携带 `generation`、`sequence`、`sample_id`、目标 TCP 位姿与 twist、`q_actual`、
`dq_previous`、`dt`、`q_nominal` 和提交时间。QP 变量是六轴关节速度 `dq`：

```text
vp* = v_target + Kp*ep
wo* = w_target + Ko*eo

min ||Wp(Jp*dq - vp*)||² + ||Wo(Jo*dq - wo*)||²
  + λd||dq||² + λs||dq-dq_previous||²
  + λq||q_actual + dt*dq-q_nominal||²
```

其中位置权重默认高于姿态权重，姿态误差为 `Log(R_target R_actual.T)`。`position`
模式不把姿态行加入 QP。约束包括：

```text
q_lower + margin <= q_actual + dt*dq <= q_upper - margin
-dq_max <= dq <= dq_max
-ddq_max*dt <= dq-dq_previous <= ddq_max*dt
```

目标 twist 使用相邻滤波目标的世界系速度前馈与当前误差比例反馈。目标是软约束，因此
不可达目标会平滑饱和。QP 异常、超时、非有限结果、旧 generation、旧 sequence 或旧
sample_id 均不能应用；控制器保持上一完整六轴目标。

## 奇异性自适应

FK/Jacobian 均来自 Pinocchio。QP 使用
`LOCAL_WORLD_ALIGNED` 的 `[J_linear_world; J_angular_world]`，与世界系位置误差及
`Log(R_target R_actual.T)` 一致。SVD 监测先用 0.30 m 特征长度归一化线速度行，避免
直接混合 m 与 rad：

```text
pose:     J_monitor = [J_linear / 0.30; J_angular]
position: J_monitor = J_linear / 0.30
```

`sigma_min >= 0.08` 时使用正常参数；`sigma_min <= 0.02` 时使用最大阻尼和最低姿态
权重；中间使用 smoothstep 连续插值。position cost 保持不变。condition number 只用于
可观测性，不参与硬切换。当前没有 manipulability task 或 Placo。

QP 在线性化时使用实际反馈，`q_next = q_actual + dq*dt` 用于求解约束与结果校验。
ACTIVE 实机目标为 `q_actual + dq*lookahead`，q1-q3 默认 50 ms，q4-q6 默认 25 ms，
再受关节限位与 command-feedback 窗口约束；不再经过第二个会在异步短目标处清零速度
的加速度整形器。异步请求的 `dt` 使用主机单调时钟
测得的相邻 QP 提交间隔并裁剪到 `[1e-6, 0.05]` s，避免 worker 周期低于控制循环时
额外损失加速度，也避免暂停后以异常大周期积分。

每次 Grip 进入 ACTIVE 都把当前实际六轴角同时写入 `q_goal`、`q_command` 和
`q_nominal`，并把上一命令速度清零。捕获参考的首帧不提交 QP，从下一 Tracking 样本
开始求解。因此静止 Grip 时 TCP 误差和姿态正则误差同时为零，也不会继承 IDLE/home
阶段的旧命令或速度。

## 线程与状态

V1/Isaac 接收、latest-only QP worker 和主控制线程分离。主线程独占反馈读取、状态机和 `send_action()`。WAITING、IDLE、ACTIVE、STALE、HOLD 状态机、Grip 重新释放、Tracking 超时、A/B 回位和反馈故障保持原有安全语义。

QP 结果携带 `sigma_min`、condition number、当前 damping/orientation weight、位置与
姿态残差、`dq`、求解时间和请求提交时间。主循环仅按 `--status-rate` 输出目标 twist、
结果年龄、实测循环 Hz、Tracking 样本年龄以及反馈读取/命令发送/整帧工作耗时，不在
60 Hz 控制路径逐帧打印。

代码职责与运行线程是两个不同层次。模块拆分如下：

```text
control/controller.py    单周期状态机编排、六轴命令与 action 合成
control/types.py         公共配置、状态和关节名称/限位常量
control/gripper.py       Trigger 重新接管、夹爪目标和单轴命令整形
control/feedback.py      反馈字段解析、有限值与反馈限位诊断
control/arm_command.py   ACTIVE 前视/非 ACTIVE 整形与反馈窗口约束
control/status.py        正常控制周期的状态和 IK 诊断快照组装
control/dynamics.py      六轴 Pinocchio 重力项计算与动力学模型结构校验
control/mit.py           插件内 MIT 发送、前馈/速度/相对目标限幅及独立夹爪发送
ik/coordination.py       generation、latest-only 请求、速度前馈和结果验收
ik/kinematics.py         Pinocchio FK/Jacobian 与 QP 数值求解
ik/async_worker.py       latest-only worker 线程与不可变请求/结果
vr/adapter.py            原始 VR action 到不可变 VRFrame 的适配
vr/tracking.py           不可变 Tracking 样本、校验与 latest-only 样本槽
vr/xr_v1.py              XRoboToolkit V1 TCP 流式解包和接收线程
runtime/                 实机 CLI、启动过程和反馈故障退出路径
diagnostics/             CSV 记录和离线数值分析
tools/                   夹爪与 VR 独立诊断命令
```

这些模块不会创建额外控制线程。`ik/coordination.py` 只由主线程调用，真正的异步求解仍
只发生在 `ik/async_worker.py` 的单个 worker 中；夹爪和反馈模块也都是主线程内的确定性逻辑。

## 安全层

QP 约束保证 ACTIVE 算法输出的速度与加速度可执行；外层仍执行最终有限值检查、软件
限位、command-feedback 相对目标保护和 follower 发送。非 ACTIVE 回位路径与 VR 主循环
中的夹爪继续执行速度/加速度整形；启动姿态阶段的夹爪仅保持反馈位置。反馈异常进入
HOLD，禁止使用上一命令冒充实际反馈继续计算。

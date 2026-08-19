# reBot B601-DM PICO 4 VR 控制设计

## 范围

本包是独立的 Python 包，不修改 LeRobot 源码。生产入口是
`rebot-vr-teleoperate`。控制方案固定为：

1. joint1~joint3 只执行 `joint4` 原点（前三轴末端/腕部基座中心）位置 IK。
2. joint4~joint6 根据同一个 VR 样本的目标旋转与完整 URDF 做绝对闭式解。
3. q123 只在位置 IK 成功时更新；同一请求的有效 q456 即使位置 IK 失败仍会更新，
   最终始终通过同一个完整六轴 `q_goal` 进入命令整形与发送。
4. 不包含 Placo、QP、速度 IK、姿态 IK、六轴 pose IK 或 IK 后端选择。

本包不包含 MuJoCo 适配层。实机命令通过 LeRobot
`RebotB601Follower.send_action()` 发送。

## 线程模型

| 执行上下文 | 所有权 | 交换数据 |
| --- | --- | --- |
| V1 TCP 接收线程 | server/client socket、流式解包器 | 向 `LatestSampleBuffer` 原子替换不可变 `ControllerSample` |
| IK worker | Pinocchio 线程局部 `Data`、latest-only IK 请求槽 | 发布不可变 `IKResult` |
| 主控制线程 | 机器人反馈、状态机、六轴 `q_goal/q_command`、唯一 `send_action()` 调用 | 读取最新样本和最新 IK 结果 |

网络端不保存 Tracking 历史。IK 正在计算 A 时收到 B、C，pending 槽最终只保留
C。主线程对每个 result sequence 最多消费一次。

## V1 数据边界

`V1TrackingSource` 监听 `0.0.0.0:63901`，使用 `SO_REUSEADDR`，一次只服务一个
TCP client。`PacketStreamDecoder` 支持断包、粘包、坏前缀、坏包尾、负长度和超过
1 MiB 的 body；遇到坏字节时向后搜索 `0x3F/0xCF`。

只有 command `0x6D` 且 `functionName == "Tracking"` 的包进入样本解析。第二层
`value` 可为 JSON 字符串或 object。无效 Tracking 增加计数，但不覆盖上一有效样本。

`ControllerSample` 保存原始 XR 数据，数组是只读副本：

- `position`: m
- `quaternion_xyzw`: `[qx, qy, qz, qw]`，单位范数
- `grip/trigger`: `[0, 1]`
- `received_monotonic_ns`: PC monotonic clock，仅此字段用于新鲜度
- `tracking_timestamp_ns`: 仅用于检测上游时钟回退，不用于计算延迟
- `stream_epoch`: TCP 重连或上游时钟回退时递增

## 坐标与相对映射

默认 XR 到机器人 World 的旋转为：

```text
R_WX = [[ 0, 0,-1],
        [-1, 0, 0],
        [ 0, 1, 0]]
```

因此 XR `+X/+Y/+Z` 分别对应 World `-Y/+Z/-X`。Grip 激活首帧同时记录手柄
参考位姿、实际 `joint4` 原点位置和实际 `gripper_end` 朝向。位置和姿态采用不同的
机器人参考量：

```text
p_wrist_target = p_wrist_ref + position_scale * R_WX * (p - p_ref)
R_delta_xr = R * R_ref.T
R_delta_world = R_WX * R_delta_xr * R_WX.T
R_target = Exp(orientation_scale * Log(R_delta_world)) * R_ee_ref
```

姿态计算只使用旋转矩阵和旋转向量，不对四元数分量或欧拉角做差，因此 `q` 与
`-q` 连续等价。位置与姿态死区、低通对每个 `(stream_epoch,
tracking_timestamp_ns, received_monotonic_ns)` 最多执行一次。

腕部目标使用已经完成比例、死区和滤波的 `R_target`。Pinocchio 从完整 URDF 求
`joint4` 在 q4=0 时的转动前坐标系朝向 `R_base(q123)`，初始化时由 FK 派生常量：

```text
K = R_base(0,0,0).T * R_ee(0,0,0,0,0,0)
N = R_base(q123).T * R_target * K.T
q5 = -asin(clip(N[2,0], -1, 1))
q4 = atan2(N[1,0], N[0,0])
q6 = atan2(N[2,1], N[2,2])
```

`K` 不包含硬编码腕部安装角，并用另一组非零 q123 在初始化时复算自检。闭式解使用
当前 `_q_goal_rad[:3]`，与位置 IK seed 一致；精确 q456 超限时才裁剪，并把最大超量
作为 `wrist_clip_deg` 上报。`|cos(q5)| < 1e-6` 时保持控制器提供的上一帧 q4，把可观测
的组合旋转分配给 q6。若未来把 q6 放宽到 ±180°，还需相对上一帧 q6 做 unwrap。

## 位置 IK 与同帧同步

Pinocchio 从打包的六轴 B601-DM URDF 加载完整模型。位置 IK 的控制点是 `joint4`
关节原点，即 q1~q3 运动链的末端；姿态闭式解仍以 `gripper_end` 为目标帧。每次请求
先把当前 VR 样本的 q456 写入完整 seed，再只迭代 q123：

```text
control_mode = "position"
active_joint_indices = (0, 1, 2)
J = J_joint4_LOCAL_WORLD_ALIGNED[0:3, 0:3]
dq = 0.5 * J.T * solve(J*J.T + adaptive_damping*I, position_error)
```

每步最大关节增量为 `0.2 rad`，最多进行 6 次 alpha 减半线搜索，只接受位置误差
下降的候选。`joint4` 原点只依赖 q1~q3，因此 q4~q6 转动以及从腕部到夹爪尖端的
工具长度不会引起前三轴补偿运动。warm start 只复用同 generation 的 q123；求解前后
都恢复当前请求的 q456。失败、非有限结果、错误形状或 q123 相对请求 seed 超过 `0.5 rad` 时保持上一
q1-q3 目标，但仍原子应用该请求已经闭式求出的 q4-q6。

请求携带 `sequence/generation/sample_id/target_position/q_seed/
submitted_monotonic_ns`。Grip 激活、失活、Tracking 中断和 A/B 回位都会开始新
generation 并清空 pending；旧求解可自然结束，但结果无法通过主线程的 generation
检查。

## 命令与单位

| 边界 | 单位 |
| --- | --- |
| XR position、IK position error | m |
| Pinocchio q、q goal、速度和加速度整形 | rad、rad/s、rad/s^2 |
| LeRobot B601 follower arm action | deg |
| LeRobot B601 follower gripper action | deg，生产 CLI 默认 `-180` 张开、`0` 闭合 |

完整六轴目标统一经过速度/加速度整形。控制循环的 `dt` 最大取 `0.05 s`；越过目标
时直接吸附并把该轴速度清零。Trigger 不依赖 Grip，仍只在 Tracking 新鲜时更新夹爪
目标。

生产入口 `rebot-vr-teleoperate` 的夹爪默认使用 `force_pos`，开/闭目标为
`-180/0 deg`，命令整形速度/加速度为 `2000 deg/s` 和 `4000 deg/s^2`，FORCE_POS
力矩比例为 `0.2`。这些是生产 CLI 默认值；注册式 `RebotVRTeleopConfig` 的字段默认值
用于 LeRobot 插件配置，不应与生产 runner 的 CLI 默认值混为一谈。

## 安全状态

- `WAITING`: 尚未收到有效 Tracking，不提交 IK。
- `IDLE`: Tracking 新鲜但 Grip 未激活；Trigger 可控制夹爪。
- `ACTIVE`: 已记录同一时刻的手柄和末端参考，可提交 latest-only IK 与绝对腕部解。
- `STALE`: 样本超时或断连，保持实际机械臂位置并要求重新释放 Grip。
- `HOLD`: 机器人反馈缺失、非有限或超限，冻结最后有效命令并清空 pending IK。

启动、重连、上游时钟回退、Tracking 恢复和 A/B 回位后，必须先看到
`grip <= 0.75`，之后 `grip >= 0.85` 才能激活。ACTIVE 中间区间保持激活；松开 Grip
立即结束 generation，未激活期间的手柄运动不会积累。

Primary Button 只响应按下沿（右手 A、左手 X），设置初始六轴目标并继续通过同一整形
器平滑返回，同时废弃 pending IK 并强制重新释放 Grip。

Secondary Button 同样只响应按下沿（右手 B、左手 Y），但目标为 B601-DM 六轴零点
`[0, 0, 0, 0, 0, 0] rad`。它复用相同的整形、generation 隔离和 Grip 重新释放规则；
若 A/B 在同一 Tracking 样本同时产生按下沿，零点目标优先。

## 反馈故障 HOLD

外层闭环控制器在 `WAITING/IDLE/ACTIVE/STALE` 之外提供机器人反馈故障 `HOLD`。反馈
缺字段、包含 NaN/Inf 或越过带容差的软件限位时，不再直接抛出异常：第一次故障会
清空 pending IK、递增 generation、冻结六轴和夹爪最后有效命令，并禁止处理新的 VR
目标。状态行报告连续故障帧数和具体原因。

反馈在阈值前恢复时，控制器从最新实际关节位置重新初始化目标、命令和速度，并要求
Grip 再次释放后才能接管。默认连续 `5` 帧异常才请求 runner 退出，可通过
`--feedback-fault-max-consecutive` 调整。退出前 runner 在默认 `0.25 s` 的收敛窗口内
用所有仍然有限的当前反馈替换保持命令；缺失关节继续使用最后有效命令。随后仅针对
该持久反馈故障退出路径保留电机扭矩，避免 `disable_torque_on_disconnect=True` 造成
突然下坠。正常 Ctrl-C/正常结束仍沿用用户配置的断开失能行为。

HOLD 命令已经是控制器最后确认的安全命令。发送 HOLD 和退出收敛命令时会临时关闭
follower 的相对反馈二次裁剪，防止 follower 内部再次读取丢包并把反馈回落为零后，
错误地把保持命令裁到零附近；绝对关节软件限位仍由 follower 正常执行。该临时设置
只包围单次 HOLD 发送，随后立即恢复。

## LeRobot 集成边界

`RebotVRTeleop` 仍注册为 `rebot_vr`，但它只在调用方每周期先调用
`send_feedback()` 时生成动作。LeRobot 0.6 的通用 B601 `teleoperate/record` 循环不会
向该 teleoperator 发送机器人反馈，因此该路径会失败关闭并提示使用
`rebot-vr-teleoperate`。这样不会用命令值假装实际关节值，也不会退回开环关节启发式。

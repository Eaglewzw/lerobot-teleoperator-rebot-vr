# B601-DM VR 逆解与六轴控制说明

本文说明 PICO 4/XRoboToolkit V1 样本如何变成 reBot B601-DM 的六个关节命令。
代码使用弧度进行运动学计算，发送给 LeRobot follower 时转换为角度。

## 1. 关节模型

控制器内部的六轴向量固定为：

```text
q = [q1, q2, q3, q4, q5, q6]
  = [shoulder_pan, shoulder_lift, elbow_flex,
     wrist_flex, wrist_yaw, wrist_roll]
```

其中 `q1~q3` 是位置 IK 的活动关节，`q4~q6` 是腕部姿态解的活动关节。
Pinocchio 使用打包的 `rebot_b601_dm_kinematics.urdf`。平移控制点是 `joint4` 原点
（q1~q3 运动链末端/腕部基座中心），姿态控制帧是 `gripper_end`。URDF 的关节顺序、
正方向和固定安装旋转是运动学的权威定义；当前
代码没有额外的固定 Pitch/Roll 轴置换或标定增益。

## 2. VR 样本到末端目标

`RelativePoseMapper` 在 Grip 激活瞬间记录：

```text
p_controller_ref, R_controller_ref
p_wrist_ref,      R_ee_ref
```

默认 XR 到机器人基座的旋转为：

```text
R_XB = [[ 0, 0,-1],
        [-1, 0, 0],
        [ 0, 1, 0]]
```

因此 XR 的 `+X/+Y/+Z` 分别映射到机器人基座的 `-Y/+Z/-X`。

### 位置

每个有效 Tracking 样本生成：

```text
p_wrist_target = p_wrist_ref
         + position_scale * R_XB
           * (p_controller - p_controller_ref)
```

位置单位是米。位置死区和一阶低通只对新样本执行一次；重复读取同一个样本不会再次
改变滤波结果。

### 姿态

手柄姿态使用四元数 `xyzw` 转为旋转矩阵，不能按四元数分量或欧拉角相减：

```text
R_delta_xr    = R_controller * R_controller_ref.T
R_delta_base  = R_XB * R_delta_xr * R_XB.T
R_target      = Exp(orientation_scale * Log(R_delta_base)) * R_ee_ref
```

姿态死区和一阶低通同样只对新样本执行一次。

`orientation_scale=0` 的含义是忽略手柄相对参考姿态的增量，使末端姿态目标保持为
`R_ee_ref`。这不是关闭腕部求解：当 `q1~q3` 变化时，`q4~q6` 仍可能变化，以维持
这个固定的末端姿态。

## 3. q1~q3：位置-only Pinocchio IK

控制器提交给异步 worker 的目标是腕部基座中心位置 `p_wrist_target` 和完整六轴种子 `q_seed`。
`q_seed[3:6]` 已经由本样本的腕部姿态解填入。

Pinocchio 每次迭代计算 `joint4` 关节原点和它的 `LOCAL_WORLD_ALIGNED` Jacobian，
只取线速度部分和前三个活动关节列：

```text
J = J_joint4[0:3, 0:3]
e = p_wrist_target - p_wrist_current

adaptive_damping = damping * max(1, 10 * ||e||)
dq = step_size * J.T
     * solve(J * J.T + adaptive_damping * I, e)
```

默认参数为：

```text
step_size = 0.5
max_iter  = 50
tolerance = 5e-4 m
damping   = 1e-4
```

每步 `dq` 的最大绝对值限制为 `0.2 rad`，最多进行六次 alpha 减半线搜索，只接受
能够降低位置误差的候选，并裁剪到关节限位。

这个 IK 的目标函数没有姿态误差项，也没有六轴 pose IK、Placo 或速度 QP。位置 IK
只允许修改：

```text
q1 = shoulder_pan
q2 = shoulder_lift
q3 = elbow_flex
```

`joint4` 原点严格只依赖 q1~q3。q4~q6 转动时，夹爪尖端可以围绕腕部移动，但不会
改变位置 IK 误差，也不会驱使 q1~q3 为夹爪长度或腕部转动做额外平移补偿。q456
仍随请求携带，是为了保证同一 VR 样本的六轴目标同步，而不是因为它参与位置目标。

## 4. q4~q6：闭式腕部姿态解

腕部不使用增量旋转向量的固定轴置换，而是直接根据 URDF 的旋转链求绝对关节角。
给定 q1~q3 和目标末端旋转，代码先计算 q4 转动之前的基座旋转：

```text
R_base(q123) = joint4 在 q4=q5=q6=0 时的朝向
```

常量腕部链由 FK 自动导出，而不是硬编码安装旋转：

```text
K = R_base(0,0,0).T * R_ee(0,0,0,0,0,0)
```

然后构造：

```text
N = R_base(q123).T * R_target * K.T
```

将 `N` 按标准内旋 ZYX 分解：

```text
q5 = -asin(clip(N[2,0], -1, 1))
q4 = atan2(N[1,0], N[0,0])
q6 = atan2(N[2,1], N[2,2])
```

当 `|cos(q5)| < 1e-6` 时，q4/q6 退化；代码保持上一帧 q4，把剩余旋转给 q6，避免
产生 NaN 或跳变。闭式结果先计算未裁剪的精确值，再分别裁剪到 q4~q6 软件限位，
并通过 `wrist_clip_deg` 上报超限幅度。

六个电机的目标来源如下：

| 电机 | 目标来源 | 是否参与位置 IK |
| --- | --- | --- |
| `shoulder_pan` (q1) | joint4 腕部中心位置 DLS | 是 |
| `shoulder_lift` (q2) | joint4 腕部中心位置 DLS | 是 |
| `elbow_flex` (q3) | joint4 腕部中心位置 DLS | 是 |
| `wrist_flex` (q4) | 闭式末端姿态解 | 否 |
| `wrist_yaw` (q5) | 闭式末端姿态解 | 否 |
| `wrist_roll` (q6) | 闭式末端姿态解 | 否 |

表中的电机名必须与 follower 和 URDF 的关节约定一致。若实机发现两个腕部电机的
物理接线/命名与 URDF 不一致，应在确认反馈和命令通道后统一修正硬件边界映射，不能
只交换求解结果而继续使用未交换的 FK，否则位置和姿态误差都会失去物理意义。

## 5. 同帧合并与异步 worker

每个请求携带：

```text
sequence, generation, sample_id, target_position,
完整 q_seed[6], submitted_monotonic_ns
```

提交前先把该 Tracking 样本的 q456 写入 `q_seed[3:6]`。worker 可以在同一 generation
内用上一次成功结果的 q1~q3 warm start，但求解前会重新覆盖 q456；求解后也会恢复该
请求的 q456，防止 warm start 或 Pinocchio 返回值污染非活动关节。

worker 是 latest-only：求解 A 时收到 B、C，A 完成后只处理 C。generation 在 Grip
激活/失活、Tracking 中断、回零时递增并清空 pending；迟到结果被主线程丢弃。

主线程每个 sequence 只消费一次：

```text
q_goal[3:6] <- 有限且形状正确的当前腕部结果（即使位置 IK 失败）
q_goal[0:3] <- 仅在 IK success、generation 当前且状态 ACTIVE 时更新
```

因此 q123 和 q456 最终作为同一个完整六轴目标进入下游，禁止提前单独发布腕部目标。
位置 IK 失败、非有限结果或 q1~q3 分支跳变超过 `0.5 rad` 时，q1~q3 保持上一目标，
而当前样本的有效 q456 仍可更新。

## 6. 六轴命令整形与 follower

`q_goal` 不直接发送。控制周期 `dt` 先裁剪到最大 `0.05 s`，六个关节统一使用：

```text
desired_velocity = clip((q_goal - q_command) / dt, +/-max_speed)
velocity_change  = clip(desired_velocity - current_velocity,
                         +/-max_acceleration * dt)
q_command        = q_command + (current_velocity + velocity_change) * dt
```

越过目标时吸附到目标并将该轴速度清零。随后还会用实际反馈限制命令与当前位置的
最大差值（默认由 `--max-relative-target-deg` 提供二次保护），最后按 follower 的
关节名称发送角度值。

所以状态行中：

```text
actual_deg  = 电机反馈
target_deg  = IK/腕部解的完整目标
command_deg = 速度、加速度和反馈差限制后的命令
sent        = follower 最终接受的命令
```

若 `target_deg` 已变化而 `actual_deg` 不跟随，问题在命令整形、follower、电机控制
模式、标定或机械侧，不在 IK 方程本身。

反馈缺失、非有限或越过安全限位时，控制器进入外层 `HOLD`，清空异步 IK 并重复最后
一个有效命令，不再计算新目标。恢复时从当前实际关节位置重新同步且要求 Grip 再次
释放；连续异常达到配置阈值后，runner 先将保持命令收敛到仍然有效的当前反馈再退出。
该持久反馈故障退出路径保留电机扭矩；正常退出仍按
`--disable-torque-on-disconnect` 的默认值关闭扭矩。

## 7. 典型参数语义

```text
--position-scale                  手柄平移到 joint4 腕部中心位置的倍率
--orientation-scale               手柄相对旋转到末端姿态的倍率
--max-joint-speed-rad-s           q1-q3 速度上限，腕部参数缺省时也用于 q4-q6
--wrist-speed-rad-s               可选 q4-q6 速度上限，缺省时回退臂部值
--max-joint-acceleration-rad-s2   q1-q3 加速度上限，腕部参数缺省时也用于 q4-q6
--wrist-acceleration-rad-s2       可选 q4-q6 加速度上限，缺省时回退臂部值
--ik-rate                         位置 IK worker 的最高求解频率，默认 100 Hz
--max-relative-target-deg         q1-q3 与夹爪 follower/反馈相对目标保护
--wrist-relative-target-deg       可选 q4-q6 相对目标保护，缺省时回退臂部值
--feedback-fault-max-consecutive  允许的连续反馈异常帧数，默认 5
--feedback-fault-settle-time      持久故障退出前收敛时间，默认 0.25 s
```

降低 `position-scale` 只会降低腕部中心平移目标变化，不会降低腕部姿态解的比例；降低
`orientation-scale` 会减小手柄旋转对末端姿态的影响，但不会把 q4~q6 从控制链中删除。

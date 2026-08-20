# B601-DM 全六轴 TCP 差分 QP IK

## 控制点

控制点固定为 URDF 中的 `gripper_end`。每个有效反馈周期使用实际六轴 `q_actual` 计算 TCP 位置、TCP 旋转和 Jacobian。默认 `pose` 模式中六个关节同时参与位置和姿态任务；`position` 模式仍使用六个关节，但只建立三维位置任务。

## 目标生成

Grip 激活时保存 VR 参考位姿和实际 TCP 参考位姿。平移使用坐标变换后的相对位置，旋转使用 `Log/Exp` 的 SO(3) 相对旋转，不使用四元数分量差或欧拉角差。

## QP

```text
e_p = p_target - p_actual
e_o = Log(R_target R_actual.T)
v_p* = v_target + Kp*e_p
w_o* = w_target + Ko*e_o

min ||Wp(Jp*dq-v_p*)||² + ||Wo(Jo*dq-w_o*)||²
  + λd||dq||² + λs||dq-dq_previous||²
  + λq||q_actual+dt*dq-q_nominal||²
```

`position` 模式删除第二项，而不是简单地把手柄 `orientation_scale` 设为零。后者仍会要求机械臂保持 Grip 激活瞬间的 TCP 姿态。

`v_target` 由相邻滤波后目标位置除以 PC `received_monotonic_ns` 间隔得到；`w_target`
使用世界系 `Log(R_target_new R_target_previous.T) / dt_sample`。Grip 捕获、epoch 变化或无效
时间间隔时前馈清零。误差反馈默认 `Kp=10 1/s`、`Ko=8 1/s`，因此持续匀速目标不再
只能依靠已经形成的位置误差追赶。

约束：

```text
q_lower + margin <= q_actual + dt*dq <= q_upper - margin
-dq_max <= dq <= dq_max
-ddq_max*dt <= dq-dq_previous <= ddq_max*dt
```

位置默认权重高于姿态。姿态任务为软目标，目标不可达时平滑饱和。`dq=0` 在反馈位于安全限位内时始终是可行保持解。

## Jacobian 与奇异性

Pinocchio 返回 `gripper_end` 的 `LOCAL_WORLD_ALIGNED` Jacobian：

```text
J = [J_linear_world; J_angular_world]
```

这与世界系 `p_target-p_actual` 和左乘误差
`Log(R_target R_actual.T)` 一致。由于原始 6D Jacobian 混合 m/s 与 rad/s，不能直接把
其奇异值当成无量纲指标。监测使用：

```text
pose:     J_monitor = [J_linear / characteristic_length; J_angular]
position: J_monitor =  J_linear / characteristic_length
```

默认 characteristic length 为 0.30 m。对打包 B601 URDF 的限位内离线采样显示，起始
姿态 `[0,-0.8,-0.8,0,0,0]` 的 pose `sigma_min≈0.276`；因此默认在 0.08 开始保护，
在 0.02 达到最大保护。condition number 仅用于诊断，自适应由 `sigma_min` 驱动。

## 连续自适应

令：

```text
x = clip((sigma_min - sigma_critical) /
         (sigma_threshold - sigma_critical), 0, 1)
h = x²(3-2x)
```

`h` 是端点一阶导数为零的 smoothstep。参数连续变化：

```text
damping = damping_max + h(damping_min-damping_max)
orientation_weight = orientation_min +
                     h(orientation_normal-orientation_min)
```

远离奇异位形时使用 `damping_min=1e-3`、orientation weight 2.0；严重接近奇异位形时
使用 `damping_max=0.1`、orientation weight 0.05，保持 position cost 20 不变。
`position` 模式的 orientation weight 恒为零。

求解器仍计算 `q_next = q_actual + dt*dq` 用于约束和结果校验。实机 ACTIVE 命令使用
反馈基准上的分轴 POS_VEL 前视：

```text
q_goal[0:3] = q_actual[0:3] + dq[0:3] * 0.050
q_goal[3:6] = q_actual[3:6] + dq[3:6] * 0.025
```

随后裁剪关节限位和 command-feedback 窗口。ACTIVE 不再把该短目标交给通用位置整形器
重复执行速度/加速度限制；QP 已约束 `dq` 与 `dq-dq_previous`，而 follower 的 POS_VEL
仍接收分轴速度上限。A/B 回位和 VR 主循环夹爪保持原有位置整形；启动姿态只整形
q1-q6，夹爪保持实际反馈位置。Grip 捕获首帧只同步
实际关节姿态、命令和 nominal 并清零历史速度，下一样本才提交 QP。

`dt` 来自主机单调时钟测得的相邻 QP 提交间隔，不使用 VR 上游时间戳；首个请求使用
主控制循环实际周期。进入请求前统一裁剪到 `[1e-6, 0.05]` s，避免零周期和长暂停破坏
速度/加速度约束。

## 请求和结果隔离

请求包含 `generation`、`sequence`、`sample_id`、目标 TCP 位姿与 twist、`q_actual`、
`dq_previous`、`dt`、`q_nominal` 和提交时间。结果必须同时匹配当前 generation、在途
sequence 和 sample_id，并通过有限值、求解时间和约束检查。失败或过期结果不会更新
任何关节。提交时间随结果返回，用于上报从提交到主线程实际采用结果的 `result_age_ms`。

## 参数

| 参数 | 默认值 |
|---|---:|
| `--qp-solver` | `scipy` |
| `--ik-mode` | `pose` |
| `--qp-position-cost` | `20` |
| `--qp-orientation-cost` | `2` |
| `--qp-orientation-cost-min` | `0.05` |
| `--qp-position-gain` | `10 1/s` |
| `--qp-orientation-gain` | `8 1/s` |
| `--arm-command-lookahead-ms` | `50 ms` |
| `--wrist-command-lookahead-ms` | `25 ms` |
| `--qp-damping-min`（兼容 `--qp-damping`） | `1e-3` |
| `--qp-damping-max` | `0.1` |
| `--singularity-threshold` | `0.08` |
| `--singularity-critical-threshold` | `0.02` |
| `--singularity-characteristic-length-m` | `0.3` |
| `--qp-smoothness-cost` | `0.05` |
| `--qp-posture-cost` | `0.01` |
| `--joint-limit-margin-deg` | `2` |
| `--qp-max-solve-time-ms` | `8` |

两种模式共用同一个反馈式 QP、安全约束、异步 worker 和完整六轴目标发布路径。当前没有
引入 manipulability task、Placo 或额外依赖。

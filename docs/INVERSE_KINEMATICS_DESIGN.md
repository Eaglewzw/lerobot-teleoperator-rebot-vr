# B601-DM 全六轴 TCP 差分 QP IK

## 控制点

控制点固定为 URDF 中的 `gripper_end`。每个有效反馈周期使用实际六轴 `q_actual` 计算 TCP 位置、TCP 旋转和完整 6x6 Jacobian。六个关节同时参与位置和姿态任务。

## 目标生成

Grip 激活时保存 VR 参考位姿和实际 TCP 参考位姿。平移使用坐标变换后的相对位置，旋转使用 `Log/Exp` 的 SO(3) 相对旋转，不使用四元数分量差或欧拉角差。

## QP

```text
e_p = p_target - p_actual
e_o = Log(R_target R_actual.T)

min ||Wp(Jp*dq-e_p/dt)||² + ||Wo(Jo*dq-e_o/dt)||²
  + λd||dq||² + λs||dq-dq_previous||²
  + λq||q_actual+dt*dq-q_nominal||²
```

约束：

```text
q_lower + margin <= q_actual + dt*dq <= q_upper - margin
-dq_max <= dq <= dq_max
-ddq_max*dt <= dq-dq_previous <= ddq_max*dt
```

位置默认权重高于姿态。姿态任务为软目标，奇异点使用阻尼和正则化，目标不可达时平滑饱和。`dq=0` 在反馈位于安全限位内时始终是可行保持解。

求解结果以 `q_next = q_actual + dt*dq` 作为绝对下一步六轴目标，不把该增量重复累加到
旧目标。Grip 捕获首帧只同步实际关节姿态、命令和 nominal 并清零历史速度，下一样本
才提交 QP，以避免反馈延迟形成目标积分和腕部漂移。

## 请求和结果隔离

请求包含 `generation`、`sequence`、`sample_id`、目标 TCP 位姿、`q_actual`、`dq_previous`、`dt`、`q_nominal` 和提交时间。结果必须同时匹配当前 generation、在途 sequence 和 sample_id，并通过有限值、求解时间和约束检查。失败或过期结果不会更新任何关节。

## 参数

| 参数 | 默认值 |
|---|---:|
| `--qp-solver` | `scipy` |
| `--qp-position-cost` | `20` |
| `--qp-orientation-cost` | `2` |
| `--qp-damping` | `1e-3` |
| `--qp-smoothness-cost` | `0.05` |
| `--qp-posture-cost` | `0.01` |
| `--joint-limit-margin-deg` | `2` |
| `--qp-max-solve-time-ms` | `8` |

生产路径没有其他 IK 模式。所有 VR 位姿目标均通过同一个全六轴 TCP QP 控制器生成完整六轴目标。

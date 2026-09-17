# QP 逆运动学

[控制流程](CONTROL_DESIGN.md) · [参数](PARAMETERS.md)

实现：[kinematics.py](../../src/lerobot_teleoperator_rebot_vr/ik/kinematics.py)、[coordination.py](../../src/lerobot_teleoperator_rebot_vr/ik/coordination.py)。

## 任务与误差

控制点为 `gripper_end`。pose 模式求解六轴；position 模式只允许 q1–q3 运动，腕部保持激活时目标。

Pinocchio Jacobian 使用 `LOCAL_WORLD_ALIGNED`：前三行为线速度，后三行为角速度。位置差和姿态误差均在基座/世界系表达：

```text
e_p = p_target - p_actual
e_o = Log(R_target * R_actual.T)
v* = v_target + Kp * e_p
w* = w_target + Ko * e_o
```

目标速度来自相邻目标位姿之差 / PC 接收时间间隔；捕获参考、epoch 改变或时间无效时清零。这里的 Kp/Ko 是任务增益，与电机 MIT Kp/Kd 不同。

## 目标函数

```text
min ||Wp*(Jp*dq-v*)||² + ||Wo*(Jo*dq-w*)||²
  + damping * ||dq||²
  + smoothness_cost * ||dq-dq_previous||²
  + posture_cost * ||q_actual+dt*dq-q_nominal||²
```

Wp/Wo 是任务 cost 的平方根。位置和姿态为加权软任务；position 模式删除姿态项，并硬约束 `dq[3:6]=0`。

## 硬约束

```text
safe_lower <= q_actual + dt*dq <= safe_upper
-speed <= dq <= speed
-acceleration*dt <= dq-dq_previous <= acceleration*dt

brake_upper = max(0, sqrt(2*acceleration*(safe_upper-q)) - acceleration*dt)
brake_lower = -max(0, sqrt(2*acceleration*(q-safe_lower)) - acceleration*dt)
brake_lower <= dq <= brake_upper
```

限位取 URDF 与 follower 的交集，再内缩 margin。反馈已进入余量区时，对应安全边界取当前反馈位置。

各约束取交集；交集为空返回 `infeasible_constraints`。即使位置未越界，加速度约束也可能不允许及时停车。降低姿态权重不能解决硬约束冲突。

`dt` 取相邻 QP 提交的 PC 单调时间间隔，首帧取控制周期，裁剪到 `[1e-6, 0.05]` s。

## 奇异性自适应

```text
pose:     J_monitor = [J_linear / L; J_angular]
position: J_monitor = J_linear[:, :3] / L

x = clip((sigma_min-critical)/(threshold-critical), 0, 1)
h = x²*(3-2*x)
damping = damping_max + h*(damping_min-damping_max)
orientation_weight = orientation_min + h*(orientation_normal-orientation_min)
```

默认 L=0.3 m，threshold=0.08，critical=0.02。接近奇异点时阻尼从 0.001 增至 0.1，姿态权重从 2 降至 0.05，位置权重保持 20。position 模式姿态权重恒为 0。condition number 仅用于诊断。

## 结果到命令

求解器计算 `q_next=q_actual+dq*dt` 做约束和校验。协调器每次只保留一个未消费请求，按样本去重；接受结果须成功、有限、未超求解预算，且 generation/sequence/sample_id 匹配。

POS_VEL 使用消费结果时的反馈生成 `q_goal=q_actual+dq*lookahead`。MIT 用最终加速度和制动限幅后的速度重建活动轴位置。两者再受关节限位与反馈窗口限制；position 模式不更新腕部目标。

匹配当前请求的失败结果会清零 QP 速度历史；MIT 发送层请求减速。ACTIVE 不重复通用位置整形。启动、回位和夹爪走各自位置轨迹，见 [控制流程](CONTROL_DESIGN.md)。

默认求解预算 8 ms。SciPy 返回后检查耗时，超时丢弃结果；这不是硬实时中断。全部默认值见 [参数表](PARAMETERS.md)。

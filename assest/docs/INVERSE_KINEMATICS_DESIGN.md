# 逆运动学

[控制流程](CONTROL_DESIGN.md) · [参数](PARAMETERS.md)

实现：[kinematics.py](../../src/lerobot_teleoperator_rebot_vr/ik/kinematics.py)、[split_solver.py](../../src/lerobot_teleoperator_rebot_vr/ik/split_solver.py)、[coordination.py](../../src/lerobot_teleoperator_rebot_vr/ik/coordination.py)。

## 任务与误差

split 是唯一的 IK 模式：q1–q3 的位置 QP 控制 joint4 轴心点，q4–q6 闭式跟随手柄相对旋转。joint4 的位置不受腕部关节影响，因此肩部整体旋转不会触发腕部的世界系姿态补偿。

Pinocchio Jacobian 使用 `LOCAL_WORLD_ALIGNED`：前三行为线速度，后三行为角速度。位置误差在基座/世界系表达：

```text
e_p = p_joint4_target - p_joint4_actual
v* = v_target + Kp * e_p
```

目标速度来自相邻目标位置之差 / PC 接收时间间隔；捕获参考、epoch 改变或时间无效时清零。这里的 Kp 是任务增益，与电机 MIT Kp/Kd 不同。

## 目标函数

q1–q3 的位置 QP：

```text
min ||Wp*(Jp[:3,:3]*dq[:3]-v*)||²
  + damping * ||dq||²
  + smoothness_cost * ||dq-dq_previous||²
  + posture_cost * ||q_actual+dt*dq-q_nominal||²
```

Wp 是位置任务 cost 的平方根。腕部速度被硬约束 `dq[3:6]=0`（腕部由闭式分解处理）。

## split 的腕部

```text
R_relative_target = R_joint4_ref.T * R_tcp_target
q456_target = WristDecompose(R_relative_target)
```

腕部的零位固定旋转、三轴顺序和符号由 URDF FK 自动推导。等价欧拉分支选择最接近上一目标者；奇异点保持上一 q4，再将剩余旋转分配给 q6。超限目标裁剪到安全范围并记录 `wrist_clip_deg`。

同一样本的 q123 与 q456 只在位置 QP 成功后合并成完整六轴结果。split 复用 QP 模式的 latest-only、generation 隔离、速度/加速度/限位制动和下发链路。几何上，腕部旋转会使 `gripper_end` 沿夹爪长度产生圆弧，这是模式语义，不是位置误差。

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

各约束取交集；交集为空返回 `infeasible_constraints`。即使位置未越界，加速度约束也可能不允许及时停车。

`dt` 取相邻 QP 提交的 PC 单调时间间隔，首帧取控制周期，裁剪到 `[1e-6, 0.05]` s。

## 奇异性自适应

```text
J_monitor = J_joint4_linear[:, :3] / L

x = clip((sigma_min-critical)/(threshold-critical), 0, 1)
h = x²*(3-2*x)
damping = damping_max + h*(damping_min-damping_max)
```

默认 L=0.3 m，threshold=0.08，critical=0.02。接近奇异点时阻尼从 0.001 增至 0.1，位置权重保持 20。condition number 仅用于诊断。

## 结果到命令

求解器计算 `q_next=q_actual+dq*dt` 做约束和校验。协调器每次只保留一个未消费请求，按样本去重；接受结果须成功、有限、未超求解预算，且 generation/sequence/sample_id 匹配。

POS_VEL 使用消费结果时的反馈生成 `q_goal=q_actual+dq*lookahead`。MIT 用最终加速度和制动限幅后的速度重建活动轴位置。两者再受关节限位与反馈窗口限制；split 原子更新六轴。

匹配当前请求的失败结果会清零 QP 速度历史；MIT 发送层请求减速。ACTIVE 不重复通用位置整形。启动、回位和夹爪走各自位置轨迹，见 [控制流程](CONTROL_DESIGN.md)。

默认求解预算 8 ms。直接求解后检查耗时，超时丢弃结果；这不是硬实时中断。全部默认值见 [参数表](PARAMETERS.md)。

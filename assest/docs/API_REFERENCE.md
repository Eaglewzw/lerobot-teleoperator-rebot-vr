# API 速查

[架构](ARCHITECTURE.md) · [控制流程](CONTROL_DESIGN.md) · [参数](PARAMETERS.md)

以下为简写签名，完整参数见源码。Python 配置类不自动加载运行 YAML。

## 单位与数据

| 数据 | 约定 |
|---|---|
| action/observation | 七个 `{joint}.pos`，单位 deg |
| IK / MIT 六轴 | rad、rad/s、rad/s² |
| TCP | 位置 (3,) m，旋转矩阵 (3,3) |
| ControllerSample | XR 坐标，四元数 xyzw；含接收时间、epoch、按键 |
| VRFrame | 已转为基座坐标的样本 |
| `*_monotonic_ns` | PC 单调时钟 |
| CSV `timestamp_ns` | PC 墙钟 |
| `tracking_timestamp_ns` | 上游时钟，不能直接与 PC 时间相减 |

ControllerSample、VRFrame 的数组复制后只读。关节顺序见 [控制流程](CONTROL_DESIGN.md)。

## LeRobot 插件

源码：[rebot_vr.py](../../src/lerobot_teleoperator_rebot_vr/rebot_vr.py)、[配置](../../src/lerobot_teleoperator_rebot_vr/config_rebot_vr.py)。

`RebotVRTeleopConfig` 注册为 `teleop.type=rebot_vr`。`RebotVRTeleop` 支持注入 VR controller 和 arm_controller。

| 方法 | 行为 |
|---|---|
| `connect(calibrate=True)` | 连接 VR，启动 QP |
| `send_feedback(feedback)` | 校验并覆盖七轴反馈槽 |
| `get_action()` | 消费一次反馈，返回 action；缺反馈报错 |
| `disconnect()` | 停止组件、释放自有资源 |

调用方负责 follower 连接、反馈读取和命令下发。

## VR

源码：[vr/](../../src/lerobot_teleoperator_rebot_vr/vr/)。

| 接口 | 返回 / 用途 |
|---|---|
| `parse_controller_sample(tracking, side, ...)` | ControllerSample；非法帧抛 TrackingSampleError |
| `LatestSampleBuffer.publish(sample)` | 本地发布序号 |
| `LatestSampleBuffer.latest()` | (sample 或 None, sequence) |
| `PacketStreamDecoder.feed(bytes)` | 完整包列表，处理碎包和粘包 |
| `PacketParser.unpack(bytes)` | 单个完整包或 None |
| `TrackingDecoder.decode_function(packet)` | (function_name, value) 或 None |
| `V1TrackingSource.start() / stop()` | TCP 生命周期 |
| `V1TrackingSource.latest_sample() / stats()` | 最新样本 / 接收统计 |
| `V1TrackingSource.feed_bytes(data, ...)` | 测试注入 |
| `sample_key(sample)` | (epoch, 源时间戳, PC 接收时间) |
| `RelativePoseMapper.update(sample, ee_position, ee_rotation, ...)` | PoseMappingUpdate |
| `RelativePoseMapper.reset(require_release=True)` | 清除参考，要求释放 Grip |

PoseTarget 包含 sample_id、position、rotation。映射与状态定义见 [控制流程](CONTROL_DESIGN.md)。

## 运动学与 QP

源码：[ik/](../../src/lerobot_teleoperator_rebot_vr/ik/)。

```text
B601Kinematics(urdf_path=None, end_effector_frame="gripper_end")
  forward_kinematics(q_rad) -> (position, rotation)
  tcp_jacobian(q_rad) -> (6, 6)
  tcp_pose_error(q_rad, target_position, target_rotation) -> (6,)
  close()

FullBodyQPIKSolver.solve(
  *, target_position, target_rotation, q_actual, dq_previous,
  dt, q_nominal, max_joint_speed, max_joint_acceleration,
  target_linear_velocity_m_s=None, target_angular_velocity_rad_s=None
) -> QPSolveResult
```

Pinocchio data 为线程局部对象；Jacobian 使用 LOCAL_WORLD_ALIGNED。QPSolveResult 含 q_next、dq、成功/原因、误差、耗时及奇异性诊断。

| 组件 | 调用 |
|---|---|
| LatestOnlyQPIKWorker | `start / submit / latest_result / clear / stop` |
| QPRequestCoordinator | `begin_generation / submit_if_ready / consume_latest` |

请求携带 generation、sequence、sample_id、目标位姿、反馈、速度历史、dt 和提交时间。通过 coordinator 消费结果，避免重复或跨代采用。数学定义见 [QP](INVERSE_KINEMATICS_DESIGN.md)。

## 控制器与 MIT

源码：[control/](../../src/lerobot_teleoperator_rebot_vr/control/)。

```text
FullBodyQPIKController(kinematics, *, xr_to_base_rotation,
                      config=None, ik_worker=None)
  start()
  update(frame, observation, dt_s, *, now_ns=None) -> (action, status)
  stop()
```

frame 可为 ControllerSample、VRFrame 或 None。反馈异常返回 HOLD；没有历史命令时 action 可为 None。`status.feedback_abort_requested` 表示持续故障，退出由 runner 处理。

| 接口 | 用途 |
|---|---|
| `shape_joint_position_command(...)` | 返回位置、速度；单位随输入 |
| `bound_position_command_to_feedback(...)` | 限制命令与反馈距离 |
| `StartupPoseMover.update(actual_rad, dt_s)` | 启动/回零位置、误差、到位状态 |
| `B601GravityCompensator.gravity_torque(q_rad)` | 六轴重力项，N·m |

```text
MITCommandDispatcher(robot, *, kp, kd, torque_limit_nm,
  arm_velocity_limit_rad_s, arm_acceleration_limit_rad_s2=None,
  position_lookahead_s=None, velocity_aligned_axes=None,
  joint_limit_margin_rad=0, gravity_scale=1, gravity_ramp_s=1,
  dynamics_urdf=None)
```

每周期先同步 observation，再调用 `set_arm_velocity()`、`send_action()`。停止接口为 `stop_arm_velocity(immediate=...)` 和 `stop_stale_arm_velocity(max_age_s)`。

正式 runner 显式配置加速度、制动和前视；None 不代表启用。类默认重力渐入 1 s，运行 YAML 为 1.5 s。前馈限幅不限制 PD 总扭矩。

## Runtime

源码：[runtime/](../../src/lerobot_teleoperator_rebot_vr/runtime/)。

| 接口 | 用途 |
|---|---|
| `build_parser().parse_args()` | 加载 YAML，再应用 CLI |
| `validate_args(args)` | 参数检查 |
| `follower_pos_vel_velocity(arm_speed_rad_s, wrist_speed_rad_s, gripper_speed_deg_s)` | 七轴 deg/s 列表 |
| `follower_relative_target(arm_relative_target_deg, wrist_relative_target_deg, gripper_relative_target_deg=None)` | 相同值返回标量，否则按电机名字典 |
| `move_to_initial_pose(...) / move_to_zero_pose(...)` | 位置轨迹与超时检查 |
| `settle_persistent_feedback_fault(...)` | 故障退出前保持 |
| `status_line(status)` | 控制台摘要 |

main 顺序：配置 → 创建组件 → 连接 VR → 连接电机 → 启动移动 → 主循环 → 清理。VR 监听失败发生在连接电机前。

## CSV 与分析

源码：[diagnostics/](../../src/lerobot_teleoperator_rebot_vr/diagnostics/)。

```text
build_csv_row(status) -> dict
CSVLogger(output_path)
  write_row(row)    # 入队，后台写盘
  close()          # 排空、关闭、生成延迟汇总

TelemetryDataset.load(path) -> dataset
dataset.statistics(signal_key, start_s, end_s) -> SignalStatistics 或 None
dataset.range_indices(start_s, end_s) -> (start, end)
decimate_minmax(x, y, max_points) -> (x_reduced, y_reduced)
```

| CSV 数据 | 含义 |
|---|---|
| actual / target / command | 七轴 × 三组，deg |
| position_error_m / orientation_error_deg | 实测 TCP 与目标误差 |
| sigma_min / condition_number | 奇异性指标 |
| qp_solve_time_ms / dq_norm_rad_s | 求解耗时 / 速度范数 |
| MIT 字段 | 目标速度、重力和前馈扭矩 |
| 延迟汇总 samples | 该指标有效记录数；p50/p95/p99 为分位数 |

仅记录主循环，不含启动/退出回零。command 是控制器位置，可能与 MIT 最终下发值不同；没有 MPJPE、完整 TCP 位姿或电机接收确认。不同样本集合的分位数不能直接相加作总延迟。

当前提供分析 API，无 CSV 图形命令。MIT 调参使用独立日志，见 [调参文档](MIT_TUNING.md)。

## 最小集成示例

只验证控制器调用，不连接电机：

```python
import numpy as np
from lerobot_teleoperator_rebot_vr import B601Kinematics, FullBodyQPIKController

kinematics = B601Kinematics()
controller = FullBodyQPIKController(
    kinematics,
    xr_to_base_rotation=np.array([[0., 0., -1.], [-1., 0., 0.], [0., 1., 0.]]),
)
observation = {f"{name}.pos": 0.0 for name in (
    "shoulder_pan", "shoulder_lift", "elbow_flex",
    "wrist_flex", "wrist_yaw", "wrist_roll", "gripper",
)}
controller.start()
try:
    action, status = controller.update(None, observation, dt_s=1 / 90)
finally:
    controller.stop()
    kinematics.close()
```

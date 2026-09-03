# 核心模块 API 参考

本文只记录当前代码中对集成、测试或模块协作有意义的接口。以下 action/observation 的关节位置单位均为 **度**；IK 和运动学内部关节角单位均为 **弧度**；位置单位为 **米**，时间戳为 `time.monotonic_ns()` 纳秒值。

## 1. 包级公共接口

包根 `lerobot_teleoperator_rebot_vr.__init__` 正式导出：

```python
B601Kinematics
CartesianControlConfig
ControllerSample
LatestSampleBuffer
PacketParser
PacketStreamDecoder
Pico4VRController
RebotVRConfig
RebotVRTeleop
RebotVRTeleopConfig
FullBodyQPIKController
TrackingDecoder
TrackingSampleError
VRFrame
V1TrackingSource
XRoboToolkitV1Controller
parse_controller_sample
```

`__init__.py` 还通过 `sys.modules` 保留 0.4 之前的旧平铺模块路径。新代码应使用领域路径，例如 `lerobot_teleoperator_rebot_vr.ik.kinematics`，不要继续依赖旧别名。

## 2. 配置与 LeRobot 集成

### `RebotVRConfig`

```python
RebotVRConfig(*, vr_backend="xrobotoolkit_v1", hand_side="right", ...)
```

集中保存 VR 地址、坐标变换、Grip 阈值、映射滤波、QP 权重、奇异性处理、关节/夹爪限制等参数。`__post_init__()` 会验证枚举、数值范围、六维初始姿态及 `base_T_anchor` 的刚体变换合法性。

相关常量：

- `REBOT_JOINTS`：六个手臂关节加 `gripper`。
- `ARM_JOINTS`：`REBOT_JOINTS[:6]`。
- `DEFAULT_BASE_T_ANCHOR`：OpenXR 坐标到 reBot 基座坐标的 4×4 变换。

### `RebotVRTeleopConfig`

```python
@TeleoperatorConfig.register_subclass("rebot_vr")
class RebotVRTeleopConfig(TeleoperatorConfig, RebotVRConfig): ...
```

用于 LeRobot 配置发现，无新增字段。

### `RebotVRTeleop`

```python
RebotVRTeleop(
    config: RebotVRTeleopConfig,
    controller: VRController | None = None,
    arm_controller: FullBodyQPIKController | None = None,
)
```

| API | 返回值 | 契约 |
|---|---|---|
| `connect(calibrate=True)` | `None` | 创建缺失组件，连接 VR 并启动 QP worker；重复连接报错 |
| `send_feedback(feedback)` | `None` | 校验并覆盖七轴反馈单槽；必须全字段且有限 |
| `get_action()` | `RobotAction` | 单次消费反馈，读取最新 VR 样本并运行控制器；无反馈/未连接报错 |
| `disconnect()` | `None` | 停止控制器和 VR，关闭内部创建的运动学资源 |
| `action_features` | `dict[str, type]` | 七个 `{joint}.pos: float` |
| `feedback_features` | `dict[str, type]` | 同上 |
| `is_connected` | `bool` | 生命周期状态 |
| `is_calibrated` | `bool` | 当前固定为 `True` |

构造器中的 `controller` 和 `arm_controller` 是测试/替换实现的注入点。

## 3. VR 模块

### 数据模型

`ControllerSample` 是 XR 原始样本，包含：接收/发布单调时钟、Tracking 源时间戳、`stream_epoch`、左右手、位置 `(3,)`、四元数 xyzw `(4,)`、Grip、Trigger、主/次按钮和 Tracking 状态。

`VRFrame` 是已经转换到机器人基座坐标的兼容样本，主要字段为 `grip_pos`、`grip_quat`、`squeeze`、`trigger`、`is_tracking`、时间戳链、按钮和可选头显位姿。

两者都是 frozen dataclass；数组会复制并设为只读。`RelativePoseMapper` 和主控制器均接受二者。

### Tracking 解析与缓冲

```python
normalize_controller_side(side: str) -> Literal["left", "right"]

parse_controller_sample(
    tracking: Mapping[str, Any],
    side: str,
    *,
    received_monotonic_ns: int | None = None,
    stream_epoch: int = 0,
) -> ControllerSample

LatestSampleBuffer.publish(sample: ControllerSample) -> int
LatestSampleBuffer.latest() -> tuple[ControllerSample | None, int]
LatestSampleBuffer.clear() -> None
```

解析失败抛 `TrackingSampleError`。buffer 的返回 sequence 是本地发布序号，不等同于 Tracking 时间戳。

### XRoboToolkit V1 协议

```python
PacketParser.unpack(data: bytes) -> dict[str, object] | None
PacketParser.pack(command: int, body: str | bytes) -> bytes

PacketStreamDecoder(on_warning=None)
    .feed(data: bytes) -> list[dict[str, object]]
    .reset() -> None
    .buffered_bytes -> int

TrackingDecoder.decode_function(packet) -> dict | None
TrackingDecoder.decode_tracking(packet) -> dict | None
TrackingDecoder.decode_json_object(value) -> dict | None
```

`PacketParser.unpack()` 只处理一个完整包；流式 TCP 数据必须交给 `PacketStreamDecoder.feed()`。

```python
TcpReceiver(host, port, on_bytes, *, on_listen=None,
            on_connect=None, on_disconnect=None, on_error=None)
    .serve_forever() -> None
    .stop() -> None
    .running -> bool

V1TrackingSource(host="0.0.0.0", port=63901, *, side="right",
                 on_status=None, on_sample=None)
    .start(timeout=3.0) -> None
    .stop() -> None
    .latest_sample() -> ControllerSample | None
    .latest() -> tuple[ControllerSample | None, int]
    .stats() -> TrackingSourceStats
    .feed_bytes(data, *, received_monotonic_ns=None) -> None
    .running / .connected -> bool
```

`feed_bytes()` 是无需真实 socket 的测试注入点。源 Tracking 时间戳回退会增加 `stream_epoch` 并清空旧样本。

### VR 后端抽象

```python
class VRController(Protocol):
    is_connected: bool
    is_tracking: bool
    def connect(self) -> None: ...
    def get_action(self) -> dict[str, Any]: ...
    def latest_sample(self) -> ControllerSample | None: ...
    def disconnect(self) -> None: ...

make_vr_controller(config: RebotVRConfig) -> VRController
```

- `xrobotoolkit_v1` 返回 `XRoboToolkitV1Controller`，使用本地 TCP server。
- `isaac` 返回 `Pico4VRController`，依赖可选 Isaac Teleop/CloudXR 包；其 `latest_sample()` 返回 `None`，通过 `get_action()` 提供 `VRFrame` 形状数据。

`XRoboToolkitV1Controller` 额外公开 `feed_bytes()` 和 `stats()`，便于测试与诊断。

### 统一适配函数

```python
sample_is_fresh(sample, now_ns: int, timeout_s: float) -> bool
sample_key(sample) -> tuple[int, int, int] | None
trigger_value(sample) -> float
vr_frame_from_raw_action(action: Mapping[str, object]) -> VRFrame
```

`sample_key()` 的三元组为 stream epoch、Tracking 时间戳和接收单调时间，用于去重和隔离重连后的样本。

### 位姿映射

```python
class TeleopState(str, Enum):
    WAITING = "waiting"
    IDLE = "idle"
    ACTIVE = "active"
    STALE = "stale"
    HOLD = "hold"

RelativePoseMapper(...)
    .update(sample, ee_position, ee_rotation, *, now_ns=None) -> PoseMappingUpdate
    .reset(*, require_release=True) -> None
    .require_release -> bool
```

`PoseMappingUpdate` 返回 `state`、可选 `PoseTarget`、是否刚捕获参考、是否要求松 Grip 及姿态诊断。`PoseTarget` 含 `sample_id`、位置 `(3,)` 和旋转矩阵 `(3,3)`。

映射器实现相对控制：进入 ACTIVE 时同时记录 VR 参考和机器人 TCP 参考，之后只应用 VR 的相对平移/旋转，因此不会把绝对 VR 世界坐标直接当作机器人目标。

## 4. IK 模块

### `B601Kinematics`

```python
B601Kinematics(urdf_path: str | Path | None = None,
               end_effector_frame: str = "gripper_end")
    .forward_kinematics(q_rad: array) -> tuple[position, rotation]
    .tcp_jacobian(q_rad: array) -> ndarray       # 6×6
    .tcp_pose_error(q_rad, target_position, target_rotation) -> ndarray  # (6,)
    .lower_position_limit -> ndarray             # (6,)
    .upper_position_limit -> ndarray             # (6,)
    .close() -> None

default_urdf_path() -> Path
```

默认读取包内 `rebot_b601_dm_kinematics.urdf`。实例内部保护 Pinocchio data，且对临时资源使用显式 `close()` 生命周期。

### `FullBodyQPIKSolver`

```python
FullBodyQPIKSolver(
    kinematics: B601Kinematics,
    *,
    solver="scipy",
    ik_mode="pose",
    ...,
)

.solve(
    *, target_position, target_rotation,
    q_actual, dq_previous, dt, q_nominal,
    max_joint_speed, max_joint_acceleration,
    target_linear_velocity_m_s=None,
    target_angular_velocity_rad_s=None,
) -> QPSolveResult
```

`QPSolveResult` 给出下一关节目标、成功标志、位置/姿态误差、求解耗时、原因、奇异值/条件数、自适应阻尼与姿态权重以及关节速度。

### 异步请求与结果

```python
IKRequest(*, sequence, generation, sample_id,
          target_position, target_rotation, q_seed,
          q_actual=None, dq_previous=None, q_nominal=None,
          target_linear_velocity_m_s=None,
          target_angular_velocity_rad_s=None,
          dt=0.01, submitted_monotonic_ns=None,
          sample_received_monotonic_ns=0)

IKResult(generation, sequence, sample_id, q_target_rad,
         success, position_error_m, solve_time_ms, ...)

LatestOnlyQPIKWorker(solver, *, max_joint_speed_rad_s,
                     max_joint_acceleration_rad_s2)
    .start() / .stop() / .clear() -> None
    .submit(request: IKRequest) -> None
    .latest_result() -> IKResult | None
```

请求与结果验证身份和数组形状，并把数组复制为只读。worker 的 `submit()` 是覆盖式非阻塞提交，不保证每个请求都被求解。

### `QPRequestCoordinator`

```python
QPRequestCoordinator(worker, config, lower_limit_rad, upper_limit_rad)
    .begin_generation() -> None
    .reset_velocity() -> None
    .capture_velocity_reference(target, frame) -> None
    .submit_if_ready(*, target, frame, q_seed_rad, q_actual_rad,
                     q_nominal_rad, dt_s, now_ns) -> bool
    .consume_latest(*, state, q_actual_rad, now_ns) -> ndarray | None
```

返回 `False` 表示重复样本或已有请求在飞；`consume_latest()` 返回 `None` 表示无新结果、结果过期/失败或身份不匹配。调用方不应绕过 coordinator 直接把 worker 结果应用到机器人。

## 5. Control 模块

### 配置与状态

`CartesianControlConfig` 是控制器内部的 frozen 配置，验证 QP、映射、Grip、关节速度/加速度、lookahead、反馈故障和夹爪参数。

`CartesianControlStatus` 是逐周期不可变快照，包含：状态机/Tracking、IK 结果和计数、按键与夹爪、actual/target/command、奇异性诊断、反馈故障，以及 VR 接收至机器人发送的分段时间戳和时延。诊断和 UI 应消费此对象，不应访问控制器私有字段。

### `FullBodyQPIKController`

```python
FullBodyQPIKController(
    kinematics: B601Kinematics,
    *,
    xr_to_base_rotation: ndarray,
    config: CartesianControlConfig | None = None,
    ik_worker: IKWorker | None = None,
)
    .start() -> None
    .stop() -> None
    .update(frame, observation: dict[str, float], dt_s: float, *, now_ns=None)
        -> tuple[dict[str, float] | None, CartesianControlStatus]
```

这是领域层最重要的 API。`frame` 可为 `ControllerSample`、`VRFrame` 或 `None`；`observation` 必须提供七个 `{joint}.pos`。注入 `ik_worker` 可在测试中替代真实异步求解器。非法反馈且尚无历史安全命令时，action 可以为 `None`。

控制器约定 `start()` 后再周期调用 `update()`，退出时 `stop()`。对于非法反馈，`update()` 返回 HOLD 命令和故障状态；达到配置的连续故障阈值时，状态中的 `feedback_abort_requested` 为真，runner 再执行持有和退出策略。

### 反馈、命令和夹爪函数

```python
read_robot_feedback(observation)
    -> tuple[q_actual_rad, gripper_actual_deg, error_reason]
feedback_limit_error(q_actual_rad, lower_limit_rad, upper_limit_rad) -> str

bound_position_command_to_feedback(command_position, feedback_position,
                                   max_error, *, lower_limit,
                                   upper_limit) -> ndarray
shape_joint_position_command(*, previous_position, previous_velocity,
                             target_position, dt_s, max_speed,
                             max_acceleration, lower_limit, upper_limit)
    -> tuple[command_deg, velocity_deg_s]

update_arm_position_command(...) -> tuple[command_rad, velocity_rad_s]
```

`GripperController` 公开同步反馈、故障恢复、Tracking 不可用、请求闭合、进入 HOLD、更新 Trigger 目标和更新整形命令的方法。Trigger 使用闩锁语义，夹爪命令受速度/加速度及可选反馈误差限制。

### 启动姿态

```python
reference_initial_q_to_dm(q_reference_rad, *, lower_limit_rad,
                          upper_limit_rad) -> ndarray

StartupPoseMover(target_rad, *, lower_limit_rad, upper_limit_rad,
                 max_speed_rad_s, max_acceleration_rad_s2,
                 tolerance_rad, max_command_feedback_error_rad=None,
                 feedback_limit_tolerance_rad=deg2rad(1), settle_samples=3)
    .update(actual_rad, dt_s) -> StartupPoseStatus
```

`reference_initial_q_to_dm()` 负责参考示例与 B601-DM 的 q2/q3 符号约定转换。`StartupPoseStatus` 给出当前命令、最大实测误差和是否已连续稳定到达；整体超时与停滞检测由 `runtime.safety.move_to_initial_pose()` 负责。

### MIT 与动力学

```python
B601GravityCompensator(urdf_path=None)
    .gravity_torque(q_rad) -> ndarray

MITCommandDispatcher(robot, *, kp, kd, torque_limit_nm,
                     arm_velocity_limit_rad_s,
                     gravity_scale=1.0, gravity_ramp_s=0.0,
                     dynamics_urdf=None)
    .get_observation() -> dict[str, Any]
    .set_observation(observation) -> None
    .set_arm_velocity(velocity_rad_s) -> None
    .set_arm_velocity_from_position_error(command_deg, actual_deg,
                                          lookahead_s) -> None
    .desired_velocity_rad_s -> ndarray
    .send_action(action) -> Any
```

dispatcher 包装原 follower：六轴走 MIT 命令，夹爪仍按配置的 follower 路径发送。它依赖当前 observation，使用前必须先同步反馈。MIT 是实验性实机路径，增益、力矩上限和重力倍率必须针对硬件验证。

## 6. Runtime 模块

### CLI

```python
build_parser(description: str | None = None) -> argparse.ArgumentParser
validate_args(args: argparse.Namespace) -> None
follower_pos_vel_velocity(args) -> float
follower_relative_target(args, joint_index: int) -> float
status_line(status: CartesianControlStatus) -> str
```

`validate_args()` 是 runner 启动前的统一边界校验。任何以代码方式构造 Namespace 并调用 `main()` 的集成也应保留这一步。

### 真机生命周期

```python
main() -> None
```

`runtime.real.main()` 负责：解析参数 → 创建 `RebotB601Follower` → 连接机器人 → 可选移动至初始位姿 → 创建运动学/VR/控制器 → 启动循环 → 状态/CSV → 故障处理 → 逆序清理。

安全辅助 API：

```python
move_to_initial_pose(robot, *, target_rad, lower_limit_rad,
                     upper_limit_rad, args, should_stop) -> bool
feedback_hold_action(observation, fallback_action)
    -> dict[str, float] | None
send_feedback_hold_action(robot, action) -> dict[str, float]
settle_persistent_feedback_fault(robot, observation, fallback_action,
                                 *, duration_s, fps) -> None

class PersistentFeedbackFault(RuntimeError): ...
```

## 7. Diagnostics 模块

### 在线记录

```python
build_csv_row(status: CartesianControlStatus) -> dict[str, object]

CSVLogger(path: str | Path)
    .write_row(row: Mapping[str, object]) -> None
    .close() -> None
```

logger 后台线程负责磁盘写入，`close()` 是必须调用的资源收尾接口，推荐使用 runner 已实现的 `try/finally` 生命周期。

### 离线分析

```python
TelemetryDataset.load(path: str | Path) -> TelemetryDataset
dataset.row_count -> int
dataset.duration_s -> float
dataset.median_sample_hz -> float | None
dataset.index_at(time_s: float) -> int
dataset.range_indices(start_s, end_s) -> tuple[int, int]
dataset.statistics(signal_key, start_s, end_s) -> SignalStatistics | None

decimate_minmax(x, y, max_points) -> tuple[x_reduced, y_reduced]
```

`TelemetryDataset` 会按时间戳稳定排序、跳过非法行，并基于 `SIGNAL_SPECS` 构造可画图信号。`decimate_minmax()` 保留时间桶极值，适合长序列可视化。

## 8. 最小集成示例

以下示例展示领域控制器的正确生命周期；真实代码仍需提供有效的七轴反馈并负责机器人下发：

```python
import numpy as np

from lerobot_teleoperator_rebot_vr import (
    B601Kinematics,
    CartesianControlConfig,
    FullBodyQPIKController,
)

kinematics = B601Kinematics()
controller = FullBodyQPIKController(
    kinematics,
    xr_to_base_rotation=np.array(
        [[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
        dtype=np.float64,
    ),
    config=CartesianControlConfig(),
)

controller.start()
try:
    action, status = controller.update(
        frame=None,
        observation={
            "shoulder_pan.pos": 0.0,
            "shoulder_lift.pos": 0.0,
            "elbow_flex.pos": 0.0,
            "wrist_flex.pos": 0.0,
            "wrist_yaw.pos": 0.0,
            "wrist_roll.pos": 0.0,
            "gripper.pos": 0.0,
        },
        dt_s=1.0 / 90.0,
    )
finally:
    controller.stop()
    kinematics.close()
```

`frame=None` 时控制器不会激活 VR 跟踪，示例只用于说明接口与清理顺序。

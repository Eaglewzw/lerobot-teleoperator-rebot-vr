# 架构与核心模块 API

本文描述 `lerobot_teleoperator_rebot_vr` 包的整体架构、核心模块划分、模块间通信机制
以及各模块的对外 API。控制环内部的数学推导见
[控制设计](CONTROL_DESIGN.md) 与 [逆解设计](INVERSE_KINEMATICS_DESIGN.md)，命令行参数见
[参数说明](PARAMETERS.md)。

## 总览

包源码按领域分为六个模块，包根目录只保留 LeRobot 插件配置、注册和兼容入口：

| 模块 | 职责 |
|---|---|
| 包根层 | LeRobot 插件注册（`RebotVRTeleop` / `RebotVRTeleopConfig`）、运行参数 dataclass、旧导入路径兼容 |
| `vr/` | VR 传输（XRoboToolkit V1 TCP）、Tracking 数据模型、坐标映射与离合状态机 |
| `ik/` | Pinocchio 正运动学/雅可比、QP 微分逆解、异步求解 worker 与请求协调 |
| `control/` | 闭环笛卡尔主控制器、命令整形、夹爪控制、MIT/重力前馈、启动位姿迁移 |
| `runtime/` | 真机遥操会话编排（入口 `rebot-vr-teleoperate`）、CLI 参数、安全保护 |
| `diagnostics/` | 逐帧 CSV 遥测记录与延迟汇总、离线信号分析 |
| `tools/` | 独立调试工具：`rebot-vr-print`（VR 数据自检）、`rebot-gripper-test`（夹爪标定） |

## 分层架构

```text
PICO 4 (XRoboToolkit APK)
        │  TCP 二进制流 (0.0.0.0:63901)
        ▼
┌─ vr/ ─────────────────────────────────────────────┐
│ xr_v1.py     TCP server + V1 协议解码 (线程 rebot-vr-v1) │
│ tracking.py  ControllerSample + LatestSampleBuffer     │
│ controller.py VRController 协议 / 后端工厂              │
│ pose_mapping.py RelativePoseMapper 离合状态机           │
└──────┬─────────────────────────────────────────────┘
       │ ControllerSample / VRFrame (latest-only 槽)
       ▼
┌─ control/ ────────────────────────────────────────┐
│ controller.py FullBodyQPIKController.update() 主控制环 │
│  ├─ FK ──► ik/kinematics.py (B601Kinematics, Pinocchio)│
│  ├─ QP 请求 ──► ik/coordination.py ──► ik/async_worker.py│
│  │             (线程 rebot-vr-qp, FullBodyQPIKSolver)   │
│  ├─ 命令整形: arm_command.py / joint_command.py         │
│  └─ 夹爪: gripper.py                                    │
└──────┬─────────────────────────────────────────────┘
       │ action dict {joint}.pos (度)
       ▼
┌─ runtime/real.py 主循环 (默认 90 Hz, 单线程) ────────┐
│  POS_VEL: RebotB601Follower.send_action (LeRobot)     │
│  MIT:     control/mit.py MITCommandDispatcher         │
│            └─ control/dynamics.py 重力前馈 (Pinocchio) │
│  旁路: diagnostics/logger.py CSVLogger (线程 rebot-vr-csv)│
└──────────────────────────────────────────────────────┘
        │  串口转 CAN (damiao 协议, 921600)
        ▼
reBot B601-DM 七电机（六轴 + 夹爪）
```

LeRobot 插件路径（`RebotVRTeleop`）是同一套 vr/ik/control 组件的薄封装，供
`--teleop.type=rebot_vr` 发现；真机闭环的唯一受支持入口是 `rebot-vr-teleoperate`
（`runtime/real.py`），它直接组合同一组件套件而不经过 LeRobot 循环。

## 模块间通信

工程整体是**三线程 latest-only 模型**：VR 接收、QP 求解、主控制循环各一个线程，
只消费最新数据，互不阻塞。跨线程传递的数据均为 frozen dataclass，numpy 数组字段
构造时拷贝并置只读，天然可安全共享。

### 1. VR 样本通道（vr/）

- 生产者：`V1TrackingSource` 的 daemon 线程 `rebot-vr-v1`，运行单客户端 TCP server
  （`TcpReceiver.serve_forever`，accept/recv 超时 0.5 s 轮询以便停止）。字节流经
  `PacketStreamDecoder` 粘包/碎包重组 → `TrackingDecoder` 双层 JSON →
  `parse_controller_sample` 生成 XR 原始坐标系的 `ControllerSample`。
- 介质：`LatestSampleBuffer` —— `threading.Lock` 保护的单槽，`publish` 覆盖旧样本
  并返回递增 sequence；旧帧被丢弃而非排队。
- 消费者：主循环调 `latest_sample()` 非阻塞读取。
- 回调链：`TcpReceiver(on_bytes=feed_bytes)` → 解码 → `buffer.publish` +
  `on_sample(sample, tracking)`。`XRoboToolkitV1Controller._on_sample` 在**网络线程**
  内同步完成 `base_T_anchor` 坐标变换生成 `VRFrame`（机器人基座坐标系），锁内更新
  自身 `_latest` 槽供 `get_action()` 兼容路径使用；回调异常被捕获记日志，不炸网络线程。
- 时钟回退保护：检测到 `tracking_timestamp_ns` 回退时 `stream_epoch` 自增、清缓冲并
  要求重新松开 Grip；`RelativePoseMapper` 独立检测 epoch 变化并重置参考。

### 2. QP IK 异步求解通道（ik/）

- 不使用 `queue.Queue`：`LatestOnlyQPIKWorker` 用 `threading.Condition` + 两个单槽
  （`_pending` 请求槽、`_latest_result` 结果槽）。`submit` 在锁内**直接覆盖**未处理的
  旧请求并 notify —— 提交方永不阻塞；worker 线程 `rebot-vr-qp` 取出请求后在锁外求解。
- 生产者：控制循环线程经 `QPRequestCoordinator.submit_if_ready` 提交；同一 VR 采样
  去重（key = `(stream_epoch, tracking_timestamp_ns, received_monotonic_ns)`），且严格
  单飞（有请求在飞时不再提交）。
- 消费者：控制循环经 `consume_latest` 消费，按 generation / sequence / sample_id 三重
  匹配校验，超 `qp_max_solve_time_ms` 的结果作废。
- 失效语义：`begin_generation()` 使代次自增并清空双槽 —— 任何状态迁移（激活、松开
  Grip、回归 home/zero、反馈故障恢复）后旧请求与旧结果自动失效，无 Event 对象。
- 求解失败时结果回退 `q_target = q_seed`（防止反馈延迟退化为不受控积分器）。

### 3. 机器人反馈通道（插件路径）

- `RebotVRTeleop` 内 `_feedback_lock` + `_latest_feedback` 单槽：LeRobot 循环线程调
  `send_feedback()` 整体替换；`get_action()` 取走**并清空**（单次消费）。无反馈即抛
  `RuntimeError` —— fail-closed，绝不回退开环。

### 4. 真机主循环（runtime/）

- `runtime/real.py:main` 为单线程轮询（默认 90 Hz）：`robot.get_observation()` →
  `vr_controller.latest_sample()` → `arm_controller.update(frame, observation, dt_s)`
  → MIT 速度下发 → `robot_io.send_action(action)` → 写 CSV → 补帧睡眠。
- SIGINT/SIGTERM 只置协作式 `stop` 标志，各循环顶部检查，不在信号上下文做清理。
- 反馈读取失败进入瞬态 HOLD（重发最后命令）；连续 `feedback_fault_max_consecutive`
  （默认 5）次后定格 0.25 s 重发 HOLD，抛 `PersistentFeedbackFault`，退出时保持电机
  扭矩防止机械臂下落。

### 5. CSV 遥测通道（diagnostics/）

- 生产者：主循环每帧 `csv_logger.write_row(build_csv_row(status))`，dict 快照
  `put_nowait` 进无界 `queue.Queue`，磁盘 I/O 不进控制回路。
- 消费者：daemon 线程 `rebot-vr-csv` 逐行写并 flush，同时累计 21 项延迟指标；
  `close()` 投哨兵、排空后写 `_latency_summary.csv`（mean/min/p50/p95/p99/max）。

## 端到端数据流（控制周期）

`FullBodyQPIKController.update(frame, observation, dt_s)` 每周期按序执行：

1. 解析反馈（度→弧度），非法/越限 → HOLD 分支（重发最后命令，计数故障）；
2. 故障恢复首帧：reset 映射器与 generation，目标/命令重置为实测位姿；
3. `B601Kinematics.forward_kinematics(q_actual)` 算 TCP 位姿；
4. 按键边沿检测（B/Y 回零、A/X 回 home）；
5. `RelativePoseMapper.update(...)` 离合状态机 → `PoseMappingUpdate`（状态 + 目标位姿）；
6. 状态迁移处理：`begin_generation()`，进入/离开 ACTIVE 时冻结目标与命令；
7. `qp.consume_latest(...)` 消费已完成 IK 结果（lookahead 外推 + 限位钳制）；
8. ACTIVE 且有新目标 → `qp.submit_if_ready(...)` 提交 QP 请求（每样本最多一个在飞）；
9. 夹爪：`update_trigger_target` + `update_command`（Trigger 闩锁 + 梯形整形）；
10. 手臂：`update_arm_position_command` —— ACTIVE 直通 QP 输出（仅限位钳制与可选
    反馈误差钳制），非 ACTIVE 走速度/加速度整形；
11. 组装 action（`{joint}.pos` 七路，度）与 `CartesianControlStatus` 返回。

状态机（`TeleopState`，定义在 `vr/pose_mapping.py`）：

```text
WAITING ──收到新鲜 tracking──► IDLE ──Grip ≥ 0.85(迟滞)──► ACTIVE
   ▲                            │                            │
   └────样本超时/epoch 变化──── STALE ◄───Grip ≤ 0.75────────┘
HOLD：仅由 controller 在反馈故障时置位，反馈恢复后经重置回到 WAITING/IDLE
```

## 核心 API

### 包根层

`RebotVRTeleop`（`rebot_vr.py`，LeRobot `Teleoperator` 子类，`name = "rebot_vr"`）：

```python
RebotVRTeleop(config: RebotVRTeleopConfig,
              controller: VRController | None = None,          # 测试注入点
              arm_controller: FullBodyQPIKController | None = None)
.connect(calibrate: bool = True) -> None        # 建 B601Kinematics + QP 控制器 + VR 后端并启动
.get_action() -> RobotAction                    # fail-closed；消费反馈槽 + 最新 VR 样本 → update()
.send_feedback(feedback: dict[str, Any]) -> None  # 校验 7 路 {joint}.pos 有限后写入反馈槽
.disconnect() -> None                           # 逆序停止 arm_controller → VR → kinematics.close()
.action_features / .feedback_features -> dict[str, type]   # {joint}.pos × 7
.is_connected / .is_calibrated -> bool
```

`RebotVRConfig`（`config_rebot_vr.py`，`@dataclass(kw_only=True)`）：全部运行/映射/QP
参数（VR 后端、离合阈值、缩放、滤波、QP 权重/阻尼/奇异阈值、速度加速度上限、
lookahead、夹爪端点、`base_T_anchor`、监听地址等），`__post_init__` 全量校验。
`RebotVRTeleopConfig(TeleoperatorConfig, RebotVRConfig)` 经
`@TeleoperatorConfig.register_subclass("rebot_vr")` 注册，无新增字段。

常量：`REBOT_JOINTS`（7 关节名）、`ARM_JOINTS = REBOT_JOINTS[:6]`、
`DEFAULT_BASE_T_ANCHOR`（OpenXR→reBot 基座 4×4 变换）。

兼容导入：包 `__init__.py` 用 `sys.modules.setdefault` 注册 18 个 0.4 之前的旧平铺
模块别名（如 `lerobot_teleoperator_rebot_vr.kinematics` → `ik.kinematics`），
并 re-export 主要公开符号。

### vr/

数据模型（frozen dataclass，数组只读）：

```python
ControllerSample   # XR 原始坐标系：received/published_monotonic_ns, tracking_timestamp_ns,
                   # stream_epoch, side, position(3,), quaternion_xyzw(4,), grip, trigger,
                   # primary/secondary_button, status
VRFrame            # 机器人基座坐标系：grip_pos/grip_quat, squeeze, trigger, is_tracking,
                   # 时间戳链, stream_epoch, side, 按钮, head_pos/head_quat
```

传输与协议（`xr_v1.py`）：

```python
PacketParser.unpack(data: bytes) -> dict | None          # 单个小端 V1 包
PacketParser.pack(command: int, body: str | bytes) -> bytes
PacketStreamDecoder(on_warning=None).feed(data: bytes) -> list[dict]   # 粘包/碎包重组
TrackingDecoder.decode_tracking(packet) -> dict | None   # 仅 functionName=="Tracking"
TcpReceiver(host, port, on_bytes, *, on_listen/on_connect/on_disconnect/on_error)
  .serve_forever() / .stop()
V1TrackingSource(host="0.0.0.0", port=63901, *, side="right", on_status=None, on_sample=None)
  .start(timeout=3.0) / .stop()
  .latest_sample() -> ControllerSample | None
  .latest() -> tuple[ControllerSample | None, int]       # (样本, sequence)
  .stats() -> TrackingSourceStats
  .feed_bytes(data, *, received_monotonic_ns=None)       # 测试注入
LatestSampleBuffer.publish(sample) -> int / .latest() / .clear()   # 线程安全单槽
parse_controller_sample(tracking, side, *, received_monotonic_ns=None, stream_epoch=0)
  -> ControllerSample                                    # Tracking JSON → 样本
```

后端适配（`controller.py`）：

```python
class VRController(Protocol):
    is_connected / is_tracking -> bool
    connect() / disconnect() -> None
    get_action() -> dict[str, Any]                # VRFrame 转 LeRobot action 形状
    latest_sample() -> ControllerSample | None    # 原始 XR 样本（Isaac 后端恒 None）
make_vr_controller(config: RebotVRConfig) -> VRController
    # "xrobotoolkit_v1" → XRoboToolkitV1Controller（默认，TCP）
    # "isaac"           → Pico4VRController（Isaac Teleop + CloudXR 拉模型）
```

离合位姿映射（`pose_mapping.py`）：

```python
class TeleopState(str, Enum): WAITING / IDLE / ACTIVE / STALE / HOLD
PoseTarget(sample_id, position(3,), rotation(3,3))
PoseMappingUpdate(state, target, reference_captured, require_release, orientation_diagnostics)
RelativePoseMapper(*, side="right", xr_to_world=DEFAULT_XR_TO_WORLD,
                   position_scale=1.0, orientation_scale=1.0,
                   position_filter_hz=0.0, orientation_filter_hz=0.0,
                   position_deadband_m=0.0, orientation_deadband_rad=0.0,
                   grip_press_threshold=0.85, grip_release_threshold=0.75,
                   stale_timeout_s=0.2, ...)
  .update(sample, ee_position, ee_rotation, *, now_ns=None) -> PoseMappingUpdate
  .reset(*, require_release=True) -> None
```

通用工具（`adapter.py`）：`sample_is_fresh(sample, now_ns, timeout_s)`、
`sample_key(sample)`、`trigger_value(sample)`、
`vr_frame_from_raw_action(action) -> VRFrame`。

### ik/

运动学与求解器（`kinematics.py`）：

```python
B601Kinematics(urdf_path=None, end_effector_frame="gripper_end")  # 惰性加载 Pinocchio
  .forward_kinematics(q_rad) -> (position(3,), rotation(3,3))     # TCP FK
  .tcp_jacobian(q_rad) -> np.ndarray (6,6)   # LOCAL_WORLD_ALIGNED，[linear; angular]
  .tcp_pose_error(q_rad, target_position, target_rotation) -> np.ndarray (6,)
  .lower_position_limit / .upper_position_limit -> np.ndarray (6,)
  .close()
FullBodyQPIKSolver(kinematics, *, solver="scipy", ik_mode="pose",
                   position_cost=20.0, orientation_cost=2.0, orientation_cost_min=0.05,
                   position_gain=10.0, orientation_gain=8.0,
                   damping_min=1e-3, damping_max=0.1,
                   smoothness_cost=0.05, posture_cost=0.01,
                   joint_limit_margin_rad=0.03, max_solve_time_ms=8.0,
                   singularity_threshold=0.08, singularity_critical_threshold=0.02,
                   singularity_characteristic_length_m=0.3)
  .solve(*, target_position, target_rotation, q_actual, dq_previous, dt, q_nominal,
         max_joint_speed, max_joint_acceleration,
         target_linear_velocity_m_s=None, target_angular_velocity_rad_s=None)
    -> QPSolveResult   # q_target_rad, joint_velocity_rad_s, success, 误差, solve_time_ms,
                       # reason, sigma_min, condition_number, damping, orientation_weight
default_urdf_path() -> Path    # 包内 assets/rebot_b601_dm_kinematics.urdf
```

异步通道（`async_worker.py` / `coordination.py`）：

```python
IKRequest(sequence, generation, sample_id, target_position, target_rotation,
          q_seed, q_actual, dq_previous, q_nominal, dt, 时间戳链, 目标 twist)
IKResult(generation, sequence, sample_id, q_target_rad, success, 误差与诊断, 时间戳链)
LatestOnlyQPIKWorker(solver, *, max_joint_speed_rad_s, max_joint_acceleration_rad_s2)
  .start() / .stop() / .clear()
  .submit(request: IKRequest) -> None            # 覆盖式 pending 槽，非阻塞
  .latest_result() -> IKResult | None
  .submitted / .solved / .rejected -> int        # 满足 control.types.IKWorker Protocol
QPRequestCoordinator(worker, config, lower_limit_rad, upper_limit_rad)
  .begin_generation() -> None                    # 代次自增 + 清槽 + 复位状态
  .submit_if_ready(*, target, frame, q_seed_rad, q_actual_rad, q_nominal_rad,
                   dt_s, now_ns) -> bool         # 采样去重 + 严格单飞 + 速度前馈差分
  .consume_latest(*, state, q_actual_rad, now_ns) -> np.ndarray | None
                                                 # 三重匹配校验 + lookahead 外推 + 限位钳制
  .capture_velocity_reference(target, frame) / .reset_velocity()
```

### control/

主控制器（`controller.py`）：

```python
FullBodyQPIKController(kinematics, *, xr_to_base_rotation,
                       config: CartesianControlConfig | None = None,
                       ik_worker: IKWorker | None = None)
  .start() / .stop()                             # 启停 IK worker 线程
  .update(frame, observation: dict[str, float], dt_s: float, *, now_ns=None)
    -> tuple[dict[str, float] | None, CartesianControlStatus]
    # observation 单位为度；action 为 {joint}.pos × 7（度）；HOLD 且无可发命令时 action=None
```

配置与状态（`types.py`，均 frozen dataclass）：

```python
CartesianControlConfig     # QP/映射/限速/lookahead/夹爪全量参数，__post_init__ 校验
CartesianControlStatus     # 80+ 字段：状态机、actual/target/command 各 7 路、IK 诊断、
                           # 反馈故障计数、30+ 延迟/时间戳指标、MIT 向量
class IKWorker(Protocol)   # start/stop/clear/submit/latest_result + submitted/solved/rejected
ARM_JOINT_NAMES / GRIPPER_NAME / FOLLOWER_LOWER_RAD / FOLLOWER_UPPER_RAD
```

命令整形（`joint_command.py` / `arm_command.py` / `gripper.py`）：

```python
shape_joint_position_command(*, previous_position, previous_velocity, target_position,
                             dt_s, max_speed, max_acceleration, lower_limit, upper_limit)
  -> (command, velocity)                          # 梯形速度整形 + 过冲归零 + 限位
bound_position_command_to_feedback(command_position, feedback_position, max_error, *,
                                   lower_limit, upper_limit) -> np.ndarray
update_arm_position_command(*, previous_position_rad, previous_velocity_rad_s,
                            target_position_rad, actual_position_rad, state, dt_s,
                            lower_limit_rad, upper_limit_rad, config, shape_fn, bound_fn)
  -> (command_rad, velocity_rad_s)                # ACTIVE 直通；非 ACTIVE 整形
GripperController                                # 可变 dataclass，Trigger 闩锁
  .update_trigger_target(*, tracking_fresh, trigger, open_deg, closed_deg)
  .update_command(*, actual_deg, dt_s, ..., shape_fn, bound_fn)
  .synchronize_to_feedback / .reset_after_feedback_recovery / .enter_feedback_hold / ...
```

启动与反馈（`startup.py` / `feedback.py`）：

```python
reference_initial_q_to_dm(q_reference_rad, *, lower_limit_rad, upper_limit_rad)
  -> np.ndarray                                  # RS→DM 符号约定（q2/q3 取反）
StartupPoseMover(target_rad, *, lower/upper_limit_rad, max_speed_rad_s,
                 max_acceleration_rad_s2, tolerance_rad, ...)
  .update(actual_rad, dt_s) -> StartupPoseStatus(command_rad, max_actual_error_rad, done)
read_robot_feedback(observation) -> (q_actual_rad, gripper_actual_deg, error_str)
feedback_limit_error(q_actual_rad, lower_limit_rad, upper_limit_rad) -> str
```

MIT 路径（`mit.py` / `dynamics.py`）：

```python
MITCommandDispatcher(robot, *, kp, kd, torque_limit_nm, arm_velocity_limit_rad_s,
                     gravity_scale=1.0, gravity_ramp_s=1.0, dynamics_urdf=None)
  .send_action(action) -> dict                   # q1-q6 改 send_mit(pos, vel, kp, kd, ff)
  .set_arm_velocity(velocity_rad_s | None)
  .set_arm_velocity_from_position_error(command_deg, actual_deg, lookahead_s)
  .get_observation() / .set_observation()        # 缓存反馈供重力计算
  # __getattr__ 透明代理到底层 RebotB601Follower
B601GravityCompensator(urdf_path=None).gravity_torque(q_rad) -> np.ndarray (6,)  # Nm
```

### runtime/

```python
real.main() -> None                              # rebot-vr-teleoperate 入口
cli.build_parser(description=None) -> argparse.ArgumentParser    # 五组约 60 个参数
cli.validate_args(args) -> None                  # 正性/区间/QP/MIT/夹爪全量校验
cli.follower_pos_vel_velocity(arm_rad_s, wrist_rad_s, gripper_deg_s) -> list[float]
cli.follower_relative_target(arm_deg, wrist_deg, gripper_deg=None) -> float | dict
cli.status_line(status, sent_action=None) -> str # 11 行人类可读状态
safety.move_to_initial_pose(robot, *, target_rad, lower/upper_limit_rad, args,
                            should_stop) -> bool
safety.feedback_hold_action(observation, fallback_action) -> dict | None
safety.send_feedback_hold_action(robot, action)  # 临时绕过 max_relative_target
safety.settle_persistent_feedback_fault(robot, observation, fallback_action, *,
                                        duration_s, fps) -> None
class safety.PersistentFeedbackFault(RuntimeError)
```

### diagnostics/

```python
build_csv_row(status: CartesianControlStatus) -> dict[str, object]   # 拍平一行
CSVLogger(output_path)
  .write_row(row: Mapping) -> None   # dict 快照 put_nowait，非阻塞
  .close() -> None                   # 排空后写 <stem>_latency_summary.csv
TelemetryDataset.load(csv_path) -> TelemetryDataset
  .statistics(signal_key, start_s, end_s) -> SignalStatistics | None
  .index_at(time_s) / .range_indices(start_s, end_s)
  .row_count / .duration_s / .median_sample_hz
decimate_minmax(x, y, max_points) -> (x, y)      # min-max 桶抽稀保极值
SIGNAL_SPECS / SIGNAL_BY_KEY / DEFAULT_SIGNAL_KEYS
```

CSV 列族：`TIMESTAMP_FIELDNAMES`（20）+ `LATENCY_FIELDNAMES`（21）+
`MIT_FIELDNAMES`（19）+ 关节 actual/target/command 21 列 + 状态列；汇总列
`SUMMARY_FIELDNAMES`（metric/samples/mean/min/p50/p95/p99/max）。

### tools/

```python
print_vr_data.main()   # rebot-vr-print：不连机器人打印手柄 tracking/按键/位姿
gripper_test.main()    # rebot-gripper-test：q1-q6 保持反馈，夹爪直达目标测试
gripper_test.run_gripper_test(robot, *, target_deg, speed_deg_s, acceleration_deg_s2,
                              relative_target_deg, fps, timeout_s, tolerance_deg,
                              settle_samples, status_rate, should_stop)
  -> GripperTestResult
```

## 入口点与插件注册

- 插件发现：发行包名前缀 `lerobot_teleoperator_` +
  `@TeleoperatorConfig.register_subclass("rebot_vr")`，无 entry_points。
- console scripts（`pyproject.toml`）：
  - `rebot-vr-teleoperate` → `runtime.real:main`（真机遥操，唯一受支持闭环入口）
  - `rebot-vr-print` → `tools.print_vr_data:main`
  - `rebot-gripper-test` → `tools.gripper_test:main`
- 依赖：`lerobot[rebot] >=0.6,<0.7`、`pin >=3.9,<5`（Pinocchio）、`scipy`、`numpy`；
  可选 extras：`qp(osqp)`、`isaac`、`test`。

## 模块依赖方向

```text
runtime ──► control ──► ik ──► vr (tracking/models/pose_mapping)
   │          │          ▲          ▲
   │          ▼          │          │
   │       diagnostics   └── types ◄┘   (ik↔control 经 types.py 的 Protocol/dataclass 解耦)
   ▼
lerobot.robots.rebot_b601_follower (外部)
```

- `vr/tracking.py`、`vr/models.py` 不依赖包内任何模块（纯数据层）；
- `vr/controller.py` 是 vr 包内唯一依赖 `config_rebot_vr` 的文件；
- `control/types.py` 从 `ik.async_worker` 导入 `IKRequest/IKResult`，
  `ik/coordination.py` 反向导入 `control.types` 的 `CartesianControlConfig/IKWorker`，
  构成经 Protocol 解耦的循环依赖；
- `diagnostics` 只消费 `control.types.CartesianControlStatus`，不被控制路径依赖；
- `tools/` 只依赖 `vr`、`control.joint_command/types` 与外部 lerobot。

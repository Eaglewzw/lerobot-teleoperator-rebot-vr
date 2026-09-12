# reBot VR 遥操作工程架构

## 1. 系统定位

该工程是 LeRobot 的第三方遥操作插件：接收 PICO 4/XRoboToolkit 控制器位姿，将 VR 相对运动映射为 reBot B601-DM 的 TCP 目标，通过闭环六轴 QP 逆运动学产生关节目标，最后经 LeRobot follower 下发到六个机械臂关节和一个夹爪。

源码采用 `src` 布局，Python 包为 `lerobot_teleoperator_rebot_vr`。工程有 **5 个核心模块**，另有 **2 个辅助模块**：

| 类型 | 模块 | 主要职责 |
|---|---|---|
| 核心 1 | 包根配置与集成层 | 参数模型、LeRobot 插件注册、兼容导入、组件装配 |
| 核心 2 | `vr/` | TCP 协议解码、VR 数据模型、坐标变换、Grip 离合映射 |
| 核心 3 | `ik/` | Pinocchio 运动学、QP 求解、异步 worker、请求/结果协调 |
| 核心 4 | `control/` | 闭环主控制器、反馈校验、关节与夹爪命令整形、MIT 控制 |
| 核心 5 | `runtime/` | 真机生命周期、90 Hz 主循环、CLI、安全退出和故障处理 |
| 辅助 | `diagnostics/` | CSV 异步遥测、时延汇总、离线数据分析 |
| 辅助 | `tools/` | VR 打印和夹爪测试命令 |

## 2. 目录与职责

```text
pyproject.toml                         打包、依赖、三个命令行入口
src/lerobot_teleoperator_rebot_vr/
├── config_rebot_vr.py                公共配置、关节常量、插件配置注册
├── rebot_vr.py                       LeRobot Teleoperator 适配器
├── __init__.py                       公共导出及旧模块路径兼容
├── urdf/                             运动学/动力学 URDF
├── vr/
│   ├── xr_v1.py                      XRoboToolkit V1 TCP 分帧与 Tracking 解码
│   ├── tracking.py                   原始控制器样本和 latest-only 缓冲
│   ├── controller.py                 VR 源协议与 XRoboToolkit V1 实现
│   ├── pose_mapping.py               相对位姿映射与离合状态机
│   ├── adapter.py                    ControllerSample/VRFrame 统一适配
│   └── models.py                     VRFrame 数据模型
├── ik/
│   ├── kinematics.py                 FK、Jacobian、位姿误差、QP IK
│   ├── async_worker.py               latest-only 异步 QP worker
│   └── coordination.py               请求去重、代次隔离、结果消费/lookahead
├── control/
│   ├── controller.py                 FullBodyQPIKController 主控制逻辑
│   ├── types.py                      配置、状态和协议
│   ├── feedback.py                   七轴反馈读取与限位校验
│   ├── arm_command.py                手臂位置命令生成
│   ├── joint_command.py              速度/加速度/反馈误差约束
│   ├── gripper.py                    Trigger 夹爪状态与轨迹
│   ├── startup.py                    初始姿态迁移
│   ├── mit.py / dynamics.py          MIT 分发与重力前馈
│   └── status.py                     控制状态/时延信息汇总
├── runtime/
│   ├── real.py                       推荐的真机入口
│   ├── cli.py                        CLI 构建、校验和状态显示
│   └── safety.py                     HOLD、初始移动、持续故障处理
├── diagnostics/                      遥测写入与分析
└── tools/                            独立诊断工具
```

包根的 `teleoperate_real.py`、`print_vr_data.py` 和 `gripper_test.py` 只是兼容旧路径的薄转发文件，不承载核心实现。

## 3. 总体分层

```text
PICO 4 / XRoboToolkit APK
        │ TCP V1 二进制流（默认 0.0.0.0:63901）
        ▼
┌──────────────────── VR 层 ─────────────────────┐
│ PacketStreamDecoder → TrackingDecoder          │
│ → ControllerSample → LatestSampleBuffer        │
│ → RelativePoseMapper → PoseTarget              │
└──────────────────────┬─────────────────────────┘
                       │ 不可变样本 / latest-only 单槽
                       ▼
┌───────────────── Control + IK 层 ───────────────┐
│ robot observation → FK                         │
│ PoseTarget → QPRequestCoordinator              │
│ → LatestOnlyQPIKWorker → FullBodyQPIKSolver    │
│ → 安全钳制/lookahead → 七轴 action dict         │
└──────────────────────┬─────────────────────────┘
                       │ {"<joint>.pos": degree}
                       ▼
┌────────────────── Runtime 层 ──────────────────┐
│ RebotB601Follower                              │
│ POS_VEL，或 MITCommandDispatcher + 重力前馈    │
│ CSVLogger（旁路，不阻塞控制循环）              │
└──────────────────────┬─────────────────────────┘
                       │ 达妙串口转 CAN
                       ▼
              reBot B601-DM（6 轴 + 夹爪）
```

主要外部依赖：LeRobot 提供 follower/插件抽象和电机 I/O；Pinocchio（`pin`）读取 URDF 并计算运动学/动力学；SciPy 提供旋转运算和默认 QP 优化；安装 `qp` extra 后可选 OSQP。

## 4. 运行入口

### 4.1 推荐真机入口

`rebot-vr-teleoperate` 映射到 `runtime.real:main`。它拥有完整反馈闭环：创建 follower、运动学、XRoboToolkit V1 数据源、控制器和日志器，按周期执行反馈读取、控制更新和命令下发。

### 4.2 LeRobot 插件入口

`RebotVRTeleop` 通过 `RebotVRTeleopConfig` 注册为 `teleop.type=rebot_vr`。该入口复用同一套 VR、IK 和 control 组件，但要求调用者每周期先调用 `send_feedback()`，再调用 `get_action()`。反馈槽读取后即清空；缺少新反馈时抛出 `RuntimeError`，不会退化为开环控制。

### 4.3 工具入口

| 命令 | 实现 | 用途 |
|---|---|---|
| `rebot-vr-teleoperate` | `runtime.real:main` | 真机遥操作 |
| `rebot-vr-print` | `tools.print_vr_data:main` | 验证 VR 网络数据和坐标 |
| `rebot-gripper-test` | `tools.gripper_test:main` | 绕过 VR 验证夹爪 |

## 5. 模块通信

### 5.1 VR 网络线程到主循环

1. `TcpReceiver` 是单客户端 TCP server，收到任意长度字节块。
2. `PacketStreamDecoder` 处理 TCP 碎包、粘包和损坏数据，输出完整 V1 包。
3. `TrackingDecoder` 只提取 `functionName == "Tracking"` 的双层 JSON。
4. `parse_controller_sample()` 生成 XR 原始坐标系的 `ControllerSample`。
5. `LatestSampleBuffer.publish()` 在锁保护下覆盖旧帧并递增 sequence；主循环通过 `latest_sample()` 非阻塞读取最新值。
6. XRoboToolkit 控制器的回调同步生成基座坐标系 `VRFrame`，供兼容 `get_action()` 路径使用。

此通道刻意不排队：控制系统更关心最新位姿而不是逐帧回放。样本中的 numpy 数组在构造时复制并设为只读，以便跨线程共享。

### 5.2 主循环到 QP 线程

`QPRequestCoordinator` 把 `PoseTarget`、实测关节角、上一速度和约束封装为 `IKRequest`。`LatestOnlyQPIKWorker` 使用 `threading.Condition` 和一个待处理槽：提交不会等待，未处理旧请求可被新请求覆盖；求解在锁外运行，结果写入另一个单槽。

协调器提供三类一致性保护：

- 以 `(stream_epoch, tracking_timestamp_ns, received_monotonic_ns)` 对 VR 样本去重；
- 单飞控制，同一时刻最多有一个未消费请求；
- 用 generation、sequence、sample_id 校验结果，状态切换后旧结果不能污染新控制阶段。

求解超过 `qp_max_solve_time_ms`、返回非有限值、失败或身份不匹配时，结果不会成为新目标。成功结果的关节速度经过分轴 lookahead 和安全限位生成候选命令。

### 5.3 主循环到机器人

`FullBodyQPIKController.update()` 接收统一的 `ControllerSample | VRFrame | None` 和 observation 字典，返回：

```python
action = {
    "shoulder_pan.pos": float,   # degree
    "shoulder_lift.pos": float,
    "elbow_flex.pos": float,
    "wrist_flex.pos": float,
    "wrist_yaw.pos": float,
    "wrist_roll.pos": float,
    "gripper.pos": float,
}
```

默认 `POS_VEL` 路径调用 follower 的 `send_action()`；实验性 MIT 路径由 `MITCommandDispatcher` 将位置误差转换为期望速度，并叠加 `B601GravityCompensator.gravity_torque()` 的前馈力矩。机器人 I/O 在主线程内同步完成，因此每个控制周期都使用对应的最新反馈。

### 5.4 主循环到诊断线程

`build_csv_row(status)` 将状态转成扁平字典；`CSVLogger.write_row()` 使用 `put_nowait` 写入内存队列，后台线程执行文件 I/O。`close()` 发送哨兵、排空队列，并生成同名 `_latency_summary.csv`。日志失败不会被伪装成控制数据。

## 6. 线程和所有权模型

| 执行上下文 | 所有者 | 主要工作 | 共享机制 |
|---|---|---|---|
| 主线程 | `runtime.real` | 反馈、映射、控制、下发、状态打印 | 同步调用 |
| `rebot-vr-v1` | `V1TrackingSource` | TCP accept/recv/解码 | `LatestSampleBuffer` 单槽 + Lock |
| `rebot-vr-qp` | `LatestOnlyQPIKWorker` | QP 求解 | 请求/结果单槽 + Condition |
| `rebot-vr-csv` | `CSVLogger` | CSV 写入、统计 | `queue.Queue` |

停止时按依赖逆序清理：控制器/QP → VR → follower/运动学 → CSV。SIGINT/SIGTERM 只设置停止标志，清理不在信号处理器中执行。

## 7. 单周期数据流

`FullBodyQPIKController.update(frame, observation, dt_s)` 的主要顺序如下：

1. 读取七路 `{joint}.pos` 反馈并校验有限性与 follower 软件限位。
2. 反馈异常时进入 HOLD，重发最后安全命令并累计故障次数；恢复首帧重置映射和 QP generation。
3. 对六轴实测角做 FK，得到当前 TCP 位姿。
4. 检测 A/X（回初始位姿）与 B/Y（回零）按钮边沿。
5. `RelativePoseMapper.update()` 根据 Tracking 新鲜度、Grip 迟滞和相对运动生成目标。
6. 状态切换时冻结/同步命令并隔离旧 QP 请求。
7. 消费已完成的合格 QP 结果。
8. ACTIVE 且出现新 VR 样本时提交下一次 QP 请求。
9. Trigger 经 `GripperController` 更新夹爪目标和整形命令。
10. 手臂命令经过关节限位、速度/加速度约束及可选反馈误差钳制。
11. 返回七轴 action 和包含 IK、安全、按键、关节及全链路时延的 `CartesianControlStatus`。

## 8. 遥操作状态机

```text
WAITING --收到新鲜 Tracking--> IDLE --Grip 超过 press 阈值--> ACTIVE
   ^                              ^                            |
   |                              |                            |
   +---- 无样本/epoch 变化 -- STALE <--Grip 低于 release ------+

反馈非法：任意状态 → HOLD
反馈恢复：重置参考和 generation → WAITING/IDLE，要求先松开 Grip
```

- `WAITING`：尚无可用 Tracking。
- `IDLE`：Tracking 正常但离合未按下，机械臂保持。
- `ACTIVE`：捕获 VR/机械臂参考后进行相对位姿跟踪。
- `STALE`：样本超时或流 epoch 改变，冻结目标并要求重新释放 Grip。
- `HOLD`：机器人反馈异常，禁止产生新运动目标。

Grip 采用 press/release 双阈值迟滞，避免临界值抖动。启动、中断和流时间戳回退后都要求先完全释放 Grip，再允许重新激活。

## 9. 安全边界

- **闭环反馈**：所有笛卡尔控制都以实测 `q_actual` 做 FK 和 QP 基准；插件接口没有反馈时直接失败。
- **反馈故障**：无字段、非数值、NaN/Inf 或越限均进入 HOLD；连续故障达到阈值后抛出 `PersistentFeedbackFault`。
- **请求隔离**：每次激活、释放、home/zero、stale、故障恢复都会开启新 generation。
- **关节保护**：求解器约束、限位 margin、速度/加速度整形和 follower 相对目标限制共同生效。
- **启动保护**：`StartupPoseMover` 以受限速度移动到初始姿态，并检测超时和停滞。
- **退出策略**：持续反馈故障退出前短暂重发 HOLD；正常退出是否关闭扭矩由 CLI 参数控制，必须先支撑机械臂。

## 10. 设计取舍

- VR 和 QP 通道采用 latest-only，而日志采用队列：前两者优先低延迟，日志需要保留每个已提交快照。
- QP 在独立线程：昂贵求解不会直接阻塞反馈与下发周期。
- `ControllerSample` 与 `VRFrame` 被统一适配：核心控制器不绑定某一个 VR 后端。
- 真机 runner 与 LeRobot 插件共用领域组件：前者负责完整运行编排，后者只做框架适配。
- 包根保留旧模块别名，领域实现仍集中在子包中，降低迁移成本而不重复逻辑。

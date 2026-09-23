# 工程架构

[项目说明](../../README.md) · [控制流程](CONTROL_DESIGN.md) · [API](API_REFERENCE.md) · [参数](PARAMETERS.md)

## 数据链路

```text
PICO / XRoboToolkit
  → TCP 解包 → 最新 Tracking 样本
  → Grip 相对位姿映射 → TCP / joint4 目标
  → Pinocchio FK/Jacobian → 异步 QP / 分离式 IK
  → 结果校验 → 前视位置与限位
  → POS_VEL follower / MIT 分发器 → 六轴电机
Trigger → 夹爪整形 → 第七个电机
主循环状态 → CSV 队列 → 后台写盘
```

TCP 默认监听 `0.0.0.0:63901`，主循环默认 90 Hz。

## 源码职责

源码根目录：`src/lerobot_teleoperator_rebot_vr/`。

| 路径 | 职责 |
|---|---|
| `config_rebot_vr.py`、`rebot_vr.py` | LeRobot 配置注册、反馈与 action 接口 |
| `constants.py` | 关节名称、软件限位、前馈力矩上限 |
| `vr/` | TCP 协议、Tracking 解析、坐标转换、Grip 映射 |
| `ik/kinematics.py` | FK、TCP/joint4 Jacobian、全身 QP |
| `ik/split_solver.py` | q1–q3 位置 QP、URDF 推导的 q4–q6 闭式分解 |
| `ik/async_worker.py`、`coordination.py` | 异步请求、样本去重、结果隔离 |
| `control/` | 状态机、命令整形、夹爪、MIT 与重力补偿 |
| `runtime/real.py`、`cli.py` | 真机主循环、配置加载、参数检查 |
| `runtime/safety.py`、`shutdown.py` | 启动、回零、故障保持、失能 |
| `runtime/feedback_requests.py` | 启动阶段反馈请求限频 |
| `diagnostics/` | CSV 记录、时延汇总、离线分析 API |
| `tools/` | VR 打印、夹爪测试等工具 |
| `urdf/` | 运动学与动力学模型 |

`config/pos_vel.yaml` 和 `config/mit.yaml` 提供保持 pose 行为的兼容默认参数；
`config/{pos_vel,mit}_{pose,split}.yaml` 提供显式的“电机控制 × IK”完整 profile。
CLI 显式值优先。Python 导入统一使用 `control/`、`ik/`、`vr/`、`runtime/`、`diagnostics/` 和 `tools/` 下的实际模块；旧模块别名和顶层转发文件已移除。外部脚本需迁移旧导入路径，已安装的正式命令名称不变。

## 入口

| 命令 | 实现 | 用途 |
|---|---|---|
| `rebot-vr-teleoperate` | `runtime.real:main` | 真机遥操 |
| `rebot-vr-print` | `tools.print_vr_data:main` | 检查 VR 数据 |
| `rebot-gripper-test` | `tools.gripper_test:main` | 单独测试夹爪 |

入口以 [pyproject.toml](../../pyproject.toml) 为准。当前没有 CSV 图形分析命令。

LeRobot 插件 `RebotVRTeleop` 每周期要求先 `send_feedback()`，再 `get_action()`；反馈只能消费一次。调用方负责电机连接与下发。

## 线程

| 线程 | 工作 | 数据交换 |
|---|---|---|
| 主线程 | 反馈、映射、结果消费、下发、状态打印 | 同步机器人 I/O |
| VR 线程 | TCP 接收与解析 | 加锁的 latest-only 样本槽 |
| IK 线程 | QP 或 split 求解 | Condition 保护的请求/结果槽 |
| CSV 线程 | 写盘、汇总 | Queue |

VR 和 IK 数组复制后设为只读。worker 可覆盖待处理请求；正式控制器同一时刻只允许一个未消费请求。结果须匹配 generation、sequence、sample_id，并通过成功、有限性和求解时间检查。

## 主循环顺序

反馈校验 → FK → 按键与 Grip 映射 → 消费 QP 结果 → 提交新样本 → 夹爪与手臂命令 → 下发 → 状态/CSV。

退出清理：QP → VR → follower → 运动学 → CSV。SIGINT/SIGTERM 设置停止标志，由主线程收尾。状态与异常处理见 [控制流程](CONTROL_DESIGN.md)。

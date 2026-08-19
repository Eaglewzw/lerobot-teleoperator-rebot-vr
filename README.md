# reBot B601-DM PICO 4 VR Teleoperator

面向 LeRobot 0.6.x 和 Seeed Studio reBot B601-DM 的 PICO 4 笛卡尔遥操作插件。

```text
PICO 手柄位姿
  → XR→B601 坐标转换
  → Grip 按下时建立腕部中心位置 + 夹爪姿态参考
  → q1-q3 异步位置 IK（腕部中心）+ q4-q6 闭式姿态解
  → 同一 VR 帧原子更新六轴目标
  → 限位、跳支保护、速度/加速度整形
  → LeRobot RebotB601Follower
```

| 控制 | 行为 |
|---|---|
| Grip | 按住激活遥操，松开保持当前姿态 |
| Trigger | 夹爪开合（`0` 张开，`1` 闭合） |
| A / X | 平滑返回起始姿态 |
| B / Y | 平滑返回六轴零点 |

## 安装

要求 Python 3.12+、LeRobot `>=0.6.0,<0.7.0`。XRoboToolkit V1 无需额外依赖；Isaac/CloudXR 后端安装 `[isaac]` extra。

```bash
uv pip install \
  --python /home/verser/Python/lerobot/.venv/bin/python \
  -e /home/verser/Python/lerobot-teleoperator-rebot-vr
source /home/verser/Python/lerobot/.venv/bin/activate
```

## 快速开始

### 1. 测试 VR 数据（不连机械臂）

```bash
rebot-vr-print --backend xrobotoolkit_v1 --host 0.0.0.0 --port 63901 --hand right --rate 10
```

检查 `tracking=true`、`grip`/`trigger` 范围 0–1、位姿跟随手部运动。同一时间只能有一个程序监听 63901。

### 2. 实机遥操

```bash
rebot-vr-teleoperate --robot-port /dev/ttyACM0 --backend xrobotoolkit_v1
```

启动后机械臂限速移动到起始姿态；**先完全松开 Grip 一次**，再按住 Grip 激活遥操。速度与加速度参数见[参数表](#参数)。首次使用建议按[分阶段测试](#3-分阶段测试)流程操作。

### 3. 分阶段测试

| 阶段 | 目的 | 参数 |
|---|---|---|
| 1 | 仅位置（姿态锁定） | `--position-scale 0.2 --orientation-scale 0 --max-joint-speed-rad-s 0.1` |
| 2 | 仅姿态 | `--position-scale 0 --orientation-scale 0.3 --max-joint-speed-rad-s 0.1` |
| 3 | 完整映射 | `--position-scale 1.0 --orientation-scale 1.0 --max-joint-speed-rad-s 0.4` |

## 参数

| 参数 | 默认 | 最大值 | 说明 |
|---|---|---|---|
| `--position-scale` | `1.0` | —（非负） | 手柄位移 → 腕部中心位移倍率 |
| `--orientation-scale` | `1.0` | —（非负） | 手柄旋转 → 末端旋转倍率 |
| `--max-joint-speed-rad-s` | `2.0` | **5.5** | q1-q3 速度上限（rad/s） |
| `--max-joint-acceleration-rad-s2` | `8.0` | **20** | q1-q3 加速度上限（rad/s²） |
| `--wrist-speed-rad-s` | 回退臂部 | **12** | q4-q6 速度上限（rad/s） |
| `--wrist-acceleration-rad-s2` | 回退臂部 | **60** | q4-q6 加速度上限（rad/s²） |
| `--max-relative-target-deg` | `5` | **20** | follower 相对目标保护（deg）；q4-q6 可用 `--wrist-relative-target-deg` 单独设置 |
| `--gripper-torque-ratio` | `0.2` | `1.0` | 夹爪最大夹持力比例 |
| `--fps` | `60` | —（建议 ≤120） | 主循环频率 |

> 以上最大值为实机验证的安全上限。电机硬件空载上限更高（q1-q3 = 5.5、q4-q6 = 20.9 rad/s），见[下一节](#最大速度与加速度)。CLI 不强制最大值，超过时电机物理上跑不动。

完整参数：`rebot-vr-teleoperate --help`。

### 最大速度与加速度

B601-DM 电机硬件上限：

| 关节 | 电机 | 空载最大 | 额定 |
|---|---|---|---|
| q1-q3 | 达妙 DM-J4340P-2EC（40:1） | **5.5 rad/s**（315°/s） | 3.8 rad/s |
| q4-q6 | 达妙 DM-J4310-2EC（10:1） | **20.9 rad/s**（1200°/s） | 12.6 rad/s |

生产/采集推荐上限（保留余量）：

```bash
rebot-vr-teleoperate \
  --max-joint-speed-rad-s 5.5 --max-joint-acceleration-rad-s2 20 \
  --wrist-speed-rad-s 12 --wrist-acceleration-rad-s2 60 \
  --max-relative-target-deg 20
```

注意：`--max-relative-target-deg` 应 ≥ 最大速度 °/s ÷ fps，否则会掐死实际速度；加速度建议从低值分档上调（10 → 20 → 40 → 60），每档观察 `wrist_clip_deg` 与跳变冲击。

## 测试

```bash
cd /home/verser/Python/lerobot-teleoperator-rebot-vr
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src \
  /home/verser/Python/lerobot/.venv/bin/python -m pytest -q
```

## 故障排除

| 现象 | 处理 |
|---|---|
| `command not found` | 激活 LeRobot `.venv`，或使用 `python -m lerobot_teleoperator_rebot_vr.teleoperate_real` |
| `Address already in use` | 关闭占用 63901 的其他进程 |
| 按住 Grip 仍为 `idle` | 完全松开 Grip 一次再重新按住 |
| `state=hold` | 反馈异常，检查 CAN/标定；连续 5 帧后保留扭矩受控退出 |
| 退出后机械臂下坠 | 退出前托住机械臂；默认断开时关闭扭矩 |

设计文档：[CONTROL_DESIGN.md](docs/CONTROL_DESIGN.md) · [INVERSE_KINEMATICS_DESIGN.md](docs/INVERSE_KINEMATICS_DESIGN.md)

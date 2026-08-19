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

**按钮映射**：Grip 按住激活遥操、松开保持；Trigger 控制夹爪（0=张开，1=闭合）；A/X 返回起始姿态，B/Y 返回六轴零点。

## 安装

要求 Python 3.12+、LeRobot `>=0.6.0,<0.7.0`。

```bash
# 安装到 LeRobot 的 .venv
uv pip install \
  --python /home/verser/Python/lerobot/.venv/bin/python \
  -e /home/verser/Python/lerobot-teleoperator-rebot-vr

source /home/verser/Python/lerobot/.venv/bin/activate
```

XRoboToolkit V1 不需要额外依赖。Isaac/CloudXR 后端需安装 `[isaac]` extra。

## 遥操机械臂

### 1. 先测试 VR 数据（不连机械臂）

```bash
rebot-vr-print --backend xrobotoolkit_v1 --host 0.0.0.0 --port 63901 --hand right --rate 10
```

检查 `tracking=true`、`grip`/`trigger` 范围 0-1、位置姿态跟随手部运动。

### 2. 实机遥操

```bash
rebot-vr-teleoperate \
  --robot-port /dev/ttyACM0 \
  --backend xrobotoolkit_v1 \
  --position-scale 1.0 \
  --orientation-scale 1.0 \
  --max-joint-speed-rad-s 0.4 \
  --max-joint-acceleration-rad-s2 1.0
```

启动后机械臂限速移动到起始姿态，然后：
- **完全松开 Grip 一次**（解除 require-release 状态）
- **按住 Grip** 激活遥操
- **松开 Grip** 保持当前姿态
- 按 **A/X** 返回起始姿态，按 **B/Y** 返回六轴零点

### 分轴腕部速度（可选）

q4-q6 可单独设置更高上限（达妙 4310 硬件能力高于 4340P）：

```bash
--wrist-speed-rad-s 0.8 --wrist-acceleration-rad-s2 3.0 --wrist-relative-target-deg 5.0
```

缺省时回退到对应臂部参数，旧命令行为不变。

### 安全测试

首次使用建议分阶段：

```bash
# 阶段1：仅位置（姿态锁定），低速
--position-scale 0.2 --orientation-scale 0 --max-joint-speed-rad-s 0.1

# 阶段2：仅姿态，低速
--position-scale 0 --orientation-scale 0.3 --max-joint-speed-rad-s 0.1

# 阶段3：完整映射，正常速度
--position-scale 1.0 --orientation-scale 1.0 --max-joint-speed-rad-s 0.4
```

## 关键参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `--position-scale` | 1.0 | 手柄位移到腕部中心位移的倍率 |
| `--orientation-scale` | 1.0 | 手柄旋转到末端旋转的倍率 |
| `--max-joint-speed-rad-s` | 2.0 | q1-q3 速度上限（rad/s） |
| `--wrist-speed-rad-s` | 回退臂部 | q4-q6 速度上限 |
| `--max-joint-acceleration-rad-s2` | 8.0 | q1-q3 加速度上限 |
| `--wrist-acceleration-rad-s2` | 回退臂部 | q4-q6 加速度上限 |
| `--max-relative-target-deg` | 5 | follower 相对目标保护 |
| `--gripper-torque-ratio` | 0.2 | 夹爪最大夹持力比例 |
| `--fps` | 60 | 主循环频率 |

完整参数：`rebot-vr-teleoperate --help`。详细设计文档：[CONTROL_DESIGN.md](docs/CONTROL_DESIGN.md)、[INVERSE_KINEMATICS_DESIGN.md](docs/INVERSE_KINEMATICS_DESIGN.md)。

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
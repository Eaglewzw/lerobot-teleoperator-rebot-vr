# reBot PICO 4 VR 遥操包 — 实现任务

你是一个熟练的 Python 开发者，需要为 SeeedStudio reBot B601-DM 达妙机械臂创建一个 PICO 4 VR 遥操作的独立 pip 包，不修改 LeRobot 任何源码。

## 背景

LeRobot 有一个第三方插件自动发现机制：`register_third_party_plugins()` 会扫描环境中所有以 `lerobot_teleoperator_*` 开头的包名并导入，导入时 `@TeleoperatorConfig.register_subclass("xxx")` 会把你的遥操器注册到全局 registry。`make_teleoperator_from_config` 的 fallback 路径 `make_device_from_device_class` 通过命名约定（Config 后缀 → 去掉 Config 的类名）自动找到你的遥操器类并实例化。
- 参考：`register_third_party_plugins()` 在 `src/lerobot/utils/import_utils.py:214`
- 参考：`make_device_from_device_class()` 在 `src/lerobot/utils/import_utils.py:149`
- 参考：`TeleoperatorConfig` 在 `src/lerobot/teleoperators/config.py`
- 参考：`RebotB601Follower` 在 `src/lerobot/robots/rebot_b601_follower/rebot_b601_follower.py`
- 参考：Isaac Teleop VR 示例在 `examples/isaac_teleop_to_so101/`
- 参考：`make_teleoperator_from_config` 在 `src/lerobot/teleoperators/utils.py`

## reBot 从臂信息

- 7 个关节：`shoulder_pan`, `shoulder_lift`, `elbow_flex`, `wrist_flex`, `wrist_yaw`, `wrist_roll`, `gripper`
- 关节限位：shoulder_pan(-150,150), shoulder_lift(-200,1), elbow_flex(-200,1), wrist_flex(-80,90), wrist_yaw(-90,90), wrist_roll(-90,90), gripper(-270,0)
- 控制用 `RebotB601Follower` 类，发送 `send_action({"joint_name.pos": angle_degrees})`，单位是度
- 通信：`motorbridge` 通过 CAN 总线控制达妙电机，`can_adapter="damiao"` 用串口桥 `/dev/ttyACM0`

## 任务

在 `/home/verser/Python/lerobot-teleoperator-rebot-vr/` 目录下创建完整的 pip 包。

### 第一步：创建目录结构和 pyproject.toml

```
lerobot-teleoperator-rebot-vr/
├── pyproject.toml
├── README.md
├── src/
│   └── lerobot_teleoperator_rebot_vr/
│       ├── __init__.py
│       ├── config_rebot_vr.py
│       ├── rebot_vr.py
│       ├── vr_controller.py
│       └── processor.py
└── tests/
    └── test_rebot_vr.py
```

**pyproject.toml** 要求：
- 包名：`lerobot-teleoperator-rebot-vr`
- 依赖：`lerobot[rebot]>=1.0.0`, `isaacteleop[cloudxr,retargeters-lite]~=1.3.131`, `scipy>=1.14`, `numpy`
- 使用 setuptools 构建，`[tool.setuptools.packages.find]` 指向 `src`

### 第二步：实现 config_rebot_vr.py

- 创建 `RebotVRConfig` 配置 dataclass，包含所有 VR 遥操需要的参数（clutch_threshold, pos_gain, orient_gain, joint_limits, max_step, gripper_open, gripper_closed, vr_backend, ws_host, ws_port 等）
- 创建 `RebotVRTeleopConfig`，继承 `TeleoperatorConfig` 和 `RebotVRConfig`，用 `@TeleoperatorConfig.register_subclass("rebot_vr")` 注册
- 注册名必须为 `"rebot_vr"`，这样 `--teleop.type=rebot_vr` 能识别

### 第三步：实现 vr_controller.py

- 创建 `Pico4VRController` 类，封装 Isaac Teleop 的 `TeleopSession`
- `connect()` 方法：启动 CloudXR 运行时 + 创建 OpenXR 会话
- `get_action()` 方法：读取手柄 6-DOF 位姿（grip_pos, grip_quat）+ squeeze + trigger
- 使用 `ControllersSource` + `ValueInput(base_T_anchor)` 管线
- 实现 `is_tracking` 属性判断手柄是否在线
- `disconnect()` 方法：清理会话和 CloudXR 运行时
- 手柄坐标系到 reBot 基座坐标系的变换矩阵（OpenXR: X=Right,Y=Up,Z=Backward → reBot: X=Forward,Y=Left,Z=Up）
- 可选：`_repair_urdf` 方法和 `_is_isaac_fix_available` 类属性，用于兼容新版 isaacteleop 的 URDF 修复

### 第四步：实现 rebot_vr.py

- 创建 `RebotVRTeleop` 类，继承 `Teleoperator`
- `config_class = RebotVRTeleopConfig`, `name = "rebot_vr"`
- `action_features` 返回 `{f"{joint}.pos": float for joint in REBOT_JOINTS}`
- `feedback_features` 返回 `{}`
- `connect()` 方法：创建 `Pico4VRController` 并连接
- `get_action()` 方法：核心映射逻辑
  - 读取 VR 手柄数据
  - 离合（deadman）：squeeze > clutch_threshold 才激活
  - 刚握紧时记录手柄当前位姿作为增量原点
  - 松开离合时保持当前位置
  - 手柄位置增量 → 大臂关节（shoulder_pan, shoulder_lift, elbow_flex）
  - 手柄旋转增量（四元数 → 欧拉角）→ 手腕关节（wrist_flex, wrist_yaw, wrist_roll）
  - 扳机值 → 夹爪开合
  - 软限位裁剪 + 单步最大位移限制
- `disconnect()` 方法：断开 VR 连接
- `is_calibrated` 返回 `True`（VR 手柄不需要标定）
- `calibrate()` 和 `configure()` 为空操作

### 第五步：实现 processor.py（可选）

- 创建 `MapVRToRebotAction` ProcessorStep，将 VR 手柄数据映射为 reBot 关节目标
- 用 `@ProcessorStep.register_subclass("rebot_vr_to_robot")` 注册

### 第六步：实现 __init__.py

- 从 config_rebot_vr 导入 `RebotVRConfig`, `RebotVRTeleopConfig`
- 从 rebot_vr 导入 `RebotVRTeleop`
- 导入即触发 `@register_subclass` 装饰器执行

### 第七步：编写测试

- 参考 `tests/teleoperators/test_rebot_102_leader.py` 的测试风格
- 测试 action_features 和 feedback_features 的 key 正确性
- 测试 config 注册成功（`TeleoperatorConfig._choice_registry` 中包含 `"rebot_vr"`）
- 测试自动发现（模拟 `register_third_party_plugins()` 导入）
- 测试 VR 控制器 mock 下的 get_action 行为
- 测试离合逻辑（squeeze < threshold 时 hold，squeeze > threshold 时 action 非零）
- 测试关节限位裁剪
- 测试单步位移限制

### 第八步：编写 README.md

- 说明包的功能：为 reBot B601-DM 提供 PICO 4 VR 遥操
- 安装说明：`pip install lerobot-teleoperator-rebot-vr`
- PICO 4 配对指南（CloudXR 连接步骤）
- 使用方式：
  ```bash
  # 遥操作
  lerobot-teleoperate \
      --robot.type=rebot_b601_follower \
      --robot.port=/dev/ttyACM0 \
      --robot.can_adapter=damiao \
      --teleop.type=rebot_vr

  # 录制数据集
  lerobot-record \
      --robot.type=rebot_b601_follower \
      --robot.port=/dev/ttyACM0 \
      --robot.can_adapter=damiao \
      --teleop.type=rebot_vr \
      --dataset.repo_id=your_name/your_dataset \
      --dataset.single_task="pick up cube"
  ```
- 所有可配置参数的说明
- 故障排除

### 第九步：验证

```bash
cd /home/verser/Python/lerobot-teleoperator-rebot-vr
uv pip install -e .
python -c "
from lerobot.utils.import_utils import register_third_party_plugins
register_third_party_plugins()
from lerobot.teleoperators.config import TeleoperatorConfig
assert 'rebot_vr' in TeleoperatorConfig._choice_registry, '注册失败!'
print('✅ 自动发现 + 注册成功')
"
```

## 关键约束

1. **不修改 LeRobot 源码** — 完全独立的外部包
2. **包名必须是 `lerobot_teleoperator_*` 模式** — 自动发现机制的必要条件
3. **`__init__.py` 的导入必须触发 `@register_subclass` 装饰器** — 这是注册到全局 registry 的唯一方式
4. **`Config` 类名必须以 `Config` 结尾** — `make_device_from_device_class` 依赖此命名约定
5. **Teleoperator 类必须放在 config 模块的父模块中或与 `config_xxx` 中 `xxx` 同名的模块中** — fallback 的搜索路径
6. **离合（deadman）是安全关键** — 松开离合必须冻结机械臂
7. **关节限位裁剪必须生效** — 防止超出机械限位
8. **单步位移限制** — 防止突变导致机械臂失控
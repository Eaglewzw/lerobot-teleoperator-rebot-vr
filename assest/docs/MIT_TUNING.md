# MIT 分轴调参

[运行参数](PARAMETERS.md) · [控制流程](CONTROL_DESIGN.md)

`rebot-mit-tune` 绕过 VR/QP，用预设轨迹测试单轴。配置独立，不读取或写回 `config/mit.yaml`。源码：[mit_tuning/](../../src/lerobot_teleoperator_rebot_vr/mit_tuning/)。

## 测试命令

```bash
rebot-mit-tune \
  --robot-port /dev/ttyACM0 \
  --joint q4 \
  --prepare-pose-deg 0 -45 -45 0 0 0 \
  --step-deg 5 \
  --transition-s 0.5 \
  --kp 10 \
  --kd 0.5 \
  --csv-log logs/mit_tuning/q4-kp10-kd0.5.csv
```

输入 `RUN` 后连接并运动，现场需托住机械臂、准备中止。准备姿态使用真实 DM 角度，无 q2/q3 符号转换；不指定时，以起始反馈为测试中心。

默认两轮：中心 → +5° → 中心 → -5° → 中心。其他五轴保持中心，夹爪保持连接时位置。

## 参数

| 参数 | 默认 | 用途 |
|---|---|---|
| `--joint` | 必填 | q1–q6 或电机名 |
| `--step-deg` | 5 | 相对中心的单侧偏移，deg |
| `--transition-s` / `--hold-s` | 0.5 / 2.5 | 每段运动 / 保持时间，s |
| `--cycles` / `--warmup-s` | 2 / 1 | 轮数 / 预保持时间 |
| `--fps` | 90 | 测试频率，允许 20–200 Hz |
| `--prepare-speed-rad-s` | 0.3 | 准备轨迹峰值速度 |
| `--prepare-settle-s` / `--prepare-tolerance-deg` | 2 / 3 | 准备结束保持时间 / 到位容差 |
| `--base-kp` | 25 30 30 10 10 10 | 六轴基线 |
| `--base-kd` | 5 5 4 0.5 0.5 0.5 | 六轴基线 |
| `--kp` / `--kd` | 沿用基线 | 覆盖所选轴 |
| `--torque-limit-nm` | 27 27 27 7 7 7 | 前馈限幅 |
| `--gravity-scale` / `--gravity-ramp-s` | 1 / 1.5 | 重力倍率 / 渐入秒数 |

轨迹为五次多项式。单段行程 D rad、时间 T s 时，峰值速度为 `1.875*D/T`，峰值加速度为 `(10/sqrt(3))*D/T²`。

## 保护

| 检查项 | 限制 / 默认 |
|---|---|
| 单侧偏移 | q1 ≤ 90°；q2/q3 ≤ 30°；q4 ≤ 60°；q5/q6 ≤ 88° |
| 目标端点 | 距软件限位至少 2° |
| 过渡 / 轮数 | ≥ 0.2 s / 1–10 轮 |
| `--max-relative-target-deg` | 20° 命令-反馈窗口 |
| `--max-tracking-error-deg` | 20° 所选轴误差 |
| `--max-hold-error-deg` | 8° 其他轴误差 |
| `--max-velocity-rad-s` / `--max-temperature-c` | 3 rad/s / 65°C |
| `--fault-consecutive` | 3 帧 |

速度阈值用于超速中止，不自动缩放轨迹。结束或异常时尝试按反馈位置零速度 HOLD，再失能；反馈缓存不能严格证明新鲜度或失能成功。

## 判读与迁移

每次生成 CSV、`*_summary.json` 和 `*_shutdown.json`。重点比较跟踪 RMSE、稳态偏差、超调及保持段振幅。稳定判据默认为 0.5° 持续 0.25 s；平均稳定时间须结合 `settled_holds / total_holds` 阅读。

比较时固定姿态、负载、行程、过渡时间和重力倍率，每次只改一个增益。选定后手动写入遥操 YAML，再测六轴联动。

调参工具显式发送轨迹 q/dq，未启用遥操的前视重建和最终加速度限幅。因此单轴平稳不代表整条遥操链路平稳。

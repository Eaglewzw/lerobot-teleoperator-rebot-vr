# 全六轴 TCP QP 离线测试报告

测试脚本：`PYTHONPATH=src python tests/qp_trajectory_benchmark.py`

测试对象固定为 `gripper_end` TCP。每个轨迹点使用实际六轴反馈计算完整 FK、6x6 Jacobian 和差分 QP；六个关节共同参与位置与姿态跟踪，结果以完整六轴目标原子应用。

覆盖轨迹：

- `past_90`
- `wide_arc`
- `circle`
- 纯 TCP 平移、纯 TCP 旋转和位置/姿态组合
- 关节限位、速度/加速度约束和不可达目标保持

输出指标包括 TCP 位置误差、QP 失败次数、最大单帧关节变化、平均/P95/最大求解时间和约束违规次数。

当前环境已验证 Pinocchio FK/Jacobian 数值一致性，热态 SciPy QP 求解时间约为 1-2 ms。真实 PICO 4、B601 机械臂和 CAN 闭环尚未执行。

## 实机分阶段

1. 仿真/离线轨迹注入。
2. 低速纯位置：`--position-scale 0.2 --orientation-scale 0 --max-joint-speed-rad-s 0.1`。
3. 低速纯姿态：`--position-scale 0 --orientation-scale 0.3 --max-joint-speed-rad-s 0.1`。
4. 低速六维组合：`--position-scale 0.2 --orientation-scale 0.2 --max-joint-speed-rad-s 0.2`。
5. 接近工作空间边界。
6. 正常数据采集速度。

所有阶段都必须保持机械臂周围无人，操作员随时准备释放 Grip。

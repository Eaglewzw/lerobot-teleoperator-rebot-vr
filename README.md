# reBot VR Teleoperation

**English** | [简体中文](README.zh-CN.md)

<h3 align="center">VR Teleoperation for reBot B601-DM with PICO 4</h3>

<p align="center">
  <a href="https://www.python.org"><img alt="Python" src="https://img.shields.io/badge/Python-3.12+-3776AB?style=for-the-badge&logo=python&logoColor=white"></a>
  <a href="https://github.com/huggingface/lerobot"><img alt="LeRobot" src="https://img.shields.io/badge/LeRobot-0.6.x-orange?style=for-the-badge"></a>
  <a href="assest/docs/INVERSE_KINEMATICS_DESIGN.md"><img alt="split IK" src="https://img.shields.io/badge/IK-split-green?style=for-the-badge"></a>
  <a href="https://www.picoxr.com"><img alt="PICO 4" src="https://img.shields.io/badge/PICO-4-blueviolet?style=for-the-badge"></a>
  <a href="LICENSE"><img alt="License" src="https://img.shields.io/badge/License-Apache--2.0-blue?style=for-the-badge"></a>
</p>

> A PICO 4 controller-based VR teleoperation platform for the reBot B601-DM robotic arm, covering pose mapping, split inverse kinematics, MIT/POS_VEL motor control, and dual-arm coordination for teleoperation demonstrations and data collection workflows.

* * *

## Overview

- **VR teleoperation**: Map PICO 4 controller poses to robot motion in real time, with single- and dual-arm operation, Grip clutching, button-triggered returns, and gripper control.
- **Split IK**: A position QP for q1–q3 tracks the joint4 pivot, while a closed-form solution for q4–q6 follows wrist orientation. Wrist rotation does not interfere with shoulder position tracking.
- **Dual-arm coordination and safety**: Support MIT and POS_VEL modes, synchronized startup, coordinated fault handling, joint-limit margins, braking constraints, fault holds, startup positioning, and return to zero on exit.
- **Diagnostics**: CSV logging, latency percentile summaries, and an offline analysis API for reviewing runs and tuning parameters.

* * *

## Demo

<p align="center">
  <img src="assest/dual-arm-block-grab.gif" width="600" alt="Dual-arm VR teleoperation demo">
</p>

## Architecture and Control

| Module | Responsibility | Implementation |
| --- | --- | --- |
| VR input | Receive PICO 4 controller poses and buttons, and validate tracking data | `vr/` |
| Inverse kinematics | Position QP for q1–q3 and closed-form wrist solution for q4–q6 | `ik/` |
| Motor commands | MIT with gravity feedforward and bounded references, plus POS_VEL | `control/` |
| Runtime | Single- and dual-arm control loops, startup positioning, return to zero, and fault holds | `runtime/` |

| Motor control | Single-arm profile | Dual-arm profile |
| --- | --- | --- |
| MIT | `config/mit_split.yaml` | `config/dual_mit_split.yaml` |
| POS_VEL | `config/pos_vel_split.yaml` | — |

* * *

## Safety

- Depending on the configuration, an arm can move automatically at startup even without pressing Grip. Verify zero calibration, joint directions, and the operating area before running. Start with a low-speed check.
- DM profiles disable motors on exit by default; the RS profile disables after verified zero return on normal Ctrl+C. Support the arms before exit. Torque retention cannot protect against supply loss or hardware communication timeouts.
- MIT mode is experimental. Verify gains and the direction of gravity compensation. `mit_torque_limit_nm` limits only feedforward torque, not total motor output torque.
- Dual-arm operation currently requires upright bases placed side by side and facing the same direction, with two independent CAN buses. There is no inter-arm collision detection or shared-object constraint.

* * *

## Quick Start

### 1. Requirements

- Linux, Python 3.12+, and LeRobot 0.6.x with the `rebot` extra
- Damiao serial-to-CAN adapter and PICO 4 with developer mode and USB debugging enabled

### 2. Installation and Calibration

Run from the repository root:

```bash
uv venv --python 3.12 .venv
source .venv/bin/activate
uv pip install -e .
```

If you already have a LeRobot environment, install into it directly. Adjust the path to match your environment:

```bash
uv pip install --python ~/Python/lerobot/.venv/bin/python -e .
source ~/Python/lerobot/.venv/bin/activate
```

Install the sender application on PICO:

```bash
adb install -r assest/reBot.apk
```

> In the sender app, enter the host computer's LAN IP address and port `63901`; do not enter `0.0.0.0`. To check incoming data without connecting an arm, run `rebot-vr-print --host 0.0.0.0 --port 63901 --hand right --rate 10`. Exit this tool afterward to release the port.

Calibrate the zero position for each arm separately. Stop other programs using the serial port, confirm that motor IDs 1–7 are online, and use the same `robot.id` as the teleoperation configuration:

```bash
# Single-arm example; adjust the serial port to match your connection
lerobot-calibrate \
  --robot.type=rebot_b601_follower \
  --robot.port=/dev/ttyACM0 \
  --robot.id=rebot_b601_vr
```

> For dual arms, use each arm's configured serial port and the IDs `rebot_left` / `rebot_right`. The dual-arm runner rejects missing calibration. Do not bypass checks or loosen arrival tolerances to compensate for zero-position errors.

### 3. Run

```bash
# Validate and print the effective configuration without connecting hardware
rebot-vr-teleoperate-dual --config config/dual_mit_split.yaml --dry-run

# Low-speed startup/return-to-zero check: moves the arms with VR disabled
# Speed <= 0.25 rad/s; acceleration <= 0.5 rad/s^2
rebot-vr-teleoperate-dual --config config/dual_mit_split.yaml --startup-zero-test

# Normal dual-arm teleoperation
rebot-vr-teleoperate-dual --config config/dual_mit_split.yaml
```

> Set `arms.left/right.robot_port` to match the physical left and right arms, and use distinct `robot_id` values. The supplied configuration uses `/dev/serial/by-path/...` to identify fixed USB sockets; do not swap the cables. If the command is unavailable, reinstall this project or run `PYTHONPATH=src python -m lerobot_teleoperator_rebot_vr.runtime.dual --config ...`.

For a single arm using MIT and split IK, after calibration:

```bash
rebot-vr-teleoperate \
  --robot-port /dev/ttyACM0 \
  --motor-control-mode mit \
  --control-config config/mit_split.yaml
```

* * *

### B601-RS Single Arm (Experimental MIT)

Use `--robot-model b601_rs` to load [config/rs_mit_split.yaml](config/rs_mit_split.yaml).
The default remains B601-DM; existing DM single/dual-arm commands are unchanged.
RS supports single and dual arms, SocketCAN and MIT, with MotorBridge 0.5.3+.

Read all seven motor positions without enabling, changing modes, or setting zeros:

```bash
PYTHONPATH=src python -m lerobot_teleoperator_rebot_vr.tools.rs_probe --port can0
```

Calibrate through LeRobot's standard command. Align the physical RS zero pose,
fully close the gripper and support the arm before pressing Enter at `Press ENTER when ready...`:

```bash
lerobot-calibrate \
  --robot.type=rebot_b601_rs_follower \
  --robot.port=can0 \
  --robot.id=rebot_b601_rs_vr

rebot-vr-teleoperate \
  --robot-model b601_rs \
  --robot-port can0 \
  --robot-id rebot_b601_rs_vr
```

Calibration writes and checks seven zeros, saves a record under the robot ID,
and exits without enabling. VR loads that record automatically; no
`--rs-zero-confirmed` flag is needed. RS defaults to 50 Hz, low motion limits,
automatic startup movement to `[0, 0.8, 0.8, 0, 0, 0]` radians, and zero return
on normal Ctrl+C exit, with a 0.5° arrival tolerance and a stable-feedback check. RS then disables torque; a failed/interrupted return never authorizes torque-off. Trigger controls the gripper between measured open
310.12° and closed 0° endpoints for this arm; remeasure after changing its zero or hardware. See the
[RS single-arm notes](assest/docs/RS_SINGLE_ARM.md) for calibration, model sources
and validation limits.

For two RS arms, use `config/dual_rs_mit_split.yaml` with the same dual-arm entry point:

```bash
rebot-vr-teleoperate-dual --config config/dual_rs_mit_split.yaml --dry-run
```

The confirmed wiring is left `can0` (ID `rebot_b601_rs_left`) and right `can1`
(ID `rebot_b601_rs_right`). Recalibrate each arm separately under these IDs before running;
both RS calibration records are checked before either CAN worker starts.
Both grippers are enabled with shared MIT gains and motion limits. At the user's
request, the right gripper uses the left gripper's 0°/310.12° endpoints; its own
travel has not been independently measured.
See [RS dual-arm setup and commands](assest/docs/RS_DUAL_ARM.md) for calibration,
gripper setup, the low-speed check and normal operation. DM commands are unchanged.

## Controller Inputs and Shutdown

| Input | Behavior |
| --- | --- |
| Grip | Hold to follow; release to hold position |
| Trigger | Control the gripper independently of Grip; open/closed angles are set in YAML |
| Right A / Left X | Return the corresponding arm to its initial pose |
| Right B / Left Y | Return the corresponding arm's six joints to zero and close its gripper |
| First Ctrl+C | During normal operation, request return to zero before exit according to each arm's configuration |
| Second Ctrl+C / SIGTERM | Abort return to zero and begin cleanup; this is not a hardware emergency stop |

> After startup, tracking recovery, or a button-triggered return, fully release Grip before reactivating it. Dual-arm following is enabled only when both arms are ready and tracking and feedback are valid. A problem on either side prevents both arms from continuing to follow. Startup failures, feedback faults, exceptions, and timed completion do not trigger an automatic return to zero.

## Logging and Tests

Dual-arm logs are saved to `logs/dual/<run_id>/` by default. For single-arm logging, append `--csv-log logs/session.csv` to the run command. See the [API reference](assest/docs/API_REFERENCE.md#csv-与分析) for CSV fields and analysis.

Run tests without connecting hardware:

```bash
uv pip install -e '.[test]'
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q
```

* * *

## Detailed Documentation

The detailed documents below are currently in Chinese.

[Architecture](assest/docs/ARCHITECTURE.md) · [API](assest/docs/API_REFERENCE.md) · [Control Design](assest/docs/CONTROL_DESIGN.md) · [Inverse Kinematics Design](assest/docs/INVERSE_KINEMATICS_DESIGN.md) · [Parameters](assest/docs/PARAMETERS.md)

* * *

> **Maintainer's note**: See [Inverse Kinematics Design](assest/docs/INVERSE_KINEMATICS_DESIGN.md) for the QP objective, hard constraints, and singularity adaptation. See [Control Design](assest/docs/CONTROL_DESIGN.md) for MIT command generation and bounded position references.

License: [Apache-2.0](LICENSE)

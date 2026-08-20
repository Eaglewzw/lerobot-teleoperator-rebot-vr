"""Move only the B601-DM gripper while holding the six arm joints at feedback."""

from __future__ import annotations

import argparse
import logging
import signal
import time
from dataclasses import dataclass
from typing import Callable, Protocol

import numpy as np

from .cartesian_controller import ARM_JOINT_NAMES, GRIPPER_NAME
from .joint_command import (
    bound_position_command_to_feedback,
    shape_joint_position_command,
)


GRIPPER_LOWER_DEG = -270.0
GRIPPER_UPPER_DEG = 0.0


class GripperRobot(Protocol):
    def get_observation(self) -> dict[str, float]: ...

    def send_action(self, action: dict[str, float]) -> dict[str, float]: ...


@dataclass(frozen=True)
class GripperTestResult:
    actual_deg: float
    target_deg: float
    command_deg: float
    sent_deg: float
    reached: bool


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Directly test the B601-DM gripper without VR or Trigger input. "
            "The six arm joints are held at their latest feedback positions."
        )
    )
    parser.add_argument("--robot-port", default="/dev/ttyACM0")
    parser.add_argument("--robot-id", default="rebot_b601_vr")
    parser.add_argument(
        "--can-adapter", choices=("damiao", "socketcan"), default="damiao"
    )
    parser.add_argument("--dm-serial-baud", type=int, default=921600)
    parser.add_argument(
        "--target-deg",
        type=float,
        required=True,
        help="absolute gripper motor target in degrees (-270 open, 0 closed)",
    )
    parser.add_argument("--speed-deg-s", type=float, default=90.0)
    parser.add_argument("--acceleration-deg-s2", type=float, default=360.0)
    parser.add_argument(
        "--relative-target-deg",
        type=float,
        default=10.0,
        help="maximum command-to-feedback difference accepted per cycle",
    )
    parser.add_argument(
        "--torque-ratio",
        type=float,
        default=0.1,
        help="FORCE_POS maximum grip force ratio in [0, 1]",
    )
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--timeout-s", type=float, default=10.0)
    parser.add_argument("--tolerance-deg", type=float, default=1.0)
    parser.add_argument("--settle-samples", type=int, default=3)
    parser.add_argument("--status-rate", type=float, default=5.0)
    parser.add_argument("--no-calibrate", action="store_true")
    parser.add_argument(
        "--disable-torque-on-disconnect",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="disable all motors on exit; disabled by default so the arm stays supported",
    )
    return parser


def validate_args(args: argparse.Namespace) -> None:
    positive = {
        "speed-deg-s": args.speed_deg_s,
        "acceleration-deg-s2": args.acceleration_deg_s2,
        "relative-target-deg": args.relative_target_deg,
        "fps": args.fps,
        "timeout-s": args.timeout_s,
        "tolerance-deg": args.tolerance_deg,
        "status-rate": args.status_rate,
    }
    invalid = [
        name
        for name, value in positive.items()
        if not np.isfinite(value) or value <= 0.0
    ]
    if invalid:
        raise ValueError(
            f"the following parameters must be positive: {', '.join(invalid)}"
        )
    if args.settle_samples <= 0:
        raise ValueError("settle-samples must be positive")
    if not np.isfinite(args.target_deg) or not (
        GRIPPER_LOWER_DEG <= args.target_deg <= GRIPPER_UPPER_DEG
    ):
        raise ValueError("target-deg must be finite and within [-270, 0]")
    if not np.isfinite(args.torque_ratio) or not 0.0 <= args.torque_ratio <= 1.0:
        raise ValueError("torque-ratio must be finite and within [0, 1]")


def feedback_hold_action(
    observation: dict[str, float], gripper_command_deg: float
) -> tuple[dict[str, float], float]:
    """Build an action that holds q1-q6 and changes only the gripper."""

    action: dict[str, float] = {}
    for name in ARM_JOINT_NAMES:
        key = f"{name}.pos"
        try:
            value = float(observation[key])
        except (KeyError, TypeError, ValueError) as exc:
            raise RuntimeError(f"invalid robot feedback field: {key}") from exc
        if not np.isfinite(value):
            raise RuntimeError(f"non-finite robot feedback field: {key}")
        action[key] = value

    gripper_key = f"{GRIPPER_NAME}.pos"
    try:
        actual_gripper_deg = float(observation[gripper_key])
    except (KeyError, TypeError, ValueError) as exc:
        raise RuntimeError(f"invalid robot feedback field: {gripper_key}") from exc
    if not np.isfinite(actual_gripper_deg):
        raise RuntimeError(f"non-finite robot feedback field: {gripper_key}")
    action[gripper_key] = float(gripper_command_deg)
    return action, actual_gripper_deg


def run_gripper_test(
    robot: GripperRobot,
    *,
    target_deg: float,
    speed_deg_s: float,
    acceleration_deg_s2: float,
    relative_target_deg: float,
    fps: float,
    timeout_s: float,
    tolerance_deg: float,
    settle_samples: int,
    status_rate: float,
    should_stop: Callable[[], bool] = lambda: False,
) -> GripperTestResult:
    """Run a feedback-bounded direct gripper move and return its final status."""

    started_s = time.monotonic()
    previous_loop_s = started_s - 1.0 / fps
    next_status_s = started_s
    command_deg: float | None = None
    velocity_deg_s = 0.0
    settled = 0
    result: GripperTestResult | None = None

    while not should_stop():
        loop_started_s = time.monotonic()
        observation = robot.get_observation()
        actual_deg = actual_gripper_deg_from(observation)
        if not GRIPPER_LOWER_DEG - 1.0 <= actual_deg <= GRIPPER_UPPER_DEG + 1.0:
            raise RuntimeError(
                "gripper feedback is outside the B601-DM range; check calibration: "
                f"{actual_deg:.2f}deg"
            )
        if command_deg is None:
            command_deg = float(
                np.clip(actual_deg, GRIPPER_LOWER_DEG, GRIPPER_UPPER_DEG)
            )

        dt_s = float(np.clip(loop_started_s - previous_loop_s, 1e-6, 0.05))
        previous_loop_s = loop_started_s
        position, velocity = shape_joint_position_command(
            previous_position=np.array([command_deg]),
            previous_velocity=np.array([velocity_deg_s]),
            target_position=np.array([target_deg]),
            dt_s=dt_s,
            max_speed=np.array([speed_deg_s]),
            max_acceleration=np.array([acceleration_deg_s2]),
            lower_limit=np.array([GRIPPER_LOWER_DEG]),
            upper_limit=np.array([GRIPPER_UPPER_DEG]),
        )
        command_deg = float(
            bound_position_command_to_feedback(
                position,
                np.array([actual_deg]),
                relative_target_deg * 0.9,
                lower_limit=np.array([GRIPPER_LOWER_DEG]),
                upper_limit=np.array([GRIPPER_UPPER_DEG]),
            )[0]
        )
        velocity_deg_s = float(velocity[0])
        action, _ = feedback_hold_action(observation, command_deg)
        sent_action = robot.send_action(action)
        sent_deg = float(sent_action.get(f"{GRIPPER_NAME}.pos", command_deg))
        reached = abs(actual_deg - target_deg) <= tolerance_deg
        settled = settled + 1 if reached else 0
        result = GripperTestResult(
            actual_deg=actual_deg,
            target_deg=target_deg,
            command_deg=command_deg,
            sent_deg=sent_deg,
            reached=settled >= settle_samples,
        )

        if loop_started_s >= next_status_s:
            print(
                "gripper_deg(actual/target/shaped/sent)="
                f"{actual_deg:.2f}/{target_deg:.2f}/{command_deg:.2f}/{sent_deg:.2f}",
                flush=True,
            )
            next_status_s = loop_started_s + 1.0 / status_rate
        if result.reached:
            return result
        if loop_started_s - started_s >= timeout_s:
            raise RuntimeError(
                "gripper did not reach the direct target before timeout: "
                f"actual={actual_deg:.2f}deg target={target_deg:.2f}deg "
                f"sent={sent_deg:.2f}deg"
            )
        sleep_s = 1.0 / fps - (time.monotonic() - loop_started_s)
        if sleep_s > 0.0:
            time.sleep(sleep_s)

    if result is None:
        raise RuntimeError("gripper test stopped before the first command")
    return result


def actual_gripper_deg_from(observation: dict[str, float]) -> float:
    try:
        value = float(observation[f"{GRIPPER_NAME}.pos"])
    except (KeyError, TypeError, ValueError) as exc:
        raise RuntimeError("invalid robot feedback field: gripper.pos") from exc
    if not np.isfinite(value):
        raise RuntimeError("non-finite robot feedback field: gripper.pos")
    return value


def main() -> None:
    args = build_parser().parse_args()
    validate_args(args)
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    try:
        from lerobot.robots.rebot_b601_follower import (
            RebotB601Follower,
            RebotB601FollowerRobotConfig,
        )
    except ImportError as exc:
        raise ImportError(
            "This command requires LeRobot with rebot_b601_follower support (0.6.x)."
        ) from exc

    robot_config = RebotB601FollowerRobotConfig(
        port=args.robot_port,
        id=args.robot_id,
        can_adapter=args.can_adapter,
        dm_serial_baud=args.dm_serial_baud,
        control_mode="pos_vel",
        gripper_control_mode="force_pos",
        gripper_torque_ratio=args.torque_ratio,
        pos_vel_velocity=[150.0] * 6 + [args.speed_deg_s],
        max_relative_target=args.relative_target_deg,
        disable_torque_on_disconnect=args.disable_torque_on_disconnect,
    )
    robot = RebotB601Follower(robot_config)
    stop = False

    def stop_now(_signal_number, _frame) -> None:
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, stop_now)
    signal.signal(signal.SIGTERM, stop_now)
    connected = False
    print(
        "Direct gripper test: q1-q6 will hold their measured positions; "
        f"gripper target={args.target_deg:.1f}deg.",
        flush=True,
    )
    if not args.disable_torque_on_disconnect:
        print(
            "Motor torque will remain enabled on exit; support the arm before power-off.",
            flush=True,
        )
    try:
        robot.connect(calibrate=not args.no_calibrate)
        connected = True
        result = run_gripper_test(
            robot,
            target_deg=args.target_deg,
            speed_deg_s=args.speed_deg_s,
            acceleration_deg_s2=args.acceleration_deg_s2,
            relative_target_deg=args.relative_target_deg,
            fps=args.fps,
            timeout_s=args.timeout_s,
            tolerance_deg=args.tolerance_deg,
            settle_samples=args.settle_samples,
            status_rate=args.status_rate,
            should_stop=lambda: stop,
        )
        if result.reached:
            print(
                f"Gripper target reached: actual={result.actual_deg:.2f}deg.",
                flush=True,
            )
        else:
            print(
                "Gripper test stopped before reaching the target: "
                f"actual={result.actual_deg:.2f}deg.",
                flush=True,
            )
    finally:
        if connected or robot.is_connected:
            robot.disconnect()


if __name__ == "__main__":
    main()

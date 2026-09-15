"""Command-line entry point for isolated, supervised MIT tuning."""

from __future__ import annotations

import argparse
import logging
import signal
import threading
import time
from datetime import datetime
from pathlib import Path

import numpy as np

from .analysis import TuningAnalyzer
from .hardware import LeRobotMITTuningRobot
from .logger import TuningCSVLogger
from .models import (
    ARM_JOINT_NAMES,
    EFFORT_LIMIT_NM,
    JOINT_ALIASES,
    MITTuningConfig,
)
from .session import MITTuningSession


DEFAULT_KP = (25.0, 30.0, 30.0, 10.0, 10.0, 10.0)
DEFAULT_KD = (5.0, 5.0, 4.0, 0.5, 0.5, 0.5)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Run a supervised, repeatable single-axis MIT excursion without VR or QP"
        )
    )
    connection = parser.add_argument_group("robot connection")
    connection.add_argument("--robot-port", default="/dev/ttyACM0")
    connection.add_argument("--robot-id", default="rebot_b601_vr")
    connection.add_argument(
        "--can-adapter", choices=("damiao", "socketcan"), default="damiao"
    )
    connection.add_argument("--dm-serial-baud", type=int, default=921600)
    connection.add_argument("--no-calibrate", action="store_true")

    motion = parser.add_argument_group("test motion")
    motion.add_argument(
        "--joint",
        required=True,
        choices=tuple(JOINT_ALIASES),
        help="joint under test: q1..q6 or its LeRobot motor name",
    )
    motion.add_argument("--step-deg", type=float, default=5.0)
    motion.add_argument("--cycles", type=int, default=2)
    motion.add_argument("--transition-s", type=float, default=0.5)
    motion.add_argument("--hold-s", type=float, default=2.5)
    motion.add_argument("--warmup-s", type=float, default=1.0)
    motion.add_argument("--fps", type=float, default=90.0)
    motion.add_argument(
        "--prepare-pose-deg",
        type=float,
        nargs=6,
        default=None,
        metavar=("Q1", "Q2", "Q3", "Q4", "Q5", "Q6"),
        help="move smoothly to this six-axis pose before capturing the test center",
    )
    motion.add_argument("--prepare-speed-rad-s", type=float, default=0.3)
    motion.add_argument("--prepare-settle-s", type=float, default=2.0)
    motion.add_argument("--prepare-tolerance-deg", type=float, default=3.0)

    gains = parser.add_argument_group("MIT gains")
    gains.add_argument(
        "--base-kp",
        type=float,
        nargs=6,
        default=DEFAULT_KP,
        metavar=("Q1", "Q2", "Q3", "Q4", "Q5", "Q6"),
    )
    gains.add_argument(
        "--base-kd",
        type=float,
        nargs=6,
        default=DEFAULT_KD,
        metavar=("Q1", "Q2", "Q3", "Q4", "Q5", "Q6"),
    )
    gains.add_argument(
        "--kp", type=float, default=None, help="selected-joint Kp; defaults to --base-kp"
    )
    gains.add_argument(
        "--kd", type=float, default=None, help="selected-joint Kd; defaults to --base-kd"
    )
    gains.add_argument(
        "--torque-limit-nm",
        type=float,
        nargs=6,
        default=tuple(EFFORT_LIMIT_NM),
        metavar=("Q1", "Q2", "Q3", "Q4", "Q5", "Q6"),
    )
    gains.add_argument("--gravity-scale", type=float, default=1.0)
    gains.add_argument("--gravity-ramp-s", type=float, default=1.5)
    gains.add_argument("--dynamics-urdf", type=Path, default=None)

    safety = parser.add_argument_group("safety and output")
    safety.add_argument(
        "--max-relative-target-deg",
        type=float,
        default=20.0,
        help="per-frame command-to-feedback limit; independent of total excursion",
    )
    safety.add_argument(
        "--max-tracking-error-deg",
        type=float,
        default=20.0,
        help="selected-joint target-to-feedback abort threshold",
    )
    safety.add_argument("--max-hold-error-deg", type=float, default=8.0)
    safety.add_argument("--max-velocity-rad-s", type=float, default=3.0)
    safety.add_argument("--max-temperature-c", type=float, default=65.0)
    safety.add_argument("--fault-consecutive", type=int, default=3)
    safety.add_argument("--settling-band-deg", type=float, default=0.5)
    safety.add_argument("--settling-window-s", type=float, default=0.25)
    safety.add_argument("--gripper-torque-ratio", type=float, default=0.2)
    safety.add_argument("--csv-log", type=Path, default=None)
    safety.add_argument(
        "--log-level", choices=("DEBUG", "INFO", "WARNING"), default="INFO"
    )
    return parser


def _validated(
    args: argparse.Namespace,
) -> tuple[MITTuningConfig, np.ndarray, np.ndarray, np.ndarray]:
    joint_index = JOINT_ALIASES[args.joint]
    config = MITTuningConfig(
        joint_index=joint_index,
        step_deg=args.step_deg,
        cycles=args.cycles,
        transition_s=args.transition_s,
        hold_s=args.hold_s,
        warmup_s=args.warmup_s,
        fps=args.fps,
        max_tracking_error_deg=args.max_tracking_error_deg,
        max_hold_error_deg=args.max_hold_error_deg,
        max_velocity_rad_s=args.max_velocity_rad_s,
        max_temperature_c=args.max_temperature_c,
        fault_consecutive=args.fault_consecutive,
        settling_band_deg=args.settling_band_deg,
        settling_window_s=args.settling_window_s,
        prepare_pose_deg=args.prepare_pose_deg,
        prepare_speed_rad_s=args.prepare_speed_rad_s,
        prepare_settle_s=args.prepare_settle_s,
        prepare_tolerance_deg=args.prepare_tolerance_deg,
    )
    kp = np.asarray(args.base_kp, dtype=np.float64)
    kd = np.asarray(args.base_kd, dtype=np.float64)
    torque = np.asarray(args.torque_limit_nm, dtype=np.float64)
    if (
        kp.shape != (6,)
        or not np.all(np.isfinite(kp))
        or np.any((kp < 0) | (kp > 500))
    ):
        raise ValueError("base Kp must contain six finite values in [0, 500]")
    if (
        kd.shape != (6,)
        or not np.all(np.isfinite(kd))
        or np.any((kd < 0) | (kd > 5))
    ):
        raise ValueError("base Kd must contain six finite values in [0, 5]")
    if args.kp is not None:
        if not np.isfinite(args.kp) or not 0 <= args.kp <= 500:
            raise ValueError("selected-joint Kp must be in [0, 500]")
        kp[joint_index] = args.kp
    if args.kd is not None:
        if not np.isfinite(args.kd) or not 0 <= args.kd <= 5:
            raise ValueError("selected-joint Kd must be in [0, 5]")
        kd[joint_index] = args.kd
    if (
        torque.shape != (6,)
        or not np.all(np.isfinite(torque))
        or np.any(torque <= 0)
        or np.any(torque > EFFORT_LIMIT_NM)
    ):
        raise ValueError(
            "torque limits must be positive and no greater than "
            "[27, 27, 27, 7, 7, 7] N*m"
        )
    if not np.isfinite(args.gravity_scale) or not 0 <= args.gravity_scale <= 2:
        raise ValueError("gravity-scale must be in [0, 2]")
    if not np.isfinite(args.gravity_ramp_s) or args.gravity_ramp_s < 0:
        raise ValueError("gravity-ramp-s must be finite and non-negative")
    if (
        not np.isfinite(args.max_relative_target_deg)
        or args.max_relative_target_deg <= 0.0
    ):
        raise ValueError("max-relative-target-deg must be finite and positive")
    if (
        not np.isfinite(args.gripper_torque_ratio)
        or not 0 <= args.gripper_torque_ratio <= 1
    ):
        raise ValueError("gripper-torque-ratio must be in [0, 1]")
    return config, kp, kd, torque


def _default_log_path(joint_index: int) -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return Path("logs") / "mit_tuning" / f"{stamp}_q{joint_index + 1}.csv"


def main() -> None:
    args = build_parser().parse_args()
    config, kp, kd, torque = _validated(args)
    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    output_path = args.csv_log or _default_log_path(config.joint_index)
    summary_path = output_path.with_name(f"{output_path.stem}_summary.json")
    shutdown_path = output_path.with_name(f"{output_path.stem}_shutdown.json")
    robot = LeRobotMITTuningRobot(
        port=args.robot_port,
        robot_id=args.robot_id,
        can_adapter=args.can_adapter,
        dm_serial_baud=args.dm_serial_baud,
        kp=kp,
        kd=kd,
        torque_limit_nm=torque,
        gravity_scale=args.gravity_scale,
        gravity_ramp_s=args.gravity_ramp_s,
        dynamics_urdf=args.dynamics_urdf,
        max_relative_target_deg=args.max_relative_target_deg,
        gripper_torque_ratio=args.gripper_torque_ratio,
        shutdown_report_path=shutdown_path,
    )
    csv_logger: TuningCSVLogger | None = None
    analyzer = TuningAnalyzer(config)
    stop_event = threading.Event()
    connected = False
    try:
        print("Support the arm: this command enables all motors and disables them on exit.")
        index = config.joint_index
        print(
            f"Planned test: q{index + 1}/{ARM_JOINT_NAMES[index]}, "
            f"excursion=+/-{config.step_deg:.2f}deg, "
            f"Kp={kp[index]:g}, Kd={kd[index]:g}."
        )
        if config.prepare_pose_deg is not None:
            print(
                "Preparation pose (deg): "
                f"{np.array2string(config.prepare_pose_deg, precision=1)}"
            )
        answer = input("Clear the workspace and type RUN to connect and start: ").strip()
        if answer != "RUN":
            print("MIT tuning cancelled; motors were not connected.")
            return
        robot.connect(calibrate=not args.no_calibrate)
        connected = True
        initial = robot.read_feedback()
        print(
            f"Test joint: q{index + 1}/{ARM_JOINT_NAMES[index]}, "
            f"center={np.rad2deg(initial.position_rad[index]):.2f}deg, "
            f"excursion=+/-{config.step_deg:.2f}deg, "
            f"Kp={kp[index]:g}, Kd={kd[index]:g}."
        )
        signal.signal(signal.SIGINT, lambda _sig, _frame: stop_event.set())
        signal.signal(signal.SIGTERM, lambda _sig, _frame: stop_event.set())
        csv_logger = TuningCSVLogger(output_path)
        session = MITTuningSession(
            robot,
            config=config,
            kp=kp,
            kd=kd,
            csv_logger=csv_logger,
            analyzer=analyzer,
            should_stop=stop_event.is_set,
        )
        completed = session.run()
        print("MIT tuning completed." if completed else "MIT tuning interrupted.")
        print(f"CSV: {output_path}")
        print(f"Summary: {summary_path}")
        print(analyzer.summary())
    finally:
        try:
            if connected:
                try:
                    current = robot.read_feedback()
                    robot.send_command(
                        current.position_rad, np.zeros(6, dtype=np.float64)
                    )
                    time.sleep(0.1)
                except Exception:
                    logging.getLogger(__name__).exception(
                        "failed to send final measured-position HOLD"
                    )
        finally:
            try:
                if robot.needs_cleanup:
                    robot.disconnect()
            finally:
                try:
                    if csv_logger is not None:
                        analyzer.write_summary(summary_path)
                finally:
                    if csv_logger is not None:
                        csv_logger.close()


if __name__ == "__main__":
    main()


__all__ = ["build_parser", "main"]

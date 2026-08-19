"""Run closed-loop PICO VR Cartesian teleoperation on a real reBot B601-DM."""

from __future__ import annotations

import argparse
import logging
import signal
import time
from pathlib import Path

import numpy as np

from .cartesian_controller import (
    ARM_JOINT_NAMES,
    GRIPPER_NAME,
    CartesianControlConfig,
    CartesianControlStatus,
    SplitArmWristController,
    vr_frame_from_raw_action,
)
from .config_rebot_vr import DEFAULT_BASE_T_ANCHOR, RebotVRConfig
from .kinematics import B601Kinematics
from .startup_pose import (
    DEFAULT_INITIAL_Q_REFERENCE_RAD,
    StartupPoseMover,
)
from .vr_controller import make_vr_controller


logger = logging.getLogger(__name__)


class PersistentFeedbackFault(RuntimeError):
    """Robot feedback remained invalid beyond the configured HOLD window."""


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    robot = parser.add_argument_group("robot")
    robot.add_argument("--robot-port", default="/dev/ttyACM0")
    robot.add_argument("--robot-id", default="rebot_b601_vr")
    robot.add_argument("--can-adapter", choices=("damiao", "socketcan"), default="damiao")
    robot.add_argument("--dm-serial-baud", type=int, default=921600)
    robot.add_argument(
        "--gripper-control-mode", choices=("force_pos", "mit"), default="force_pos"
    )
    robot.add_argument(
        "--gripper-torque-ratio",
        type=float,
        default=0.2,
        help="FORCE_POS maximum grip force ratio in [0, 1]",
    )
    robot.add_argument("--max-relative-target-deg", type=float, default=5.0)
    robot.add_argument(
        "--initial-q",
        type=float,
        nargs=6,
        default=tuple(DEFAULT_INITIAL_Q_REFERENCE_RAD),
        metavar=("Q1", "Q2", "Q3", "Q4", "Q5", "Q6"),
        help="initial pose in validated RS-example radians; q2/q3 are converted to DM signs",
    )
    robot.add_argument(
        "--move-to-initial",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="move at bounded speed to --initial-q before accepting VR control",
    )
    robot.add_argument("--initial-move-tolerance-deg", type=float, default=2.0)
    robot.add_argument("--initial-move-timeout", type=float, default=30.0)
    robot.add_argument("--initial-stall-timeout", type=float, default=5.0)
    robot.add_argument("--no-calibrate", action="store_true")
    robot.add_argument(
        "--disable-torque-on-disconnect",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="disable motors when exiting (support the arm before using the default)",
    )

    vr = parser.add_argument_group("VR source")
    vr.add_argument(
        "--backend",
        choices=("xrobotoolkit_v1", "isaac"),
        default="xrobotoolkit_v1",
    )
    vr.add_argument("--hand", choices=("left", "right"), default="right")
    vr.add_argument("--host", default="0.0.0.0")
    vr.add_argument("--port", type=int, default=63901)
    vr.add_argument("--no-cloudxr-launch", action="store_true")
    vr.add_argument("--stale-timeout", type=float, default=0.2)
    vr.add_argument("--grip-press", type=float, default=0.85)
    vr.add_argument("--grip-release", type=float, default=0.75)

    mapping = parser.add_argument_group("Cartesian mapping")
    mapping.add_argument("--position-scale", type=float, default=1.0)
    mapping.add_argument("--orientation-scale", type=float, default=1.0)
    mapping.add_argument("--position-filter-hz", type=float, default=8.0)
    mapping.add_argument("--orientation-filter-hz", type=float, default=6.0)
    mapping.add_argument("--position-deadband-m", type=float, default=5e-4)
    mapping.add_argument("--orientation-deadband-deg", type=float, default=0.25)

    ik = parser.add_argument_group("IK and safety")
    ik.add_argument("--urdf", type=Path)
    ik.add_argument("--ik-rate", type=float, default=100.0)
    ik.add_argument("--ik-max-iterations", type=int, default=50)
    ik.add_argument("--ik-tolerance-m", type=float, default=5e-4)
    ik.add_argument("--ik-damping", type=float, default=1e-4)
    ik.add_argument("--max-solution-jump-rad", type=float, default=0.5)
    ik.add_argument("--max-joint-speed-rad-s", type=float, default=2.0)
    ik.add_argument("--max-joint-acceleration-rad-s2", type=float, default=8.0)
    ik.add_argument(
        "--wrist-speed-rad-s",
        type=float,
        default=None,
        help="q4-q6 speed limit; defaults to --max-joint-speed-rad-s",
    )
    ik.add_argument(
        "--wrist-acceleration-rad-s2",
        type=float,
        default=None,
        help="q4-q6 acceleration limit; defaults to --max-joint-acceleration-rad-s2",
    )
    ik.add_argument(
        "--wrist-relative-target-deg",
        type=float,
        default=None,
        help="q4-q6 follower relative target limit; defaults to --max-relative-target-deg",
    )
    ik.add_argument("--feedback-fault-max-consecutive", type=int, default=5)
    ik.add_argument("--feedback-fault-settle-time", type=float, default=0.25)
    ik.add_argument("--gripper-max-speed-deg-s", type=float, default=3000.0)
    ik.add_argument("--gripper-max-acceleration-deg-s2", type=float, default=5000.0)
    ik.add_argument("--gripper-open-deg", type=float, default=-180.0)
    ik.add_argument("--gripper-closed-deg", type=float, default=0.0)

    runtime = parser.add_argument_group("runtime")
    runtime.add_argument("--fps", type=float, default=60.0)
    runtime.add_argument("--duration", type=float, default=0.0, help="0 runs until Ctrl-C")
    runtime.add_argument("--status-rate", type=float, default=1.0)
    runtime.add_argument("--log-level", choices=("DEBUG", "INFO", "WARNING"), default="INFO")
    return parser


def _validate_args(args: argparse.Namespace) -> None:
    positive = {
        "stale-timeout": args.stale_timeout,
        "ik-rate": args.ik_rate,
        "ik-tolerance-m": args.ik_tolerance_m,
        "max-solution-jump-rad": args.max_solution_jump_rad,
        "max-joint-speed-rad-s": args.max_joint_speed_rad_s,
        "max-joint-acceleration-rad-s2": args.max_joint_acceleration_rad_s2,
        "max-relative-target-deg": args.max_relative_target_deg,
        "gripper-max-speed-deg-s": args.gripper_max_speed_deg_s,
        "gripper-max-acceleration-deg-s2": args.gripper_max_acceleration_deg_s2,
        "initial-move-tolerance-deg": args.initial_move_tolerance_deg,
        "initial-move-timeout": args.initial_move_timeout,
        "initial-stall-timeout": args.initial_stall_timeout,
        "fps": args.fps,
        "status-rate": args.status_rate,
    }
    invalid = [name for name, value in positive.items() if not np.isfinite(value) or value <= 0.0]
    if invalid:
        raise ValueError(f"the following parameters must be positive: {', '.join(invalid)}")
    wrist_positive = {
        "wrist-speed-rad-s": args.wrist_speed_rad_s,
        "wrist-acceleration-rad-s2": args.wrist_acceleration_rad_s2,
        "wrist-relative-target-deg": args.wrist_relative_target_deg,
    }
    invalid = [
        name
        for name, value in wrist_positive.items()
        if value is not None and (not np.isfinite(value) or value <= 0.0)
    ]
    if invalid:
        raise ValueError(f"the following parameters must be positive: {', '.join(invalid)}")
    non_negative = {
        "position-scale": args.position_scale,
        "orientation-scale": args.orientation_scale,
        "position-filter-hz": args.position_filter_hz,
        "orientation-filter-hz": args.orientation_filter_hz,
        "position-deadband-m": args.position_deadband_m,
        "orientation-deadband-deg": args.orientation_deadband_deg,
        "feedback-fault-settle-time": args.feedback_fault_settle_time,
    }
    invalid = [
        name for name, value in non_negative.items() if not np.isfinite(value) or value < 0.0
    ]
    if invalid:
        raise ValueError(f"the following parameters must be non-negative: {', '.join(invalid)}")
    if args.duration < 0.0:
        raise ValueError("duration must be non-negative")
    if args.ik_max_iterations <= 0 or args.ik_damping < 0.0:
        raise ValueError("IK iterations must be positive and damping must be non-negative")
    if args.feedback_fault_max_consecutive <= 0:
        raise ValueError("feedback-fault-max-consecutive must be positive")
    if not 0.0 <= args.gripper_torque_ratio <= 1.0:
        raise ValueError("gripper-torque-ratio must be in [0, 1]")
    if not np.all(np.isfinite([args.gripper_open_deg, args.gripper_closed_deg])):
        raise ValueError("gripper open and closed positions must be finite")
    if not -270.0 <= args.gripper_open_deg < args.gripper_closed_deg <= 0.0:
        raise ValueError(
            "gripper positions must satisfy -270 <= open < closed <= 0 degrees"
        )


def _follower_pos_vel_velocity(
    arm_speed_rad_s: float,
    wrist_speed_rad_s: float,
    gripper_speed_deg_s: float,
) -> list[float]:
    return [
        *([float(np.rad2deg(arm_speed_rad_s))] * 3),
        *([float(np.rad2deg(wrist_speed_rad_s))] * 3),
        gripper_speed_deg_s,
    ]


def _follower_relative_target(
    arm_relative_target_deg: float,
    wrist_relative_target_deg: float,
) -> float | dict[str, float]:
    if wrist_relative_target_deg == arm_relative_target_deg:
        return arm_relative_target_deg
    return {
        "shoulder_pan": arm_relative_target_deg,
        "shoulder_lift": arm_relative_target_deg,
        "elbow_flex": arm_relative_target_deg,
        "wrist_flex": wrist_relative_target_deg,
        "wrist_yaw": wrist_relative_target_deg,
        "wrist_roll": wrist_relative_target_deg,
        "gripper": arm_relative_target_deg,
    }


def _status_line(
    status: CartesianControlStatus, sent_action: dict[str, float] | None = None
) -> str:
    ik = "pending" if status.ik_success is None else (
        f"ok err={status.ik_error_m:.5f}m" if status.ik_success else f"hold {status.ik_reason}"
    )
    sent_gripper_deg = (
        status.gripper_command_deg
        if sent_action is None
        else float(sent_action.get(f"{GRIPPER_NAME}.pos", status.gripper_command_deg))
    )
    orientation_error = (
        "n/a"
        if status.orientation_error_deg is None
        else f"{status.orientation_error_deg:.3f}"
    )
    feedback = (
        "ok"
        if status.feedback_valid
        else (
            f"HOLD {status.feedback_fault_count} "
            f"reason={status.feedback_fault_reason}"
        )
    )
    return (
        f"state={status.state.value:<7} tracking={status.tracking} ik={ik} "
        f"feedback={feedback} "
        f"jobs={status.submitted}/{status.solved}/{status.rejected} "
        f"A={status.primary_button} home={status.home_requested} "
        f"B={status.secondary_button} zero={status.zero_requested}\n"
        f"  trigger={status.trigger:.3f} gripper_deg(actual/target/shaped/sent)="
        f"{status.gripper_actual_deg:.1f}/{status.gripper_target_deg:.1f}/"
        f"{status.gripper_command_deg:.1f}/{sent_gripper_deg:.1f}\n"
        f"  wrist_deg[q4/q5/q6] actual/target/command="
        f"{np.array2string(status.actual_deg[3:6], precision=1)}/"
        f"{np.array2string(status.target_deg[3:6], precision=1)}/"
        f"{np.array2string(status.command_deg[3:6], precision=1)}\n"
        f"  orientation_error_deg={orientation_error} "
        f"wrist_clip_deg={status.wrist_clip_deg:.3f}\n"
        f"  actual_deg={np.array2string(status.actual_deg, precision=1, suppress_small=True)}\n"
        f"  target_deg={np.array2string(status.target_deg, precision=1, suppress_small=True)}\n"
        f"  command_deg={np.array2string(status.command_deg, precision=1, suppress_small=True)}"
    )


def _move_to_initial_pose(
    robot,
    *,
    target_rad: np.ndarray,
    lower_limit_rad: np.ndarray,
    upper_limit_rad: np.ndarray,
    args: argparse.Namespace,
    should_stop,
) -> bool:
    mover = StartupPoseMover(
        target_rad,
        lower_limit_rad=lower_limit_rad,
        upper_limit_rad=upper_limit_rad,
        max_speed_rad_s=args.max_joint_speed_rad_s,
        max_acceleration_rad_s2=args.max_joint_acceleration_rad_s2,
        tolerance_rad=np.deg2rad(args.initial_move_tolerance_deg),
        max_command_feedback_error_rad=np.deg2rad(
            args.max_relative_target_deg * 0.9
        ),
    )
    started_s = time.monotonic()
    previous_loop_s = started_s
    next_status_s = started_s
    last_progress_s = started_s
    best_error_rad = float("inf")
    progress_threshold_rad = min(
        np.deg2rad(0.5),
        args.max_joint_speed_rad_s * args.initial_stall_timeout * 0.25,
    )
    print(
        "Moving to B601-DM initial pose (rad): "
        f"{np.array2string(target_rad, precision=3, suppress_small=True)}",
        flush=True,
    )
    while not should_stop():
        loop_started_s = time.monotonic()
        if loop_started_s - started_s >= args.initial_move_timeout:
            raise RuntimeError(
                "initial-pose motion timed out; check motor feedback, calibration, and limits"
            )
        observation = robot.get_observation()
        actual_rad = np.deg2rad(
            np.array([float(observation[f"{name}.pos"]) for name in ARM_JOINT_NAMES])
        )
        gripper_actual_deg = float(observation[f"{GRIPPER_NAME}.pos"])
        status = mover.update(actual_rad, loop_started_s - previous_loop_s)
        previous_loop_s = loop_started_s
        command_deg = np.rad2deg(status.command_rad)
        action = {
            f"{name}.pos": float(command_deg[index])
            for index, name in enumerate(ARM_JOINT_NAMES)
        }
        action[f"{GRIPPER_NAME}.pos"] = gripper_actual_deg
        sent_action = robot.send_action(action)
        sent_deg = np.array(
            [float(sent_action[f"{name}.pos"]) for name in ARM_JOINT_NAMES]
        )

        if status.max_actual_error_rad < best_error_rad - progress_threshold_rad:
            best_error_rad = status.max_actual_error_rad
            last_progress_s = loop_started_s
        elif (
            loop_started_s - last_progress_s >= args.initial_stall_timeout
            and status.max_actual_error_rad > np.deg2rad(args.initial_move_tolerance_deg)
        ):
            raise RuntimeError(
                "initial-pose feedback is not following the command; "
                "check that motors are enabled and verify calibration. "
                f"actual_deg={np.array2string(np.rad2deg(actual_rad), precision=1)} "
                f"command_deg={np.array2string(command_deg, precision=1)} "
                f"sent_deg={np.array2string(sent_deg, precision=1)}"
            )

        if loop_started_s >= next_status_s:
            print(
                "initial_move "
                f"error={np.rad2deg(status.max_actual_error_rad):.1f}deg "
                f"actual={np.array2string(np.rad2deg(actual_rad), precision=1)} "
                f"command={np.array2string(command_deg, precision=1)} "
                f"sent={np.array2string(sent_deg, precision=1)}",
                flush=True,
            )
            next_status_s = loop_started_s + 1.0 / args.status_rate
        if status.done:
            print("Initial pose reached; VR control is now enabled.", flush=True)
            return True
        sleep_s = 1.0 / args.fps - (time.monotonic() - loop_started_s)
        if sleep_s > 0.0:
            time.sleep(sleep_s)
    return False


def _feedback_hold_action(
    observation: dict[str, float],
    fallback_action: dict[str, float] | None,
) -> dict[str, float] | None:
    """Target current finite feedback, falling back per joint to the last command."""
    action: dict[str, float] = {}
    for name in (*ARM_JOINT_NAMES, GRIPPER_NAME):
        key = f"{name}.pos"
        try:
            value = float(observation[key])
        except (KeyError, TypeError, ValueError):
            value = float("nan")
        if not np.isfinite(value) and fallback_action is not None:
            try:
                value = float(fallback_action[key])
            except (KeyError, TypeError, ValueError):
                value = float("nan")
        if not np.isfinite(value):
            return None
        action[key] = value
    return action


def _send_feedback_hold_action(
    robot, action: dict[str, float]
) -> dict[str, float]:
    """Send a known-safe HOLD command without re-clipping against bad feedback."""
    config = getattr(robot, "config", None)
    if config is None or not hasattr(config, "max_relative_target"):
        return robot.send_action(action)
    previous_limit = config.max_relative_target
    config.max_relative_target = None
    try:
        return robot.send_action(action)
    finally:
        config.max_relative_target = previous_limit


def _settle_persistent_feedback_fault(
    robot,
    observation: dict[str, float],
    fallback_action: dict[str, float] | None,
    *,
    duration_s: float,
    fps: float,
) -> None:
    """Converge the retained command to available feedback before leaving the loop."""
    deadline_s = time.monotonic() + duration_s
    latest_observation = observation
    retained_action = fallback_action
    first = True
    while first or time.monotonic() < deadline_s:
        first = False
        action = _feedback_hold_action(latest_observation, retained_action)
        if action is not None:
            try:
                retained_action = _send_feedback_hold_action(robot, action)
            except Exception:
                logger.exception("failed to refresh the feedback-fault HOLD command")
                return
        remaining_s = deadline_s - time.monotonic()
        if remaining_s <= 0.0:
            return
        time.sleep(min(1.0 / fps, remaining_s))
        try:
            latest_observation = robot.get_observation()
        except Exception:
            logger.exception("failed to refresh feedback while settling HOLD")
            return


def main() -> None:
    args = _parser().parse_args()
    _validate_args(args)
    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )

    # Build the model before opening the CAN device so dependency/model errors
    # cannot leave a powered robot connected without a control loop.
    kinematics = B601Kinematics(args.urdf, "gripper_end")
    vr_config = RebotVRConfig(
        vr_backend=args.backend,
        hand_side=args.hand,
        clutch_threshold=args.grip_press,
        clutch_release_threshold=args.grip_release,
        stale_timeout=args.stale_timeout,
        ws_host=args.host,
        ws_port=args.port,
        auto_launch_cloudxr=not args.no_cloudxr_launch,
    )
    wrist_speed = (
        args.wrist_speed_rad_s
        if args.wrist_speed_rad_s is not None
        else args.max_joint_speed_rad_s
    )
    wrist_acceleration = (
        args.wrist_acceleration_rad_s2
        if args.wrist_acceleration_rad_s2 is not None
        else args.max_joint_acceleration_rad_s2
    )
    wrist_rel_target = (
        args.wrist_relative_target_deg
        if args.wrist_relative_target_deg is not None
        else args.max_relative_target_deg
    )
    control_config = CartesianControlConfig(
        position_scale=args.position_scale,
        orientation_scale=args.orientation_scale,
        position_filter_hz=args.position_filter_hz,
        orientation_filter_hz=args.orientation_filter_hz,
        position_deadband_m=args.position_deadband_m,
        orientation_deadband_rad=np.deg2rad(args.orientation_deadband_deg),
        grip_press_threshold=args.grip_press,
        grip_release_threshold=args.grip_release,
        stale_timeout_s=args.stale_timeout,
        ik_rate_hz=args.ik_rate,
        ik_max_iterations=args.ik_max_iterations,
        ik_tolerance_m=args.ik_tolerance_m,
        ik_damping=args.ik_damping,
        max_solution_jump_rad=args.max_solution_jump_rad,
        max_joint_speed_rad_s=args.max_joint_speed_rad_s,
        max_joint_acceleration_rad_s2=args.max_joint_acceleration_rad_s2,
        wrist_speed_rad_s=wrist_speed,
        wrist_acceleration_rad_s2=wrist_acceleration,
        wrist_command_feedback_error_deg=wrist_rel_target * 0.9,
        feedback_fault_max_consecutive=args.feedback_fault_max_consecutive,
        initial_q_rad=tuple(args.initial_q),
        gripper_max_speed_deg_s=args.gripper_max_speed_deg_s,
        gripper_max_acceleration_deg_s2=args.gripper_max_acceleration_deg_s2,
        gripper_open_deg=args.gripper_open_deg,
        gripper_closed_deg=args.gripper_closed_deg,
        max_command_feedback_error_deg=args.max_relative_target_deg * 0.9,
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
        gripper_control_mode=args.gripper_control_mode,
        gripper_torque_ratio=args.gripper_torque_ratio,
        pos_vel_velocity=_follower_pos_vel_velocity(
            args.max_joint_speed_rad_s,
            wrist_speed,
            args.gripper_max_speed_deg_s,
        ),
        max_relative_target=_follower_relative_target(
            args.max_relative_target_deg,
            wrist_rel_target,
        ),
        disable_torque_on_disconnect=args.disable_torque_on_disconnect,
    )
    robot = RebotB601Follower(robot_config)
    vr_controller = make_vr_controller(vr_config)
    arm_controller = SplitArmWristController(
        kinematics,
        xr_to_base_rotation=np.asarray(DEFAULT_BASE_T_ANCHOR, dtype=np.float64)[:3, :3],
        config=control_config,
    )

    stop = False

    def stop_now(_signal_number, _frame) -> None:
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, stop_now)
    signal.signal(signal.SIGTERM, stop_now)

    vr_connected = False
    robot_connected = False
    arm_started = False
    preserve_torque_for_feedback_fault = False
    print("Support the arm before exit: torque is disabled on disconnect by default.")
    print("Release Grip fully after tracking starts; hold Grip only when ready to move.")
    initial_target_rad = None
    if args.move_to_initial:
        initial_target_rad = arm_controller.home_q_rad.copy()
        print(
            "Initial pose RS reference (rad): "
            f"{np.array2string(np.asarray(args.initial_q), precision=3, suppress_small=True)}; "
            "q2/q3 are sign-converted for B601-DM."
        )
    else:
        print("Initial-pose motion is disabled; VR control will start from actual feedback.")
    try:
        vr_controller.connect()
        vr_connected = True
        robot.connect(calibrate=not args.no_calibrate)
        robot_connected = True
        if args.move_to_initial:
            assert initial_target_rad is not None
            reached = _move_to_initial_pose(
                robot,
                target_rad=initial_target_rad,
                lower_limit_rad=arm_controller.lower_limit_rad,
                upper_limit_rad=arm_controller.upper_limit_rad,
                args=args,
                should_stop=lambda: stop,
            )
            if not reached:
                return
        arm_controller.start()
        arm_started = True

        started_s = time.monotonic()
        previous_loop_s = started_s
        next_status_s = started_s
        while not stop:
            loop_started_s = time.monotonic()
            if args.duration and loop_started_s - started_s >= args.duration:
                break
            dt_s = loop_started_s - previous_loop_s
            previous_loop_s = loop_started_s
            try:
                observation = robot.get_observation()
            except Exception as exc:
                logger.warning(
                    "robot feedback read failed; entering transient HOLD: %s",
                    exc,
                )
                observation = {}
            sample = vr_controller.latest_sample()
            frame = (
                sample
                if sample is not None or args.backend == "xrobotoolkit_v1"
                else vr_frame_from_raw_action(vr_controller.get_action())
            )
            action, status = arm_controller.update(frame, observation, dt_s)
            if action is None:
                sent_action = None
            elif status.feedback_valid:
                sent_action = robot.send_action(action)
            else:
                sent_action = _send_feedback_hold_action(robot, action)

            if status.feedback_abort_requested:
                print(_status_line(status, sent_action), flush=True)
                preserve_torque_for_feedback_fault = True
                _settle_persistent_feedback_fault(
                    robot,
                    observation,
                    sent_action if sent_action is not None else action,
                    duration_s=args.feedback_fault_settle_time,
                    fps=args.fps,
                )
                raise PersistentFeedbackFault(
                    "robot feedback remained invalid for "
                    f"{status.feedback_fault_count} consecutive frames: "
                    f"{status.feedback_fault_reason}"
                )

            if loop_started_s >= next_status_s:
                print(_status_line(status, sent_action), flush=True)
                next_status_s = loop_started_s + 1.0 / args.status_rate
            sleep_s = 1.0 / args.fps - (time.monotonic() - loop_started_s)
            if sleep_s > 0.0:
                time.sleep(sleep_s)
    finally:
        try:
            if arm_started:
                arm_controller.stop()
        finally:
            try:
                if vr_connected:
                    vr_controller.disconnect()
            finally:
                try:
                    if robot_connected or robot.is_connected:
                        if preserve_torque_for_feedback_fault:
                            print(
                                "Persistent feedback fault: retaining motor torque at the "
                                "last HOLD command; support the arm before disabling power.",
                                flush=True,
                            )
                            robot.config.disable_torque_on_disconnect = False
                        robot.disconnect()
                finally:
                    kinematics.close()


if __name__ == "__main__":
    main()

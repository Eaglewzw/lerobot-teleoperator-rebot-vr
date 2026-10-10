"""Startup and feedback-fault helpers for real-robot teleoperation."""

from __future__ import annotations

import logging
import time
from copy import copy
from collections import deque
from typing import Callable

import numpy as np

from ..control.startup import StartupPoseMover
from ..control.feedback import read_robot_feedback
from ..control.mit import MITCommandDispatcher
from ..control.types import ARM_JOINT_NAMES, GRIPPER_NAME
from ..diagnostics.motion import MotionRecorder, cached_motor_row, mit_command_row

logger = logging.getLogger(__name__)


class PersistentFeedbackFault(RuntimeError):
    """Robot feedback remained invalid beyond the configured HOLD window."""


class JointProgressWindow:
    """Check each continuously out-of-tolerance axis over a recent time window.

    An old best value (or a different worst joint) must not hide recovery after
    a transient overshoot. Sustained divergence still fails after one window.
    """

    def __init__(self, tolerance_rad, window_s, improvement_rad):
        self.tolerance_rad = tolerance_rad
        self.window_s = window_s
        self.improvement_rad = improvement_rad
        self.history = [deque() for _ in ARM_JOINT_NAMES]
        self.elapsed_s = np.zeros(6)

    def update(self, errors, now_s):
        stalled = []
        for i, (error, history) in enumerate(zip(errors, self.history, strict=True)):
            if error <= self.tolerance_rad:
                history.clear()
                self.elapsed_s[i] = 0.
                continue
            history.append((now_s, float(error)))
            # Keep the sample immediately before the window boundary.
            while len(history) > 1 and history[1][0] <= now_s - self.window_s:
                history.popleft()
            self.elapsed_s[i] = now_s - history[0][0]
            # Measure recovery from the recent peak, not from the first
            # threshold-crossing sample (which can precede an overshoot).
            recent_peak = max(value for _, value in history)
            if (self.elapsed_s[i] >= self.window_s
                    and recent_peak - error < self.improvement_rad):
                stalled.append(i)
        return stalled


def move_to_initial_pose(robot, *, target_rad: np.ndarray, lower_limit_rad: np.ndarray,
                         upper_limit_rad: np.ndarray, args, should_stop: Callable[[], bool],
                         write_row=None, arm_id=None) -> bool:
    return _move_to_joint_pose(
        robot, target_rad=target_rad, lower_limit_rad=lower_limit_rad,
        upper_limit_rad=upper_limit_rad, args=args, should_stop=should_stop, phase="initial",
        write_row=write_row, arm_id=arm_id,
    )


def move_to_zero_pose(robot, *, lower_limit_rad: np.ndarray, upper_limit_rad: np.ndarray,
                      args, should_stop: Callable[[], bool], write_row=None, arm_id=None) -> bool:
    """Return six joints to calibrated zero, holding the gripper at entry position."""
    if np.any(np.asarray(lower_limit_rad) > 0) or np.any(np.asarray(upper_limit_rad) < 0):
        raise RuntimeError("zero pose is outside the configured joint limits")
    limits = copy(args)
    limits.initial_move_tolerance_deg = args.exit_zero_tolerance_deg
    zero_stall_timeout = getattr(args, "exit_zero_stall_timeout", None)
    if zero_stall_timeout is not None:
        limits.initial_stall_timeout = zero_stall_timeout
    limits.max_joint_speed_rad_s = min(
        args.exit_zero_speed_rad_s, args.max_joint_speed_rad_s,
        args.wrist_speed_rad_s or args.max_joint_speed_rad_s,
    )
    limits.max_joint_acceleration_rad_s2 = min(
        args.exit_zero_acceleration_rad_s2, args.max_joint_acceleration_rad_s2,
        args.wrist_acceleration_rad_s2 or args.max_joint_acceleration_rad_s2,
    )
    limits.max_relative_target_deg = min(
        args.max_relative_target_deg, args.wrist_relative_target_deg or args.max_relative_target_deg,
    )
    config = getattr(robot, "config", None)
    original_velocity = getattr(config, "pos_vel_velocity", None)
    if original_velocity is not None:
        velocities = original_velocity if isinstance(original_velocity, list) else [original_velocity] * 7
        config.pos_vel_velocity = [
            min(value, float(np.rad2deg(limits.max_joint_speed_rad_s))) if index < 6 else value
            for index, value in enumerate(velocities)
        ]
    try:
        if isinstance(robot, MITCommandDispatcher):
            robot.stop_arm_velocity(immediate=True)
            robot.set_zero_return(True)
        return _move_to_joint_pose(
            robot, target_rad=np.zeros(6), lower_limit_rad=lower_limit_rad,
            upper_limit_rad=upper_limit_rad, args=limits, should_stop=should_stop, phase="zero",
            write_row=write_row, arm_id=arm_id,
        )
    finally:
        if isinstance(robot, MITCommandDispatcher):
            robot.set_zero_return(False)
        if original_velocity is not None:
            config.pos_vel_velocity = original_velocity


def _move_to_joint_pose(robot, *, target_rad: np.ndarray, lower_limit_rad: np.ndarray,
                        upper_limit_rad: np.ndarray, args, should_stop: Callable[[], bool], phase: str,
                        write_row=None, arm_id=None) -> bool:
    recorder = MotionRecorder(write_row, arm_id=arm_id,
                              phase="startup" if phase == "initial" else "exit_zero",
                              tolerance_deg=args.initial_move_tolerance_deg)
    recorder.event("started")
    try:
        reached = _execute_joint_pose(
            robot, target_rad=target_rad, lower_limit_rad=lower_limit_rad,
            upper_limit_rad=upper_limit_rad, args=args, should_stop=should_stop,
            phase=phase, recorder=recorder, arm_id=arm_id,
        )
    except BaseException as exc:
        recorder.event("failed", str(exc))
        raise
    recorder.event("completed" if reached else "interrupted")
    return reached


def _execute_joint_pose(robot, *, target_rad, lower_limit_rad, upper_limit_rad,
                        args, should_stop, phase, recorder, arm_id):
    mover = StartupPoseMover(
        target_rad, lower_limit_rad=lower_limit_rad, upper_limit_rad=upper_limit_rad,
        max_speed_rad_s=args.max_joint_speed_rad_s,
        max_acceleration_rad_s2=args.max_joint_acceleration_rad_s2,
        tolerance_rad=np.deg2rad(args.initial_move_tolerance_deg),
        max_command_feedback_error_rad=np.deg2rad(args.max_relative_target_deg * 0.9),
        settle_time_s=0.5 if phase == "zero" else 0.0,
        settle_speed_rad_s=np.deg2rad(1.0) if phase == "zero" else None,
        brake_at_target=getattr(args, "robot_model", None) == "b601_rs",
    )
    started_s = time.monotonic()
    previous_loop_s = started_s
    next_status_s = started_s
    last_progress_s = started_s
    best_error_rad = float("inf")
    joint_best_error_rad = np.full(6, np.inf)
    joint_last_progress_s = np.full(6, started_s)
    gripper_hold_deg = None
    progress_threshold_rad = min(np.deg2rad(0.5), np.deg2rad(args.initial_move_tolerance_deg * 0.25),
                                 args.max_joint_speed_rad_s * args.initial_stall_timeout * 0.25)
    if phase == "zero" and getattr(args, "robot_model", None) == "b601_rs":
        # Slow near-zero trim may improve by <0.125 deg over the 5s stall
        # window. Count smaller measured progress without relaxing arrival.
        progress_threshold_rad = min(progress_threshold_rad,
                                     np.deg2rad(args.initial_move_tolerance_deg * 0.05))
    progress_window = (JointProgressWindow(np.deg2rad(args.initial_move_tolerance_deg),
                                         args.initial_stall_timeout, progress_threshold_rad)
                       if phase == "zero" and getattr(args, "robot_model", None) == "b601_rs"
                       else None)
    prefix = "" if arm_id is None else f"[{arm_id}] "
    print(prefix + f"Moving to {phase} pose (rad): " + np.array2string(target_rad, precision=3, suppress_small=True), flush=True)
    while not should_stop():
        loop_started_s = time.monotonic()
        if loop_started_s - started_s >= args.initial_move_timeout:
            raise RuntimeError(f"{phase}-pose motion timed out; check motor feedback, calibration, and limits")
        feedback_started_ns = time.monotonic_ns()
        observation = robot.get_observation()
        feedback_finished_ns = time.monotonic_ns()
        feedback_row = cached_motor_row(robot) if recorder.write_row is not None else {}
        actual_rad, gripper_actual_deg, feedback_error = read_robot_feedback(observation)
        if feedback_error:
            raise RuntimeError(f"{phase}-pose feedback is invalid: {feedback_error}")
        if gripper_hold_deg is None:
            gripper_hold_deg = gripper_actual_deg
        status = mover.update(actual_rad, loop_started_s - previous_loop_s)
        previous_loop_s = loop_started_s
        command_deg = np.rad2deg(status.command_rad)
        action = {f"{name}.pos": float(command_deg[index]) for index, name in enumerate(ARM_JOINT_NAMES)}
        action[f"{GRIPPER_NAME}.pos"] = gripper_hold_deg if phase == "zero" else gripper_actual_deg
        # A stop received during feedback/FK must prevent another movement command.
        if should_stop():
            return False
        send_started_ns = time.monotonic_ns()
        sent_action = robot.send_action(action)
        send_finished_ns = time.monotonic_ns()
        sent_deg = np.array([float(sent_action[f"{name}.pos"]) for name in ARM_JOINT_NAMES])
        errors = np.abs(target_rad - actual_rad)
        improved = errors < joint_best_error_rad - progress_threshold_rad
        joint_best_error_rad[improved] = errors[improved]
        joint_last_progress_s[improved] = loop_started_s
        stalled_axes = [] if progress_window is None else progress_window.update(errors, loop_started_s)
        if recorder.write_row is not None:
            row = {
                **feedback_row, **mit_command_row(robot, np.rad2deg(actual_rad), sent_deg, feedback_row),
                "motion_max_error_deg": float(np.rad2deg(status.max_actual_error_rad)),
                "feedback_started_monotonic_ns": feedback_started_ns,
                "feedback_finished_monotonic_ns": feedback_finished_ns,
                "command_send_started_monotonic_ns": send_started_ns,
                "command_send_finished_monotonic_ns": send_finished_ns,
                "feedback_read_ms": (feedback_finished_ns - feedback_started_ns) * 1e-6,
                "send_action_ms": (send_finished_ns - send_started_ns) * 1e-6,
                "actual_gripper_deg": gripper_actual_deg,
                "target_gripper_deg": action[f"{GRIPPER_NAME}.pos"],
                "command_gripper_deg": sent_action[f"{GRIPPER_NAME}.pos"],
            }
            for i, joint in enumerate(ARM_JOINT_NAMES):
                row.update({
                    f"actual_{joint}_deg": float(np.rad2deg(actual_rad[i])),
                    f"target_{joint}_deg": float(np.rad2deg(target_rad[i])),
                    f"command_{joint}_deg": float(command_deg[i]),
                    f"sent_{joint}_deg": float(sent_deg[i]),
                    f"error_{joint}_deg": float(np.rad2deg(target_rad[i] - actual_rad[i])),
                    f"stalled_{joint}_s": float(loop_started_s - joint_last_progress_s[i]
                                               if progress_window is None else progress_window.elapsed_s[i]),
                    f"command_clipped_{joint}_flag": bool(abs(sent_deg[i] - command_deg[i]) > 1e-6),
                })
            # Log the failing sample BEFORE the stall check raises.
            recorder.sample(row)
        if status.max_actual_error_rad < best_error_rad - progress_threshold_rad:
            best_error_rad = status.max_actual_error_rad
            last_progress_s = loop_started_s
        stall_detected = (bool(stalled_axes) if progress_window is not None else
                          loop_started_s - last_progress_s >= args.initial_stall_timeout
                          and status.max_actual_error_rad > np.deg2rad(args.initial_move_tolerance_deg))
        if stall_detected:
            outside = [f"{joint}={np.rad2deg(errors[i]):.4f}deg"
                       for i, joint in enumerate(ARM_JOINT_NAMES)
                       if errors[i] > np.deg2rad(args.initial_move_tolerance_deg)]
            raise RuntimeError(f"{phase}-pose feedback is not following the command; check that motors are enabled and verify calibration. "
                               f"outside_tolerance=[{', '.join(outside)}] "
                               f"tolerance={args.initial_move_tolerance_deg:.4f}deg; "
                               f"no improvement >= {np.rad2deg(progress_threshold_rad):.4f}deg "
                               f"over {args.initial_stall_timeout:.2f}s; "
                               f"stalled_joints={[ARM_JOINT_NAMES[i] for i in stalled_axes]}; "
                               f"actual_deg={np.array2string(np.rad2deg(actual_rad), precision=4)} "
                               f"command_deg={np.array2string(command_deg, precision=4)} sent_deg={np.array2string(sent_deg, precision=4)}")
        if loop_started_s >= next_status_s:
            print(prefix + f"{phase}_move error={np.rad2deg(status.max_actual_error_rad):.4f}deg "
                  f"tolerance={args.initial_move_tolerance_deg:.4f}deg "
                  f"actual={np.array2string(np.rad2deg(actual_rad), precision=4)} "
                  f"command={np.array2string(command_deg, precision=1)} "
                  f"sent={np.array2string(sent_deg, precision=1)}", flush=True)
            next_status_s = loop_started_s + 1.0 / args.status_rate
        if status.done:
            print(prefix + ("Initial pose reached; VR control is now enabled." if phase == "initial"
                  else "Zero pose feedback settled; applying shutdown torque policy."), flush=True)
            return True
        sleep_s = 1.0 / args.fps - (time.monotonic() - loop_started_s)
        if sleep_s > 0.0:
            time.sleep(sleep_s)
    return False


def feedback_hold_action(observation: dict[str, float], fallback_action: dict[str, float] | None) -> dict[str, float] | None:
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


def send_feedback_hold_action(robot, action: dict[str, float]) -> dict[str, float]:
    if isinstance(robot, MITCommandDispatcher):
        return robot.send_feedback_hold()
    config = getattr(robot, "config", None)
    if config is None or not hasattr(config, "max_relative_target"):
        return robot.send_action(action)
    previous_limit = config.max_relative_target
    config.max_relative_target = None
    try:
        return robot.send_action(action)
    finally:
        config.max_relative_target = previous_limit


def settle_persistent_feedback_fault(robot, observation: dict[str, float], fallback_action: dict[str, float] | None,
                                     *, duration_s: float, fps: float) -> None:
    deadline_s = time.monotonic() + duration_s
    latest_observation = observation
    retained_action = fallback_action
    first = True
    while first or time.monotonic() < deadline_s:
        first = False
        action = feedback_hold_action(latest_observation, retained_action)
        if action is not None:
            try:
                retained_action = send_feedback_hold_action(robot, action)
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

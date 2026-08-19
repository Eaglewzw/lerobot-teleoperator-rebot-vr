"""Startup and feedback-fault helpers for real-robot teleoperation."""

from __future__ import annotations

import logging
import time
from typing import Callable

import numpy as np

from .cartesian_controller import ARM_JOINT_NAMES, GRIPPER_NAME
from .startup_pose import StartupPoseMover

logger = logging.getLogger(__name__)


class PersistentFeedbackFault(RuntimeError):
    """Robot feedback remained invalid beyond the configured HOLD window."""


def move_to_initial_pose(robot, *, target_rad: np.ndarray, lower_limit_rad: np.ndarray,
                         upper_limit_rad: np.ndarray, args, should_stop: Callable[[], bool]) -> bool:
    mover = StartupPoseMover(
        target_rad, lower_limit_rad=lower_limit_rad, upper_limit_rad=upper_limit_rad,
        max_speed_rad_s=args.max_joint_speed_rad_s,
        max_acceleration_rad_s2=args.max_joint_acceleration_rad_s2,
        tolerance_rad=np.deg2rad(args.initial_move_tolerance_deg),
        max_command_feedback_error_rad=np.deg2rad(args.max_relative_target_deg * 0.9),
    )
    started_s = time.monotonic()
    previous_loop_s = started_s
    next_status_s = started_s
    last_progress_s = started_s
    best_error_rad = float("inf")
    progress_threshold_rad = min(np.deg2rad(0.5), args.max_joint_speed_rad_s * args.initial_stall_timeout * 0.25)
    print("Moving to B601-DM initial pose (rad): " + np.array2string(target_rad, precision=3, suppress_small=True), flush=True)
    while not should_stop():
        loop_started_s = time.monotonic()
        if loop_started_s - started_s >= args.initial_move_timeout:
            raise RuntimeError("initial-pose motion timed out; check motor feedback, calibration, and limits")
        observation = robot.get_observation()
        actual_rad = np.deg2rad(np.array([float(observation[f"{name}.pos"]) for name in ARM_JOINT_NAMES]))
        gripper_actual_deg = float(observation[f"{GRIPPER_NAME}.pos"])
        status = mover.update(actual_rad, loop_started_s - previous_loop_s)
        previous_loop_s = loop_started_s
        command_deg = np.rad2deg(status.command_rad)
        action = {f"{name}.pos": float(command_deg[index]) for index, name in enumerate(ARM_JOINT_NAMES)}
        action[f"{GRIPPER_NAME}.pos"] = gripper_actual_deg
        sent_action = robot.send_action(action)
        sent_deg = np.array([float(sent_action[f"{name}.pos"]) for name in ARM_JOINT_NAMES])
        if status.max_actual_error_rad < best_error_rad - progress_threshold_rad:
            best_error_rad = status.max_actual_error_rad
            last_progress_s = loop_started_s
        elif loop_started_s - last_progress_s >= args.initial_stall_timeout and status.max_actual_error_rad > np.deg2rad(args.initial_move_tolerance_deg):
            raise RuntimeError("initial-pose feedback is not following the command; check that motors are enabled and verify calibration. "
                               f"actual_deg={np.array2string(np.rad2deg(actual_rad), precision=1)} "
                               f"command_deg={np.array2string(command_deg, precision=1)} sent_deg={np.array2string(sent_deg, precision=1)}")
        if loop_started_s >= next_status_s:
            print(f"initial_move error={np.rad2deg(status.max_actual_error_rad):.1f}deg "
                  f"actual={np.array2string(np.rad2deg(actual_rad), precision=1)} "
                  f"command={np.array2string(command_deg, precision=1)} "
                  f"sent={np.array2string(sent_deg, precision=1)}", flush=True)
            next_status_s = loop_started_s + 1.0 / args.status_rate
        if status.done:
            print("Initial pose reached; VR control is now enabled.", flush=True)
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

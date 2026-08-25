from __future__ import annotations

from collections.abc import Callable

import numpy as np

from ..vr.pose_mapping import TeleopState
from .types import CartesianControlConfig


def update_arm_position_command(
    *,
    previous_position_rad: np.ndarray,
    previous_velocity_rad_s: np.ndarray,
    target_position_rad: np.ndarray,
    actual_position_rad: np.ndarray,
    state: TeleopState,
    dt_s: float,
    lower_limit_rad: np.ndarray,
    upper_limit_rad: np.ndarray,
    config: CartesianControlConfig,
    shape_fn: Callable[..., tuple[np.ndarray, np.ndarray]],
    bound_fn: Callable[..., np.ndarray],
) -> tuple[np.ndarray, np.ndarray]:
    """Apply the ACTIVE lookahead path or the non-ACTIVE position shaper."""
    wrist_speed = config.max_joint_speed_rad_s
    if config.wrist_speed_rad_s is not None:
        wrist_speed = config.wrist_speed_rad_s
    wrist_acceleration = config.max_joint_acceleration_rad_s2
    if config.wrist_acceleration_rad_s2 is not None:
        wrist_acceleration = config.wrist_acceleration_rad_s2

    if state is TeleopState.ACTIVE:
        # QP velocity already obeys speed and acceleration constraints. Applying
        # the position shaper again would repeatedly stop at short async targets.
        command_rad = np.clip(
            target_position_rad, lower_limit_rad, upper_limit_rad
        )
        velocity_rad_s = (command_rad - previous_position_rad) / dt_s
    else:
        command_rad, velocity_rad_s = shape_fn(
            previous_position=previous_position_rad,
            previous_velocity=previous_velocity_rad_s,
            target_position=target_position_rad,
            dt_s=dt_s,
            max_speed=np.concatenate(
                (
                    np.full(3, config.max_joint_speed_rad_s),
                    np.full(3, wrist_speed),
                )
            ),
            max_acceleration=np.concatenate(
                (
                    np.full(3, config.max_joint_acceleration_rad_s2),
                    np.full(3, wrist_acceleration),
                )
            ),
            lower_limit=lower_limit_rad,
            upper_limit=upper_limit_rad,
        )

    if config.max_command_feedback_error_deg is None:
        return command_rad, velocity_rad_s

    wrist_feedback_error_deg = config.max_command_feedback_error_deg
    if config.wrist_command_feedback_error_deg is not None:
        wrist_feedback_error_deg = config.wrist_command_feedback_error_deg
    max_tracking_error_rad = np.deg2rad(
        np.concatenate(
            (
                np.full(3, config.max_command_feedback_error_deg),
                np.full(3, wrist_feedback_error_deg),
            )
        )
    )
    command_rad = bound_fn(
        command_rad,
        actual_position_rad,
        max_tracking_error_rad,
        lower_limit=lower_limit_rad,
        upper_limit=upper_limit_rad,
    )
    velocity_rad_s = (command_rad - previous_position_rad) / dt_s
    return command_rad, velocity_rad_s

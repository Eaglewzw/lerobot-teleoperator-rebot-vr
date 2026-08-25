from __future__ import annotations

import numpy as np


def bound_position_command_to_feedback(
    command_position: np.ndarray,
    feedback_position: np.ndarray,
    max_error: np.ndarray | float,
    *,
    lower_limit: np.ndarray,
    upper_limit: np.ndarray,
) -> np.ndarray:
    """Keep a position command close enough to feedback for follower safety."""

    command_position = np.asarray(command_position, dtype=np.float64)
    feedback_position = np.asarray(feedback_position, dtype=np.float64)
    max_error = np.broadcast_to(
        np.asarray(max_error, dtype=np.float64), command_position.shape
    )
    if np.any(max_error <= 0.0) or not np.all(np.isfinite(max_error)):
        raise ValueError("max feedback tracking error must be finite and positive")
    bounded = np.clip(
        command_position,
        feedback_position - max_error,
        feedback_position + max_error,
    )
    return np.clip(bounded, lower_limit, upper_limit)


def shape_joint_position_command(
    *,
    previous_position: np.ndarray,
    previous_velocity: np.ndarray,
    target_position: np.ndarray,
    dt_s: float,
    max_speed: np.ndarray,
    max_acceleration: np.ndarray,
    lower_limit: np.ndarray,
    upper_limit: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Apply joint limit, velocity and acceleration constraints to a position target."""

    previous_position = np.asarray(previous_position, dtype=np.float64)
    previous_velocity = np.asarray(previous_velocity, dtype=np.float64)
    target_position = np.asarray(target_position, dtype=np.float64)
    lower_limit = np.asarray(lower_limit, dtype=np.float64)
    upper_limit = np.asarray(upper_limit, dtype=np.float64)
    if (
        previous_position.ndim != 1
        or previous_velocity.shape != previous_position.shape
        or target_position.shape != previous_position.shape
        or lower_limit.shape != previous_position.shape
        or upper_limit.shape != previous_position.shape
    ):
        raise ValueError("joint command inputs must be equal one-dimensional vectors")
    if not all(
        np.all(np.isfinite(value))
        for value in (
            previous_position,
            previous_velocity,
            target_position,
            lower_limit,
            upper_limit,
        )
    ) or np.any(lower_limit > upper_limit):
        raise ValueError("joint command inputs and ordered limits must be finite")
    dt_s = float(dt_s)
    if not np.isfinite(dt_s) or dt_s < 0.0:
        raise ValueError("dt_s must be finite and non-negative")
    max_speed = np.broadcast_to(
        np.asarray(max_speed, dtype=np.float64), previous_position.shape
    )
    max_acceleration = np.broadcast_to(
        np.asarray(max_acceleration, dtype=np.float64), previous_position.shape
    )
    if (
        not np.all(np.isfinite(max_speed))
        or not np.all(np.isfinite(max_acceleration))
        or np.any(max_speed <= 0.0)
        or np.any(max_acceleration <= 0.0)
    ):
        raise ValueError("speed and acceleration limits must be finite and positive")
    target_position = np.clip(target_position, lower_limit, upper_limit)
    if dt_s == 0.0:
        return previous_position.copy(), previous_velocity.copy()

    error = target_position - previous_position
    desired_velocity = np.clip(
        error / dt_s,
        -max_speed,
        max_speed,
    )
    velocity_change = np.clip(
        desired_velocity - previous_velocity,
        -max_acceleration * dt_s,
        max_acceleration * dt_s,
    )
    velocity = previous_velocity + velocity_change
    position = previous_position + velocity * dt_s

    reached = error * (target_position - position) <= 0.0
    position[reached] = target_position[reached]
    velocity[reached] = 0.0
    position = np.clip(position, lower_limit, upper_limit)
    return position, velocity

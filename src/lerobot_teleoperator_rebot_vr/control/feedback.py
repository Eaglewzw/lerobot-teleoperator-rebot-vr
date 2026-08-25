from __future__ import annotations

import numpy as np

from .types import ARM_JOINT_NAMES, GRIPPER_NAME


def read_robot_feedback(
    observation: dict[str, float],
) -> tuple[np.ndarray, float, str]:
    """Parse follower feedback without raising on a malformed CAN sample."""
    q_actual_rad = np.full(6, np.nan, dtype=np.float64)
    gripper_actual_deg = float("nan")
    try:
        q_actual_deg = np.array(
            [float(observation[f"{name}.pos"]) for name in ARM_JOINT_NAMES],
            dtype=np.float64,
        )
        q_actual_rad = np.deg2rad(q_actual_deg)
        gripper_actual_deg = float(observation[f"{GRIPPER_NAME}.pos"])
    except (KeyError, TypeError, ValueError) as exc:
        return q_actual_rad, gripper_actual_deg, f"invalid_fields:{type(exc).__name__}"

    invalid_names = [
        ARM_JOINT_NAMES[index]
        for index in np.flatnonzero(~np.isfinite(q_actual_rad))
    ]
    if not np.isfinite(gripper_actual_deg):
        invalid_names.append(GRIPPER_NAME)
    if invalid_names:
        return q_actual_rad, gripper_actual_deg, "non_finite:" + ",".join(
            invalid_names
        )
    return q_actual_rad, gripper_actual_deg, ""


def feedback_limit_error(
    q_actual_rad: np.ndarray,
    lower_limit_rad: np.ndarray,
    upper_limit_rad: np.ndarray,
) -> str:
    outside = np.flatnonzero(
        (q_actual_rad < lower_limit_rad) | (q_actual_rad > upper_limit_rad)
    )
    if not outside.size:
        return ""
    details = ", ".join(
        f"{ARM_JOINT_NAMES[index]}={np.rad2deg(q_actual_rad[index]):.2f}deg "
        f"(allowed feedback {np.rad2deg(lower_limit_rad[index]):.1f}.."
        f"{np.rad2deg(upper_limit_rad[index]):.1f}deg)"
        for index in outside
    )
    return "outside_limits:" + details

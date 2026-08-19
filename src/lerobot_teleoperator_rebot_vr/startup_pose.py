from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .joint_command import (
    bound_position_command_to_feedback,
    shape_joint_position_command,
)


DEFAULT_INITIAL_Q_REFERENCE_RAD = np.array([0.0, 0.8, 0.8, 0.0, 0.0, 0.0])

# The B601-RS reference convention uses positive q2/q3. The B601-DM follower
# and URDF describe the same physical motion with negative q2/q3.
REFERENCE_TO_DM_SIGN = np.array([1.0, -1.0, -1.0, 1.0, 1.0, 1.0])


def reference_initial_q_to_dm(
    q_reference_rad: np.ndarray | tuple[float, ...] | list[float],
    *,
    lower_limit_rad: np.ndarray,
    upper_limit_rad: np.ndarray,
) -> np.ndarray:
    """Convert the validated RS-example joint convention to B601-DM radians."""

    q_reference_rad = np.asarray(q_reference_rad, dtype=np.float64)
    if q_reference_rad.shape != (6,) or not np.all(np.isfinite(q_reference_rad)):
        raise ValueError("initial-q must contain six finite joint angles in radians")
    q_dm_rad = q_reference_rad * REFERENCE_TO_DM_SIGN
    lower_limit_rad = np.asarray(lower_limit_rad, dtype=np.float64)
    upper_limit_rad = np.asarray(upper_limit_rad, dtype=np.float64)
    outside = np.flatnonzero(
        (q_dm_rad < lower_limit_rad - 1e-9) | (q_dm_rad > upper_limit_rad + 1e-9)
    )
    if outside.size:
        joints = ", ".join(str(index + 1) for index in outside)
        raise ValueError(
            f"initial-q maps outside the B601-DM limits for joint(s): {joints}"
        )
    return q_dm_rad


@dataclass(frozen=True)
class StartupPoseStatus:
    command_rad: np.ndarray
    max_actual_error_rad: float
    done: bool


class StartupPoseMover:
    """Generate a bounded joint trajectory and verify arrival from feedback."""

    def __init__(
        self,
        target_rad: np.ndarray,
        *,
        lower_limit_rad: np.ndarray,
        upper_limit_rad: np.ndarray,
        max_speed_rad_s: float,
        max_acceleration_rad_s2: float,
        tolerance_rad: float,
        max_command_feedback_error_rad: float | None = None,
        feedback_limit_tolerance_rad: float = np.deg2rad(1.0),
        settle_samples: int = 3,
    ) -> None:
        self.target_rad = np.asarray(target_rad, dtype=np.float64)
        self.lower_limit_rad = np.asarray(lower_limit_rad, dtype=np.float64)
        self.upper_limit_rad = np.asarray(upper_limit_rad, dtype=np.float64)
        self.max_speed_rad_s = float(max_speed_rad_s)
        self.max_acceleration_rad_s2 = float(max_acceleration_rad_s2)
        self.tolerance_rad = float(tolerance_rad)
        self.max_command_feedback_error_rad = (
            None
            if max_command_feedback_error_rad is None
            else float(max_command_feedback_error_rad)
        )
        self.feedback_limit_tolerance_rad = float(feedback_limit_tolerance_rad)
        self.settle_samples = int(settle_samples)
        if self.target_rad.shape != (6,):
            raise ValueError("startup target must contain six joint angles")
        if self.settle_samples <= 0:
            raise ValueError("settle_samples must be positive")
        if min(self.max_speed_rad_s, self.max_acceleration_rad_s2, self.tolerance_rad) <= 0.0:
            raise ValueError("startup motion limits and tolerance must be positive")
        if self.feedback_limit_tolerance_rad < 0.0:
            raise ValueError("feedback limit tolerance must be non-negative")
        if (
            self.max_command_feedback_error_rad is not None
            and self.max_command_feedback_error_rad <= 0.0
        ):
            raise ValueError("max command feedback error must be positive")

        self._command_rad: np.ndarray | None = None
        self._velocity_rad_s = np.zeros(6, dtype=np.float64)
        self._settled_samples = 0

    def update(self, actual_rad: np.ndarray, dt_s: float) -> StartupPoseStatus:
        actual_rad = np.asarray(actual_rad, dtype=np.float64)
        if actual_rad.shape != (6,) or not np.all(np.isfinite(actual_rad)):
            raise RuntimeError("robot returned invalid feedback during initial-pose motion")
        outside = np.flatnonzero(
            (actual_rad < self.lower_limit_rad - self.feedback_limit_tolerance_rad)
            | (actual_rad > self.upper_limit_rad + self.feedback_limit_tolerance_rad)
        )
        if outside.size:
            details = ", ".join(
                f"joint{index + 1}={np.rad2deg(actual_rad[index]):.2f}deg "
                f"(allowed feedback "
                f"{np.rad2deg(self.lower_limit_rad[index] - self.feedback_limit_tolerance_rad):.1f}.."
                f"{np.rad2deg(self.upper_limit_rad[index] + self.feedback_limit_tolerance_rad):.1f}deg)"
                for index in outside
            )
            raise RuntimeError(
                "robot feedback is outside B601-DM limits; "
                f"check calibration and zero pose: {details}"
            )
        if self._command_rad is None:
            # Feedback may be just outside an exact URDF boundary because of
            # motor zero noise; commands always remain inside the hard limits.
            self._command_rad = np.clip(
                actual_rad, self.lower_limit_rad, self.upper_limit_rad
            )

        previous_command_rad = self._command_rad.copy()
        step_dt_s = float(np.clip(dt_s, 1e-4, 0.1))
        self._command_rad, self._velocity_rad_s = shape_joint_position_command(
            previous_position=self._command_rad,
            previous_velocity=self._velocity_rad_s,
            target_position=self.target_rad,
            dt_s=step_dt_s,
            max_speed=np.full(6, self.max_speed_rad_s),
            max_acceleration=np.full(6, self.max_acceleration_rad_s2),
            lower_limit=self.lower_limit_rad,
            upper_limit=self.upper_limit_rad,
        )
        if self.max_command_feedback_error_rad is not None:
            self._command_rad = bound_position_command_to_feedback(
                self._command_rad,
                actual_rad,
                self.max_command_feedback_error_rad,
                lower_limit=self.lower_limit_rad,
                upper_limit=self.upper_limit_rad,
            )
            self._velocity_rad_s = (
                self._command_rad - previous_command_rad
            ) / step_dt_s
        max_error = float(np.max(np.abs(self.target_rad - actual_rad)))
        max_command_error = float(np.max(np.abs(self.target_rad - self._command_rad)))
        at_target = (
            max_error <= self.tolerance_rad
            and max_command_error <= self.tolerance_rad
        )
        self._settled_samples = (
            self._settled_samples + 1 if at_target else 0
        )
        return StartupPoseStatus(
            command_rad=self._command_rad.copy(),
            max_actual_error_rad=max_error,
            done=self._settled_samples >= self.settle_samples,
        )


__all__ = [
    "DEFAULT_INITIAL_Q_REFERENCE_RAD",
    "REFERENCE_TO_DM_SIGN",
    "StartupPoseMover",
    "StartupPoseStatus",
    "reference_initial_q_to_dm",
]

"""Closed-form B601-DM wrist orientation solver."""

from __future__ import annotations

import numpy as np


def _finite_vector(value: object, sizes: tuple[int, ...], name: str) -> np.ndarray:
    array = np.asarray(value, dtype=np.float64)
    if array.ndim != 1 or array.size not in sizes or not np.all(np.isfinite(array)):
        expected = " or ".join(str(size) for size in sizes)
        raise ValueError(f"{name} must be a finite vector of size {expected}")
    return array.copy()


def _rotation_matrix(value: object, name: str) -> np.ndarray:
    matrix = np.asarray(value, dtype=np.float64)
    if (
        matrix.shape != (3, 3)
        or not np.all(np.isfinite(matrix))
        or not np.allclose(matrix.T @ matrix, np.eye(3), atol=1e-6)
        or not np.isclose(np.linalg.det(matrix), 1.0, atol=1e-6)
    ):
        raise ValueError(f"{name} must be a right-handed rotation matrix")
    return matrix.copy()


class ClosedFormWristSolver:
    """Solve absolute q4-q6 targets from the complete B601 wrist geometry."""

    def __init__(
        self,
        kinematics,
        *,
        joint_lower: np.ndarray,
        joint_upper: np.ndarray,
    ) -> None:
        self.kinematics = kinematics
        lower = _finite_vector(joint_lower, (3, 6), "joint_lower")[-3:]
        upper = _finite_vector(joint_upper, (3, 6), "joint_upper")[-3:]
        if np.any(lower > upper):
            raise ValueError("joint_lower must not exceed joint_upper")
        self.joint_lower = lower
        self.joint_upper = upper

    def solve(
        self,
        q123_rad: np.ndarray,
        target_rotation: np.ndarray,
        *,
        previous_q4_rad: float = 0.0,
    ) -> tuple[np.ndarray, float]:
        """Return a clipped wrist target and the maximum unclipped violation."""
        q123 = _finite_vector(q123_rad, (3,), "q123_rad")
        target = _rotation_matrix(target_rotation, "target_rotation")
        exact, _singular = self.kinematics.solve_wrist_orientation(
            q123,
            target,
            previous_q4_rad=previous_q4_rad,
        )
        exact = _finite_vector(exact, (3,), "wrist solution")
        violation = np.maximum(self.joint_lower - exact, 0.0)
        violation = np.maximum(violation, exact - self.joint_upper)
        return np.clip(exact, self.joint_lower, self.joint_upper), float(np.max(violation))


__all__ = ["ClosedFormWristSolver"]

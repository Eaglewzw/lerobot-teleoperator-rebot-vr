"""Separated B601 IK: q1-q3 position QP plus closed-form wrist orientation."""

from __future__ import annotations

import itertools
import time
import warnings

import numpy as np
from scipy.spatial.transform import Rotation

from ..control.joint_command import braking_velocity_bounds
from .kinematics import FullBodyQPIKSolver, QPSolveResult


class _WristAnchorKinematics:
    """Expose the joint4 axis as a position-only QP task frame."""

    def __init__(self, kinematics) -> None:
        self.kinematics = kinematics

    @property
    def lower_position_limit(self) -> np.ndarray:
        return self.kinematics.lower_position_limit

    @property
    def upper_position_limit(self) -> np.ndarray:
        return self.kinematics.upper_position_limit

    def tcp_pose_error(
        self, q_rad: object, target_position: object, target_rotation: object
    ) -> np.ndarray:
        del target_rotation
        position, _ = self.kinematics.wrist_anchor_pose(q_rad)
        target = np.asarray(target_position, dtype=np.float64)
        if target.shape != (3,) or not np.all(np.isfinite(target)):
            raise ValueError("target_position must be a finite three-vector")
        return np.concatenate((target - position, np.zeros(3, dtype=np.float64)))

    def tcp_jacobian(self, q_rad: object) -> np.ndarray:
        return self.kinematics.wrist_anchor_jacobian(q_rad)


class SplitIKSolver:
    """Solve joint4-axis position and wrist-relative orientation independently.

    The wrist Euler sequence and signs are inferred from the packaged URDF via
    FK at construction time.  No B601-specific axis swap or fixed wrist angle
    is encoded here.
    """

    def __init__(
        self,
        kinematics,
        *,
        solver: str = "scipy",
        position_cost: float = 20.0,
        orientation_cost: float = 2.0,
        orientation_cost_min: float = 0.05,
        position_gain: float = 10.0,
        orientation_gain: float = 8.0,
        damping_min: float = 1e-3,
        damping_max: float = 0.1,
        smoothness_cost: float = 0.05,
        posture_cost: float = 0.01,
        joint_limit_margin_rad: float = 0.03,
        max_solve_time_ms: float = 8.0,
        singularity_threshold: float = 0.08,
        singularity_critical_threshold: float = 0.02,
        singularity_characteristic_length_m: float = 0.3,
        joint_lower_limit_rad: object | None = None,
        joint_upper_limit_rad: object | None = None,
    ) -> None:
        self.kinematics = kinematics
        self.orientation_gain = float(orientation_gain)
        self.joint_limit_margin_rad = float(joint_limit_margin_rad)
        self.max_solve_time_ms = float(max_solve_time_ms)
        self._position_solver = FullBodyQPIKSolver(
            _WristAnchorKinematics(kinematics),
            solver=solver,
            ik_mode="position",
            position_cost=position_cost,
            orientation_cost=orientation_cost,
            orientation_cost_min=orientation_cost_min,
            position_gain=position_gain,
            orientation_gain=orientation_gain,
            damping_min=damping_min,
            damping_max=damping_max,
            smoothness_cost=smoothness_cost,
            posture_cost=posture_cost,
            joint_limit_margin_rad=joint_limit_margin_rad,
            max_solve_time_ms=max_solve_time_ms,
            singularity_threshold=singularity_threshold,
            singularity_critical_threshold=singularity_critical_threshold,
            singularity_characteristic_length_m=(
                singularity_characteristic_length_m
            ),
            joint_lower_limit_rad=joint_lower_limit_rad,
            joint_upper_limit_rad=joint_upper_limit_rad,
        )
        self.lower_position_limit = self._position_solver.lower_position_limit.copy()
        self.upper_position_limit = self._position_solver.upper_position_limit.copy()
        self._derive_wrist_decomposition()

    def solve(
        self,
        *,
        target_position: object,
        target_rotation: object,
        q_actual: object,
        dq_previous: object,
        dt: float,
        q_nominal: object,
        max_joint_speed: object,
        max_joint_acceleration: object,
        target_linear_velocity_m_s: object | None = None,
        target_angular_velocity_rad_s: object | None = None,
        q_seed: object | None = None,
    ) -> QPSolveResult:
        started = time.monotonic_ns()
        q = np.asarray(q_actual, dtype=np.float64)
        dq_prev = np.asarray(dq_previous, dtype=np.float64)
        q_reference = np.asarray(q_nominal, dtype=np.float64)
        seed = np.asarray(q if q_seed is None else q_seed, dtype=np.float64)
        speed = np.asarray(max_joint_speed, dtype=np.float64)
        acceleration = np.asarray(max_joint_acceleration, dtype=np.float64)
        vectors = (q, dq_prev, q_reference, seed, speed, acceleration)
        if any(value.shape != (6,) for value in vectors) or not all(
            np.all(np.isfinite(value)) for value in vectors
        ):
            return self._failure(started, "invalid_input")
        dt = float(dt)
        if not np.isfinite(dt) or dt <= 0.0:
            return self._failure(started, "invalid_dt")

        position_result = self._position_solver.solve(
            target_position=target_position,
            target_rotation=target_rotation,
            q_actual=q,
            dq_previous=dq_prev,
            dt=dt,
            q_nominal=q_reference,
            max_joint_speed=speed,
            max_joint_acceleration=acceleration,
            target_linear_velocity_m_s=target_linear_velocity_m_s,
            target_angular_velocity_rad_s=target_angular_velocity_rad_s,
            q_seed=seed,
        )
        if not position_result.success:
            return self._copy_failure(started, position_result)

        try:
            wrist_target, wrist_clip_rad, relative_target = self._wrist_target(
                target_rotation, q_reference, seed[3:]
            )
            lo, hi = self._velocity_bounds(q, dq_prev, dt, speed, acceleration)
        except (ValueError, np.linalg.LinAlgError) as exc:
            return self._failure(started, f"split_exception:{type(exc).__name__}")
        if np.any(lo > hi + 1e-10):
            return self._failure(started, "infeasible_constraints")

        wrist_velocity = self.orientation_gain * (wrist_target - q[3:])
        wrist_velocity = np.clip(wrist_velocity, lo[3:], hi[3:])
        joint_velocity = np.concatenate(
            (position_result.joint_velocity_rad_s[:3], wrist_velocity)
        )
        q_next = q + joint_velocity * dt

        try:
            position_next, _ = self.kinematics.wrist_anchor_pose(q_next)
            relative_next = self.kinematics.wrist_relative_rotation(q_next)
            position_error = float(
                np.linalg.norm(
                    np.asarray(target_position, dtype=np.float64) - position_next
                )
            )
            orientation_error = float(
                Rotation.from_matrix(relative_target @ relative_next.T).magnitude()
            )
        except (ValueError, np.linalg.LinAlgError) as exc:
            return self._failure(started, f"split_exception:{type(exc).__name__}")

        return QPSolveResult(
            q_target_rad=q_next,
            joint_velocity_rad_s=joint_velocity,
            success=True,
            position_error_m=position_error,
            orientation_error_rad=orientation_error,
            solve_time_ms=(time.monotonic_ns() - started) * 1e-6,
            reason="wrist_clipped" if wrist_clip_rad > 1e-10 else "",
            sigma_min=position_result.sigma_min,
            condition_number=position_result.condition_number,
            damping=position_result.damping,
            orientation_weight=position_result.orientation_weight,
            wrist_clip_rad=wrist_clip_rad,
        )

    def _derive_wrist_decomposition(self) -> None:
        zero = np.zeros(6, dtype=np.float64)
        self._wrist_zero_rotation = self.kinematics.wrist_relative_rotation(zero)
        axis_letters: list[str] = []
        axis_signs: list[float] = []
        epsilon = 1e-5
        for joint_index in range(3, 6):
            moved = zero.copy()
            moved[joint_index] = epsilon
            delta = (
                self.kinematics.wrist_relative_rotation(moved)
                @ self._wrist_zero_rotation.T
            )
            axis = Rotation.from_matrix(delta).as_rotvec() / epsilon
            axis /= np.linalg.norm(axis)
            component = int(np.argmax(np.abs(axis)))
            if abs(axis[component]) < 0.999:
                raise ValueError("split IK requires orthogonal wrist axes")
            axis_letters.append("XYZ"[component])
            axis_signs.append(float(np.sign(axis[component])))
        if len(set(axis_letters)) != 3:
            raise ValueError("split IK requires three distinct wrist axes")
        self._wrist_sequence = "".join(axis_letters)
        self._wrist_axis_indices = np.array(
            ["XYZ".index(letter) for letter in axis_letters], dtype=int
        )
        self._wrist_axis_signs = np.asarray(axis_signs, dtype=np.float64)

    def _wrist_target(
        self,
        target_rotation: object,
        q_reference: np.ndarray,
        wrist_seed: np.ndarray,
    ) -> tuple[np.ndarray, float, np.ndarray]:
        target = np.asarray(target_rotation, dtype=np.float64)
        if (
            target.shape != (3, 3)
            or not np.all(np.isfinite(target))
            or not np.allclose(target.T @ target, np.eye(3), atol=1e-6)
            or not np.isclose(np.linalg.det(target), 1.0, atol=1e-6)
        ):
            raise ValueError("target_rotation must be a right-handed rotation matrix")
        anchor_reference_q = q_reference.copy()
        anchor_reference_q[3:] = 0.0
        _, anchor_reference_rotation = self.kinematics.wrist_anchor_pose(
            anchor_reference_q
        )
        relative_target = anchor_reference_rotation.T @ target
        variable_rotation = relative_target @ self._wrist_zero_rotation.T

        with warnings.catch_warnings():
            warnings.simplefilter("ignore", UserWarning)
            angles = Rotation.from_matrix(variable_rotation).as_euler(
                self._wrist_sequence
            )
        candidates = self._equivalent_wrist_candidates(
            variable_rotation, angles, wrist_seed
        )
        lower = self.lower_position_limit[3:] + self.joint_limit_margin_rad
        upper = self.upper_position_limit[3:] - self.joint_limit_margin_rad
        if np.any(lower >= upper):
            raise ValueError("joint limit margin leaves no wrist range")

        def candidate_score(candidate: np.ndarray) -> tuple[float, float]:
            clipped = np.clip(candidate, lower, upper)
            return (
                float(np.linalg.norm(candidate - clipped)),
                float(np.linalg.norm(candidate - wrist_seed)),
            )

        raw_target = min(candidates, key=candidate_score)
        clipped_target = np.clip(raw_target, lower, upper)
        clip_rad = float(np.max(np.abs(raw_target - clipped_target)))
        return clipped_target, clip_rad, relative_target

    def _equivalent_wrist_candidates(
        self,
        variable_rotation: np.ndarray,
        angles: np.ndarray,
        wrist_seed: np.ndarray,
    ) -> list[np.ndarray]:
        bases: list[np.ndarray]
        if abs(np.cos(angles[1])) < 1e-5:
            first_angle = self._wrist_axis_signs[0] * wrist_seed[0]
            first_rotation = Rotation.from_rotvec(
                first_angle * np.eye(3)[self._wrist_axis_indices[0]]
            ).as_matrix()
            middle_rotation = Rotation.from_rotvec(
                angles[1] * np.eye(3)[self._wrist_axis_indices[1]]
            ).as_matrix()
            remaining = middle_rotation.T @ first_rotation.T @ variable_rotation
            last_axis = np.eye(3)[self._wrist_axis_indices[2]]
            last_angle = self._angle_about_axis(remaining, last_axis)
            singular_angles = np.array(
                [first_angle, angles[1], last_angle], dtype=np.float64
            )
            bases = [singular_angles / self._wrist_axis_signs]
        else:
            alternate = np.array(
                [angles[0] + np.pi, np.pi - angles[1], angles[2] + np.pi],
                dtype=np.float64,
            )
            bases = [
                angles / self._wrist_axis_signs,
                alternate / self._wrist_axis_signs,
            ]

        candidates: list[np.ndarray] = []
        for base in bases:
            centered = base + 2.0 * np.pi * np.round(
                (wrist_seed - base) / (2.0 * np.pi)
            )
            for turns in itertools.product((-1, 0, 1), repeat=3):
                candidates.append(centered + 2.0 * np.pi * np.asarray(turns))
        return candidates

    @staticmethod
    def _angle_about_axis(rotation: np.ndarray, axis: np.ndarray) -> float:
        skew_vector = 0.5 * np.array(
            [
                rotation[2, 1] - rotation[1, 2],
                rotation[0, 2] - rotation[2, 0],
                rotation[1, 0] - rotation[0, 1],
            ],
            dtype=np.float64,
        )
        sine = float(axis @ skew_vector)
        cosine = float((np.trace(rotation) - 1.0) * 0.5)
        return float(np.arctan2(sine, cosine))

    def _velocity_bounds(
        self,
        q: np.ndarray,
        dq_previous: np.ndarray,
        dt: float,
        speed: np.ndarray,
        acceleration: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray]:
        if np.any(speed <= 0.0) or np.any(acceleration <= 0.0):
            raise ValueError("motion limits must be positive")
        lower = self.lower_position_limit
        upper = self.upper_position_limit
        if np.any(q < lower) or np.any(q > upper):
            raise ValueError("feedback outside limits")
        safe_lower = np.where(
            q <= lower + self.joint_limit_margin_rad,
            q,
            lower + self.joint_limit_margin_rad,
        )
        safe_upper = np.where(
            q >= upper - self.joint_limit_margin_rad,
            q,
            upper - self.joint_limit_margin_rad,
        )
        constrained_previous = np.clip(dq_previous, -speed, speed)
        braking_lower, braking_upper = braking_velocity_bounds(
            q, safe_lower, safe_upper, acceleration, reaction_time_s=dt
        )
        lower_velocity = np.maximum.reduce(
            (
                -speed,
                (safe_lower - q) / dt,
                constrained_previous - acceleration * dt,
                braking_lower,
            )
        )
        upper_velocity = np.minimum.reduce(
            (
                speed,
                (safe_upper - q) / dt,
                constrained_previous + acceleration * dt,
                braking_upper,
            )
        )
        return lower_velocity, upper_velocity

    @staticmethod
    def _copy_failure(
        started_ns: int, result: QPSolveResult
    ) -> QPSolveResult:
        return QPSolveResult(
            q_target_rad=np.zeros(6, dtype=np.float64),
            joint_velocity_rad_s=np.zeros(6, dtype=np.float64),
            success=False,
            position_error_m=result.position_error_m,
            orientation_error_rad=result.orientation_error_rad,
            solve_time_ms=(time.monotonic_ns() - started_ns) * 1e-6,
            reason=result.reason,
            sigma_min=result.sigma_min,
            condition_number=result.condition_number,
            damping=result.damping,
            orientation_weight=result.orientation_weight,
        )

    @staticmethod
    def _failure(started_ns: int, reason: str) -> QPSolveResult:
        return QPSolveResult(
            q_target_rad=np.zeros(6, dtype=np.float64),
            joint_velocity_rad_s=np.zeros(6, dtype=np.float64),
            success=False,
            position_error_m=float("inf"),
            orientation_error_rad=float("inf"),
            solve_time_ms=(time.monotonic_ns() - started_ns) * 1e-6,
            reason=reason,
            sigma_min=float("nan"),
            condition_number=float("nan"),
            damping=float("nan"),
            orientation_weight=float("nan"),
        )


__all__ = ["SplitIKSolver"]

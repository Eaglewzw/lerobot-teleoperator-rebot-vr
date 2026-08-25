from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation

from ..control.types import CartesianControlConfig, IKWorker
from ..vr.adapter import VR_SAMPLE_TYPES, sample_key
from ..vr.pose_mapping import PoseTarget, TeleopState
from .async_worker import IKRequest, IKResult


class QPRequestCoordinator:
    """Coordinate latest-only QP requests and generation-safe result handling."""

    def __init__(
        self,
        worker: IKWorker,
        config: CartesianControlConfig,
        lower_limit_rad: np.ndarray,
        upper_limit_rad: np.ndarray,
    ) -> None:
        self.worker = worker
        self.config = config
        self.lower_limit_rad = lower_limit_rad
        self.upper_limit_rad = upper_limit_rad
        self.generation = 0
        self.sequence = 0
        self.last_result: IKResult | None = None
        self.last_result_age_ms: float | None = None
        self.target_linear_velocity_m_s = np.zeros(3, dtype=np.float64)
        self.target_angular_velocity_rad_s = np.zeros(3, dtype=np.float64)

        self._last_submitted_sample: tuple[int, int, int] | None = None
        self._last_submitted_sequence = 0
        self._last_submitted_sample_id: int | None = None
        self._last_submission_ns: int | None = None
        self._last_consumed_sequence = 0
        self._last_request_actual_rad: np.ndarray | None = None
        self._last_request_dt_s: float | None = None
        self._dq_rad_s = np.zeros(6, dtype=np.float64)
        self._last_velocity_target: PoseTarget | None = None
        self._last_velocity_sample_key: tuple[int, int, int] | None = None

    def begin_generation(self) -> None:
        self.generation += 1
        self.worker.clear()
        self._last_submitted_sample = None
        self._last_submitted_sequence = 0
        self._last_submitted_sample_id = None
        self._last_submission_ns = None
        self.last_result = None
        self._last_request_actual_rad = None
        self._last_request_dt_s = None
        self._dq_rad_s.fill(0.0)
        self._last_velocity_target = None
        self._last_velocity_sample_key = None
        self.target_linear_velocity_m_s.fill(0.0)
        self.target_angular_velocity_rad_s.fill(0.0)
        self.last_result_age_ms = None

    def reset_velocity(self) -> None:
        self._dq_rad_s.fill(0.0)

    def capture_velocity_reference(
        self, target: PoseTarget, frame: VR_SAMPLE_TYPES | None
    ) -> None:
        self._last_velocity_target = target
        self._last_velocity_sample_key = sample_key(frame)
        self.target_linear_velocity_m_s.fill(0.0)
        self.target_angular_velocity_rad_s.fill(0.0)

    def submit_if_ready(
        self,
        *,
        target: PoseTarget,
        frame: VR_SAMPLE_TYPES | None,
        q_seed_rad: np.ndarray,
        q_actual_rad: np.ndarray,
        q_nominal_rad: np.ndarray,
        dt_s: float,
        now_ns: int,
    ) -> bool:
        current_sample_key = sample_key(frame)
        request_in_flight = (
            self._last_submitted_sequence > self._last_consumed_sequence
        )
        if (
            current_sample_key is None
            or current_sample_key == self._last_submitted_sample
            or request_in_flight
        ):
            return False

        qp_dt_s = dt_s
        if self._last_submission_ns is not None:
            qp_dt_s = float(
                np.clip((now_ns - self._last_submission_ns) * 1e-9, 1e-6, 0.05)
            )
        self.sequence += 1
        linear_velocity, angular_velocity = self._target_velocity(target, frame)
        self._last_request_actual_rad = q_actual_rad.copy()
        self._last_request_dt_s = qp_dt_s
        self.worker.submit(
            IKRequest(
                generation=self.generation,
                sequence=self.sequence,
                sample_id=target.sample_id,
                target_position=target.position,
                target_rotation=target.rotation,
                target_linear_velocity_m_s=linear_velocity,
                target_angular_velocity_rad_s=angular_velocity,
                q_seed=q_seed_rad,
                q_actual=q_actual_rad,
                dq_previous=self._dq_rad_s.copy(),
                dt=qp_dt_s,
                q_nominal=q_nominal_rad,
                submitted_monotonic_ns=now_ns,
            )
        )
        self._last_submitted_sample = current_sample_key
        self._last_submitted_sequence = self.sequence
        self._last_submitted_sample_id = target.sample_id
        self._last_submission_ns = now_ns
        return True

    def consume_latest(
        self,
        *,
        state: TeleopState,
        q_actual_rad: np.ndarray,
        now_ns: int,
    ) -> np.ndarray | None:
        result = self.worker.latest_result()
        if result is None or result.sequence <= self._last_consumed_sequence:
            return None
        self._last_consumed_sequence = result.sequence
        if (
            result.generation != self.generation
            or state is not TeleopState.ACTIVE
            or result.sequence != self._last_submitted_sequence
            or result.sample_id != self._last_submitted_sample_id
            or result.solve_time_ms > self.config.qp_max_solve_time_ms
        ):
            return None

        self.last_result = result
        solver_candidate = np.asarray(result.q_target_rad, dtype=np.float64)
        if solver_candidate.shape != (6,) or not np.all(np.isfinite(solver_candidate)):
            return None
        if not result.success:
            return None
        if result.joint_velocity_rad_s is not None:
            qp_velocity = np.asarray(result.joint_velocity_rad_s, dtype=np.float64)
            if qp_velocity.shape != (6,) or not np.all(np.isfinite(qp_velocity)):
                return None
            self._dq_rad_s = qp_velocity.copy()
        elif self._last_request_actual_rad is not None and self._last_request_dt_s is not None:
            self._dq_rad_s = (
                solver_candidate - self._last_request_actual_rad
            ) / self._last_request_dt_s

        lookahead_s = np.concatenate(
            (
                np.full(3, self.config.arm_command_lookahead_s),
                np.full(3, self.config.wrist_command_lookahead_s),
            )
        )
        candidate = q_actual_rad + self._dq_rad_s * lookahead_s
        margin = np.deg2rad(self.config.joint_limit_margin_deg)
        safe_lower = np.where(
            q_actual_rad <= self.lower_limit_rad + margin,
            q_actual_rad,
            self.lower_limit_rad + margin,
        )
        safe_upper = np.where(
            q_actual_rad >= self.upper_limit_rad - margin,
            q_actual_rad,
            self.upper_limit_rad - margin,
        )
        self.last_result_age_ms = (
            None
            if result.submitted_monotonic_ns <= 0
            else max(0.0, (now_ns - result.submitted_monotonic_ns) * 1e-6)
        )
        return np.clip(candidate, safe_lower, safe_upper)

    def _target_velocity(
        self, target: PoseTarget, frame: VR_SAMPLE_TYPES | None
    ) -> tuple[np.ndarray, np.ndarray]:
        current_sample_key = sample_key(frame)
        previous = self._last_velocity_target
        previous_key = self._last_velocity_sample_key
        linear = np.zeros(3, dtype=np.float64)
        angular = np.zeros(3, dtype=np.float64)
        if (
            current_sample_key is not None
            and previous_key is not None
            and previous is not None
            and current_sample_key[0] == previous_key[0]
        ):
            elapsed_s = (current_sample_key[2] - previous_key[2]) * 1e-9
            if 1e-6 <= elapsed_s <= self.config.stale_timeout_s:
                linear = (target.position - previous.position) / elapsed_s
                angular = (
                    Rotation.from_matrix(target.rotation @ previous.rotation.T).as_rotvec()
                    / elapsed_s
                )
        self._last_velocity_target = target
        self._last_velocity_sample_key = current_sample_key
        self.target_linear_velocity_m_s = linear
        self.target_angular_velocity_rad_s = angular
        return linear.copy(), angular.copy()

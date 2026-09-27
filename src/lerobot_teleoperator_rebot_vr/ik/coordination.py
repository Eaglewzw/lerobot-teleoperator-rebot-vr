from __future__ import annotations

from dataclasses import replace
import numpy as np
from scipy.spatial.transform import Rotation

from ..control.types import CartesianControlConfig, IKWorker
from ..vr.adapter import VR_SAMPLE_TYPES, sample_key
from ..vr.pose_mapping import PoseTarget, TeleopState
from .async_worker import IKRequest, IKResult
from ..diagnostics.velocity import vector_fields
from ..vr.linear_velocity import LinearVelocityEstimator
from ..control.reference_motion import CartesianReferenceMotion


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
        self.linear_velocity_estimator = LinearVelocityEstimator(config)
        self.reference_motion = (
            CartesianReferenceMotion(config.split_reference_speed_m_s,
                                     config.split_reference_acceleration_m_s2)
            if config.ik_mode == "split" and config.split_reference_speed_m_s > 0 else None
        )
        self._sent_velocity = None
        self._sent_velocity_ns = None
        self._pending_motor_request: IKRequest | None = None
        self._last_dispatch_ns: int | None = None
        self.lower_limit_rad = lower_limit_rad
        self.upper_limit_rad = upper_limit_rad
        self.generation = 0
        self.sequence = 0
        self.request_diagnostics = {}
        self.result_diagnostics = {}
        self.last_result: IKResult | None = None
        self.last_result_age_ms: float | None = None
        self.last_result_consumed_monotonic_ns: int | None = None
        self.last_result_accepted_monotonic_ns: int | None = None
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
        self._pending_motor_request = None
        self._last_dispatch_ns = None
        self._sent_velocity = None
        self._sent_velocity_ns = None
        if self.reference_motion is not None:
            self.reference_motion.reset()
        self.linear_velocity_estimator.reset()
        self.request_diagnostics = {}
        self.result_diagnostics = {}
        self.generation += 1
        self.worker.clear()
        self._last_submitted_sample = None
        self._last_submitted_sequence = 0
        self._last_submitted_sample_id = None
        self._last_submission_ns = None
        self.last_result = None
        self.last_result_consumed_monotonic_ns = None
        self.last_result_accepted_monotonic_ns = None
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

    def observe_sent_velocity(self, velocity, sent_monotonic_ns: int) -> None:
        """Snapshot only a successfully sent motor command, not a QP intention."""
        v = np.asarray(velocity, dtype=float)
        if v.shape != (6,) or not np.isfinite(v).all() or sent_monotonic_ns < 0:
            raise ValueError("invalid sent velocity snapshot")
        self._sent_velocity = v.copy()
        self._sent_velocity_ns = int(sent_monotonic_ns)
        request = self._pending_motor_request
        if request is None:
            return
        age_s = (sent_monotonic_ns - request.submitted_monotonic_ns) * 1e-9
        if request.generation != self.generation or not 0 <= age_s <= .05:
            self.cancel_pending_motor_request()
            return
        dt = request.dt if self._last_dispatch_ns is None else float(np.clip(
            (sent_monotonic_ns - self._last_dispatch_ns)*1e-9, 1e-6, .05))
        request = replace(request, dq_previous=v, dt=dt,
                          submitted_monotonic_ns=sent_monotonic_ns)
        self._pending_motor_request = None
        self._last_dispatch_ns = sent_monotonic_ns
        self._last_submission_ns = sent_monotonic_ns
        self._last_request_dt_s = dt
        self._diagnostic_previous_velocity = v.copy()
        self.request_diagnostics.update({
            "ik_request_submitted_monotonic_ns": sent_monotonic_ns,
            "ik_request_dt_s": dt,
            "ik_request_sent_velocity_age_ms": 0.,
            "ik_request_dispatch_after_send": True,
            **vector_fields("ik_request_previous_velocity", v, "rad_s"),
        })
        self.worker.submit(request)

    def cancel_pending_motor_request(self) -> None:
        """Do not dispatch a prepared request after HOLD, stop, or a slow send."""
        if self._pending_motor_request is not None:
            self._last_consumed_sequence = max(
                self._last_consumed_sequence, self._pending_motor_request.sequence)
            self._pending_motor_request = None
            self._last_submitted_sample = None

    def capture_velocity_reference(
        self, target: PoseTarget, frame: VR_SAMPLE_TYPES | None
    ) -> None:
        self.linear_velocity_estimator.capture(target.position, sample_key(frame))
        if self.reference_motion is not None:
            self.reference_motion.reset(target.position)
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
        actual_position_m: np.ndarray | None = None,
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

        sent_age_s = None
        if self._sent_velocity_ns is not None and not self.config.qp_use_sent_velocity:
            sent_age_s = (now_ns - self._sent_velocity_ns) * 1e-9
            if not 0 <= sent_age_s <= .05:
                # Wait for a new successful motor send. Never fabricate a zero
                # velocity while the motor layer may still be decelerating.
                return False

        qp_dt_s = dt_s
        if self._last_submission_ns is not None:
            qp_dt_s = float(
                np.clip((now_ns - self._last_submission_ns) * 1e-9, 1e-6, 0.05)
            )
        self.sequence += 1
        linear_velocity, angular_velocity = self._target_velocity(target, frame)
        request_position = target.position.copy()
        if self.reference_motion is not None:
            request_position, linear_velocity = self.reference_motion.update(
                request_position, float(np.clip(qp_dt_s, 1e-6, .05)))
        previous_velocity = self._dq_rad_s.copy()
        if self._sent_velocity_ns is not None:
            previous_velocity = self._sent_velocity.copy()
        self._last_request_actual_rad = q_actual_rad.copy()
        self._last_request_dt_s = qp_dt_s
        self.request_diagnostics = {
            **self.linear_velocity_estimator.diagnostics,
            "ik_request_sequence": self.sequence,
            "ik_request_generation": self.generation,
            "ik_request_sample_id": target.sample_id,
            "ik_request_submitted_monotonic_ns": now_ns,
            "ik_request_prepared_monotonic_ns": now_ns,
            "ik_request_dt_s": qp_dt_s,
            **vector_fields("ik_request_linear_velocity", linear_velocity, "m_s", "xyz"),
            **vector_fields("ik_request_previous_velocity", previous_velocity, "rad_s"),
            **vector_fields("ik_request_target_position", request_position, "m", "xyz"),
            "ik_request_sent_velocity_age_ms": None if sent_age_s is None else sent_age_s*1000,
            "ik_request_dispatch_after_send": False,
        }
        if actual_position_m is not None:
            self.request_diagnostics.update(vector_fields(
                "ik_request_position_error", request_position - actual_position_m, "m", "xyz"
            ))
        self._diagnostic_previous_velocity = previous_velocity.copy()
        request = IKRequest(
            generation=self.generation,
            sequence=self.sequence,
            sample_id=target.sample_id,
            target_position=request_position,
            target_rotation=target.rotation,
            target_linear_velocity_m_s=linear_velocity,
            target_angular_velocity_rad_s=angular_velocity,
            q_seed=q_seed_rad,
            q_actual=q_actual_rad,
            dq_previous=previous_velocity,
            dt=qp_dt_s,
            q_nominal=q_nominal_rad,
            submitted_monotonic_ns=now_ns,
            sample_received_monotonic_ns=(
                0 if frame is None else int(frame.received_monotonic_ns)
            ),
        )
        if self.config.qp_use_sent_velocity:
            # consume -> prepare -> motor send -> dispatch. Submitting before
            # the send uses the previous cycle's velocity and creates a two-
            # cycle acceleration recurrence with a one-cycle dt.
            self._pending_motor_request = request
        else:
            self.worker.submit(request)
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
        ):
            return None

        self.last_result = result
        self.result_diagnostics = {}
        if result.joint_velocity_rad_s is not None:
            velocity = np.asarray(result.joint_velocity_rad_s)
            self.result_diagnostics.update(vector_fields("qp_output_velocity", velocity, "rad_s"))
            if result.success:
                speed = np.array([self.config.max_joint_speed_rad_s] * 3 + [
                    self.config.wrist_speed_rad_s or self.config.max_joint_speed_rad_s] * 3)
                acceleration = np.array([self.config.max_joint_acceleration_rad_s2] * 3 + [
                    self.config.wrist_acceleration_rad_s2 or self.config.max_joint_acceleration_rad_s2] * 3)
                previous = np.clip(self._diagnostic_previous_velocity, -speed, speed)
                for kind, flags in (
                    ("speed", np.isclose(abs(velocity), speed, rtol=0, atol=1e-6)),
                    ("acceleration", np.isclose(abs(velocity - previous),
                        acceleration * self._last_request_dt_s, rtol=0, atol=1e-6)),
                ):
                    self.result_diagnostics.update(vector_fields(
                        f"qp_{kind}_bound_active", flags, "flag"))
        self.last_result_consumed_monotonic_ns = now_ns
        self.last_result_age_ms = (
            None
            if result.submitted_monotonic_ns <= 0
            else max(0.0, (now_ns - result.submitted_monotonic_ns) * 1e-6)
        )
        if result.solve_time_ms > self.config.qp_max_solve_time_ms:
            return self._reject_matching_result()
        solver_candidate = np.asarray(result.q_target_rad, dtype=np.float64)
        if solver_candidate.shape != (6,) or not np.all(np.isfinite(solver_candidate)):
            return self._reject_matching_result()
        if not result.success:
            return self._reject_matching_result()
        if result.joint_velocity_rad_s is not None:
            qp_velocity = np.asarray(result.joint_velocity_rad_s, dtype=np.float64)
            if qp_velocity.shape != (6,) or not np.all(np.isfinite(qp_velocity)):
                return self._reject_matching_result()
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
        self.last_result_accepted_monotonic_ns = now_ns
        return np.clip(candidate, safe_lower, safe_upper)

    def _reject_matching_result(self) -> None:
        # A failed matching result makes the previous QP velocity stale. The
        # motor layer performs the physical deceleration; the next QP starts
        # from zero so an obsolete outward velocity cannot latch constraints.
        self._dq_rad_s.fill(0.0)
        return None

    @property
    def accepted_joint_velocity_rad_s(self) -> np.ndarray:
        """Return the velocity from the most recently accepted QP result."""
        return self._dq_rad_s.copy()

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
                angular = (
                    Rotation.from_matrix(target.rotation @ previous.rotation.T).as_rotvec()
                    / elapsed_s
                )
        linear = self.linear_velocity_estimator.update(target.position, current_sample_key)
        self._last_velocity_target = target
        self._last_velocity_sample_key = current_sample_key
        self.target_linear_velocity_m_s = linear
        self.target_angular_velocity_rad_s = angular
        return linear.copy(), angular.copy()

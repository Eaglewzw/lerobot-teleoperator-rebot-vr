from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Protocol

import numpy as np
from scipy.spatial.transform import Rotation

from .async_ik import IKRequest, IKResult, LatestOnlyQPIKWorker
from .joint_command import (
    bound_position_command_to_feedback,
    shape_joint_position_command,
)
from .pose_mapping import RelativePoseMapper, TeleopState
from .processor import VRFrame
from .startup_pose import reference_initial_q_to_dm
from .tracking import ControllerSample
from .kinematics import FullBodyQPIKSolver


ARM_JOINT_NAMES = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_yaw",
    "wrist_roll",
)
GRIPPER_NAME = "gripper"

# Match the LeRobot RebotB601Follower software limits, expressed in radians.
FOLLOWER_LOWER_RAD = np.deg2rad(np.array([-150.0, -200.0, -200.0, -80.0, -90.0, -90.0]))
FOLLOWER_UPPER_RAD = np.deg2rad(np.array([150.0, 1.0, 1.0, 90.0, 90.0, 90.0]))
FEEDBACK_LIMIT_TOLERANCE_RAD = np.deg2rad(1.0)


class IKWorker(Protocol):
    submitted: int
    solved: int
    rejected: int

    def start(self) -> None: ...
    def stop(self) -> None: ...
    def clear(self) -> None: ...
    def submit(self, request: IKRequest) -> None: ...
    def latest_result(self) -> IKResult | None: ...


@dataclass(frozen=True)
class CartesianControlConfig:
    qp_solver: str = "scipy"
    qp_position_cost: float = 20.0
    qp_orientation_cost: float = 2.0
    qp_damping: float = 1e-3
    qp_smoothness_cost: float = 0.05
    qp_posture_cost: float = 0.01
    joint_limit_margin_deg: float = 2.0
    qp_max_solve_time_ms: float = 8.0
    position_scale: float = 1.0
    orientation_scale: float = 1.0
    position_filter_hz: float = 8.0
    orientation_filter_hz: float = 6.0
    position_deadband_m: float = 5e-4
    orientation_deadband_rad: float = np.deg2rad(0.25)
    grip_press_threshold: float = 0.85
    grip_release_threshold: float = 0.75
    stale_timeout_s: float = 0.2
    max_joint_speed_rad_s: float = 2.0
    max_joint_acceleration_rad_s2: float = 8.0
    wrist_speed_rad_s: float | None = None
    wrist_acceleration_rad_s2: float | None = None
    wrist_command_feedback_error_deg: float | None = None
    gripper_command_feedback_error_deg: float | None = None
    feedback_fault_max_consecutive: int = 5
    initial_q_rad: tuple[float, float, float, float, float, float] = (
        0.0,
        0.8,
        0.8,
        0.0,
        0.0,
        0.0,
    )
    gripper_open_deg: float = -270.0
    gripper_closed_deg: float = 0.0
    gripper_max_speed_deg_s: float = 90.0
    gripper_max_acceleration_deg_s2: float = 360.0
    max_command_feedback_error_deg: float | None = None

    def __post_init__(self) -> None:
        non_negative = np.asarray(
            (
                self.position_scale,
                self.orientation_scale,
                self.position_filter_hz,
                self.orientation_filter_hz,
                self.position_deadband_m,
                self.orientation_deadband_rad,
                self.qp_position_cost, self.qp_orientation_cost,
                self.qp_damping, self.qp_smoothness_cost, self.qp_posture_cost,
                self.joint_limit_margin_deg, self.qp_max_solve_time_ms,
            ),
            dtype=np.float64,
        )
        if not np.all(np.isfinite(non_negative)) or np.any(non_negative < 0.0):
            raise ValueError("mapping values and IK damping must be non-negative")
        positive = np.asarray(
            (
                self.stale_timeout_s,
                self.max_joint_speed_rad_s,
                self.max_joint_acceleration_rad_s2,
                self.gripper_max_speed_deg_s,
                self.gripper_max_acceleration_deg_s2,
                self.qp_max_solve_time_ms,
            ),
            dtype=np.float64,
        )
        if not np.all(np.isfinite(positive)) or np.any(positive <= 0.0):
            raise ValueError("control rates and motion limits must be finite and positive")
        wrist_positive = np.asarray(
            tuple(
                value
                for value in (
                    self.wrist_speed_rad_s,
                    self.wrist_acceleration_rad_s2,
                    self.wrist_command_feedback_error_deg,
                    self.gripper_command_feedback_error_deg,
                )
                if value is not None
            ),
            dtype=np.float64,
        )
        if not np.all(np.isfinite(wrist_positive)) or np.any(wrist_positive <= 0.0):
            raise ValueError("wrist and gripper control limits must be finite and positive")
        if self.feedback_fault_max_consecutive <= 0:
            raise ValueError("feedback_fault_max_consecutive must be positive")
        if not 0.0 <= self.grip_release_threshold < self.grip_press_threshold <= 1.0:
            raise ValueError("Grip thresholds must satisfy 0 <= release < press <= 1")
        initial_q = np.asarray(self.initial_q_rad, dtype=np.float64)
        if initial_q.shape != (6,) or not np.all(np.isfinite(initial_q)):
            raise ValueError("initial_q_rad must contain six finite values")
        if not np.all(
            np.isfinite((self.gripper_open_deg, self.gripper_closed_deg))
        ) or self.gripper_open_deg >= self.gripper_closed_deg:
            raise ValueError("gripper positions must be finite with open < closed")
        if (
            self.max_command_feedback_error_deg is not None
            and (
                not np.isfinite(self.max_command_feedback_error_deg)
                or self.max_command_feedback_error_deg <= 0.0
            )
        ):
            raise ValueError("max command feedback error must be positive")


@dataclass(frozen=True)
class CartesianControlStatus:
    state: TeleopState
    tracking: bool
    ik_success: bool | None
    ik_error_m: float | None
    ik_reason: str
    submitted: int
    solved: int
    rejected: int
    trigger: float
    primary_button: bool
    secondary_button: bool
    home_requested: bool
    zero_requested: bool
    generation: int
    gripper_actual_deg: float
    gripper_target_deg: float
    gripper_command_deg: float
    actual_deg: np.ndarray
    target_deg: np.ndarray
    command_deg: np.ndarray
    orientation_error_deg: float | None
    wrist_clip_deg: float
    feedback_valid: bool = True
    feedback_fault_count: int = 0
    feedback_fault_reason: str = ""
    feedback_abort_requested: bool = False


class FullBodyQPIKController:
    """Closed-loop B601 Cartesian controller used by the real-robot runner."""

    def __init__(
        self,
        kinematics,
        *,
        xr_to_base_rotation: np.ndarray,
        config: CartesianControlConfig | None = None,
        ik_worker: IKWorker | None = None,
    ) -> None:
        self.kinematics = kinematics
        self.config = config or CartesianControlConfig()
        self.mapper = RelativePoseMapper(
            xr_to_world=xr_to_base_rotation,
            position_scale=self.config.position_scale,
            orientation_scale=self.config.orientation_scale,
            position_filter_hz=self.config.position_filter_hz,
            orientation_filter_hz=self.config.orientation_filter_hz,
            position_deadband_m=self.config.position_deadband_m,
            orientation_deadband_rad=self.config.orientation_deadband_rad,
            grip_press_threshold=self.config.grip_press_threshold,
            grip_release_threshold=self.config.grip_release_threshold,
            stale_timeout_s=self.config.stale_timeout_s,
        )
        model_lower = np.asarray(kinematics.lower_position_limit, dtype=np.float64)
        model_upper = np.asarray(kinematics.upper_position_limit, dtype=np.float64)
        self.lower_limit_rad = np.maximum(model_lower, FOLLOWER_LOWER_RAD)
        self.upper_limit_rad = np.minimum(model_upper, FOLLOWER_UPPER_RAD)
        self.feedback_lower_limit_rad = np.maximum(
            self.lower_limit_rad - FEEDBACK_LIMIT_TOLERANCE_RAD,
            FOLLOWER_LOWER_RAD,
        )
        self.feedback_upper_limit_rad = np.minimum(
            self.upper_limit_rad + FEEDBACK_LIMIT_TOLERANCE_RAD,
            FOLLOWER_UPPER_RAD,
        )
        if ik_worker is not None:
            self.worker = ik_worker
        else:
            qp = FullBodyQPIKSolver(
                kinematics, solver=self.config.qp_solver,
                position_cost=self.config.qp_position_cost,
                orientation_cost=self.config.qp_orientation_cost,
                damping=self.config.qp_damping,
                smoothness_cost=self.config.qp_smoothness_cost,
                posture_cost=self.config.qp_posture_cost,
                joint_limit_margin_rad=np.deg2rad(self.config.joint_limit_margin_deg),
                max_solve_time_ms=self.config.qp_max_solve_time_ms,
            )
            speed = np.concatenate((np.full(3, self.config.max_joint_speed_rad_s), np.full(3, self.config.wrist_speed_rad_s or self.config.max_joint_speed_rad_s)))
            acceleration = np.concatenate((np.full(3, self.config.max_joint_acceleration_rad_s2), np.full(3, self.config.wrist_acceleration_rad_s2 or self.config.max_joint_acceleration_rad_s2)))
            self.worker = LatestOnlyQPIKWorker(qp, max_joint_speed_rad_s=speed, max_joint_acceleration_rad_s2=acceleration)

        self._generation = 0
        self._sequence = 0
        self._last_submitted_sample: tuple[int, int, int] | None = None
        self._last_submitted_sequence = 0
        self._last_submitted_sample_id: int | None = None
        self._last_qp_submission_ns: int | None = None
        self._last_consumed_sequence = 0
        self._last_state = TeleopState.WAITING
        self._q_goal_rad: np.ndarray | None = None
        self._qp_nominal_rad: np.ndarray | None = None
        self._q_command_rad: np.ndarray | None = None
        self._dq_command_rad_s: np.ndarray | None = None
        # QP smoothness uses the last successful QP velocity. Keep this
        # separate from the outer command-shaper velocity, which may be
        # clamped or reset by feedback binding.
        self._dq_qp_rad_s = np.zeros(6, dtype=np.float64)
        self._last_qp_request_actual_rad: np.ndarray | None = None
        self._last_qp_request_dt_s: float | None = None
        self._gripper_goal_deg = 0.0
        self._gripper_command_deg = 0.0
        self._gripper_velocity_deg_s = 0.0
        self._last_ik_result: IKResult | None = None
        self._last_wrist_violation_rad = 0.0
        self._primary_button_down = False
        self._secondary_button_down = False
        self._feedback_fault_count = 0
        self._feedback_fault_reason = ""
        self._last_valid_q_actual_rad: np.ndarray | None = None
        self._last_valid_gripper_actual_deg: float | None = None
        self.home_q_rad = reference_initial_q_to_dm(
            self.config.initial_q_rad,
            lower_limit_rad=self.lower_limit_rad,
            upper_limit_rad=self.upper_limit_rad,
        )
        self.zero_q_rad = np.zeros(6, dtype=np.float64)

    def start(self) -> None:
        self.worker.start()

    def stop(self) -> None:
        self.worker.stop()

    def update(
        self,
        frame: ControllerSample | VRFrame | None,
        observation: dict[str, float],
        dt_s: float,
        *,
        now_ns: int | None = None,
    ) -> tuple[dict[str, float] | None, CartesianControlStatus]:
        q_actual_rad, gripper_actual_deg, feedback_error = self._read_feedback(
            observation
        )
        if feedback_error:
            return self._hold_for_feedback_fault(
                q_actual_rad, gripper_actual_deg, feedback_error
            )
        outside = np.flatnonzero(
            (q_actual_rad < self.feedback_lower_limit_rad)
            | (q_actual_rad > self.feedback_upper_limit_rad)
        )
        if outside.size:
            details = ", ".join(
                f"{ARM_JOINT_NAMES[index]}={np.rad2deg(q_actual_rad[index]):.2f}deg "
                f"(allowed feedback "
                f"{np.rad2deg(self.feedback_lower_limit_rad[index]):.1f}.."
                f"{np.rad2deg(self.feedback_upper_limit_rad[index]):.1f}deg)"
                for index in outside
            )
            return self._hold_for_feedback_fault(
                q_actual_rad,
                gripper_actual_deg,
                "outside_limits:" + details,
            )
        recovering_feedback = self._feedback_fault_count > 0
        q_control_actual_rad = np.clip(
            q_actual_rad, self.lower_limit_rad, self.upper_limit_rad
        )
        if self._q_command_rad is None:
            self._q_goal_rad = q_control_actual_rad.copy()
            self._qp_nominal_rad = q_control_actual_rad.copy()
            self._q_command_rad = q_control_actual_rad.copy()
            self._dq_command_rad_s = np.zeros(6, dtype=np.float64)
            self._gripper_goal_deg = gripper_actual_deg
            self._gripper_command_deg = gripper_actual_deg

        if recovering_feedback:
            self.mapper.reset(require_release=True)
            self._begin_generation()
            self._q_goal_rad = q_control_actual_rad.copy()
            self._qp_nominal_rad = q_control_actual_rad.copy()
            self._q_command_rad = q_control_actual_rad.copy()
            self._dq_command_rad_s.fill(0.0)
            self._dq_qp_rad_s.fill(0.0)
            self._gripper_goal_deg = gripper_actual_deg
            self._gripper_command_deg = gripper_actual_deg
            self._gripper_velocity_deg_s = 0.0
            self._last_wrist_violation_rad = 0.0
            self._last_state = self.mapper.state
            self._feedback_fault_count = 0
            self._feedback_fault_reason = ""

        self._last_valid_q_actual_rad = q_actual_rad.copy()
        self._last_valid_gripper_actual_deg = gripper_actual_deg

        assert self._q_goal_rad is not None
        assert self._q_command_rad is not None
        assert self._dq_command_rad_s is not None
        # Normalize dt before it is captured in an asynchronous QP request.
        dt_s = float(np.clip(dt_s, 1e-6, 0.05))

        tcp_position, ee_rotation = self.kinematics.forward_kinematics(q_control_actual_rad)
        now_value_ns = time.monotonic_ns() if now_ns is None else int(now_ns)
        sample_fresh = self._sample_is_fresh(frame, now_value_ns)
        primary_button = bool(
            sample_fresh and frame is not None and frame.primary_button
        )
        secondary_button = bool(
            sample_fresh and frame is not None and frame.secondary_button
        )
        primary_pressed = primary_button and not self._primary_button_down
        secondary_pressed = secondary_button and not self._secondary_button_down
        self._primary_button_down = primary_button
        self._secondary_button_down = secondary_button
        # Secondary (B/Y) wins if both button edges arrive in the same sample.
        zero_requested = secondary_pressed
        home_requested = primary_pressed and not zero_requested
        return_requested = home_requested or zero_requested
        if return_requested:
            self.mapper.reset(require_release=True)
        mapping = self.mapper.update(
            frame, tcp_position, ee_rotation, now_ns=now_ns
        )

        if mapping.state != self._last_state:
            # A return edge also forces IDLE; one invalidation is sufficient for
            # both events and keeps generation changes deterministic.
            if not return_requested and not recovering_feedback:
                self._begin_generation()
            if mapping.state is TeleopState.ACTIVE:
                self._q_goal_rad = q_control_actual_rad.copy()
                # A Grip session starts from the measured posture. Synchronize
                # the shaped command and clear velocity so activation cannot
                # inherit motion from the previous IDLE/home command.
                self._qp_nominal_rad = q_control_actual_rad.copy()
                self._q_command_rad = q_control_actual_rad.copy()
                self._dq_command_rad_s.fill(0.0)
                self._dq_qp_rad_s.fill(0.0)
            else:
                self._q_goal_rad = q_control_actual_rad.copy()
                self._q_command_rad = q_control_actual_rad.copy()
                self._dq_command_rad_s.fill(0.0)
                self._dq_qp_rad_s.fill(0.0)
                self._last_wrist_violation_rad = 0.0
                if mapping.state in (TeleopState.WAITING, TeleopState.STALE):
                    self._gripper_goal_deg = gripper_actual_deg
                    self._gripper_command_deg = gripper_actual_deg
                    self._gripper_velocity_deg_s = 0.0
            self._last_state = mapping.state

        if return_requested:
            self._begin_generation()
            target = self.zero_q_rad if zero_requested else self.home_q_rad
            self._q_goal_rad = target.copy()
            self._last_wrist_violation_rad = 0.0

        # Consume a completed result before deciding whether the next request
        # is still in flight. This permits one new QP request per fresh sample.
        self._consume_latest_result(mapping.state)

        if mapping.state is TeleopState.ACTIVE and mapping.target is not None:
            sample_key = self._sample_key(frame)
            request_in_flight = (
                self._last_submitted_sequence > self._last_consumed_sequence
            )
            if (
                sample_key is not None
                and sample_key != self._last_submitted_sample
                and not request_in_flight
                and not mapping.reference_captured
            ):
                seed = self._q_goal_rad.copy()
                qp_dt_s = dt_s
                if self._last_qp_submission_ns is not None:
                    qp_dt_s = float(
                        np.clip(
                            (now_value_ns - self._last_qp_submission_ns) * 1e-9,
                            1e-6,
                            0.05,
                        )
                    )
                self._sequence += 1
                self._last_qp_request_actual_rad = q_control_actual_rad.copy()
                self._last_qp_request_dt_s = qp_dt_s
                self.worker.submit(
                    IKRequest(
                        generation=self._generation,
                        sequence=self._sequence,
                        sample_id=mapping.target.sample_id,
                        target_position=mapping.target.position,
                        target_rotation=mapping.target.rotation,
                        q_seed=seed,
                        q_actual=q_control_actual_rad,
                        dq_previous=self._dq_qp_rad_s.copy(),
                        dt=qp_dt_s,
                        q_nominal=(
                            q_control_actual_rad
                            if self._qp_nominal_rad is None
                            else self._qp_nominal_rad
                        ),
                        submitted_monotonic_ns=now_value_ns,
                    )
                )
                self._last_submitted_sample = sample_key
                self._last_submitted_sequence = self._sequence
                self._last_submitted_sample_id = mapping.target.sample_id
                self._last_qp_submission_ns = now_value_ns
                # Immediate/test workers can finish synchronously. Consume
                # that result as well without delaying the visible status.
                self._consume_latest_result(mapping.state)

        trigger = self._trigger(frame)
        tracking_fresh = mapping.state in (TeleopState.IDLE, TeleopState.ACTIVE)
        if tracking_fresh:
            self._gripper_goal_deg = (
                self.config.gripper_open_deg
                + trigger * (self.config.gripper_closed_deg - self.config.gripper_open_deg)
            )

        previous_q_command_rad = self._q_command_rad.copy()
        wrist_speed = (
            self.config.max_joint_speed_rad_s
            if self.config.wrist_speed_rad_s is None
            else self.config.wrist_speed_rad_s
        )
        wrist_acceleration = (
            self.config.max_joint_acceleration_rad_s2
            if self.config.wrist_acceleration_rad_s2 is None
            else self.config.wrist_acceleration_rad_s2
        )
        self._q_command_rad, self._dq_command_rad_s = shape_joint_position_command(
            previous_position=self._q_command_rad,
            previous_velocity=self._dq_command_rad_s,
            target_position=self._q_goal_rad,
            dt_s=dt_s,
            max_speed=np.concatenate(
                (
                    np.full(3, self.config.max_joint_speed_rad_s),
                    np.full(3, wrist_speed),
                )
            ),
            max_acceleration=np.concatenate(
                (
                    np.full(3, self.config.max_joint_acceleration_rad_s2),
                    np.full(3, wrist_acceleration),
                )
            ),
            lower_limit=self.lower_limit_rad,
            upper_limit=self.upper_limit_rad,
        )
        if self.config.max_command_feedback_error_deg is not None:
            wrist_feedback_error = (
                self.config.max_command_feedback_error_deg
                if self.config.wrist_command_feedback_error_deg is None
                else self.config.wrist_command_feedback_error_deg
            )
            max_tracking_error_rad = np.deg2rad(
                np.concatenate(
                    (
                        np.full(3, self.config.max_command_feedback_error_deg),
                        np.full(3, wrist_feedback_error),
                    )
                )
            )
            self._q_command_rad = bound_position_command_to_feedback(
                self._q_command_rad,
                q_actual_rad,
                max_tracking_error_rad,
                lower_limit=self.lower_limit_rad,
                upper_limit=self.upper_limit_rad,
            )
            self._dq_command_rad_s = (
                self._q_command_rad - previous_q_command_rad
            ) / dt_s
        gripper_position, gripper_velocity = shape_joint_position_command(
            previous_position=np.array([self._gripper_command_deg]),
            previous_velocity=np.array([self._gripper_velocity_deg_s]),
            target_position=np.array([self._gripper_goal_deg]),
            dt_s=dt_s,
            max_speed=np.array([self.config.gripper_max_speed_deg_s]),
            max_acceleration=np.array([self.config.gripper_max_acceleration_deg_s2]),
            lower_limit=np.array([min(self.config.gripper_open_deg, self.config.gripper_closed_deg)]),
            upper_limit=np.array([max(self.config.gripper_open_deg, self.config.gripper_closed_deg)]),
        )
        self._gripper_command_deg = float(gripper_position[0])
        self._gripper_velocity_deg_s = float(gripper_velocity[0])
        if self.config.max_command_feedback_error_deg is not None:
            gripper_feedback_error = (
                self.config.max_command_feedback_error_deg
                if self.config.gripper_command_feedback_error_deg is None
                else self.config.gripper_command_feedback_error_deg
            )
            unclipped_gripper_deg = self._gripper_command_deg
            self._gripper_command_deg = float(
                bound_position_command_to_feedback(
                    np.array([self._gripper_command_deg]),
                    np.array([gripper_actual_deg]),
                    gripper_feedback_error,
                    lower_limit=np.array(
                        [min(self.config.gripper_open_deg, self.config.gripper_closed_deg)]
                    ),
                    upper_limit=np.array(
                        [max(self.config.gripper_open_deg, self.config.gripper_closed_deg)]
                    ),
                )[0]
            )
            if self._gripper_command_deg != unclipped_gripper_deg:
                self._gripper_velocity_deg_s = 0.0

        command_deg = np.rad2deg(self._q_command_rad)
        target_deg = np.rad2deg(self._q_goal_rad)
        orientation_error_deg: float | None = None
        if mapping.state is TeleopState.ACTIVE and mapping.target is not None:
            orientation_error_deg = float(
                np.rad2deg(
                    Rotation.from_matrix(
                        ee_rotation @ mapping.target.rotation.T
                    ).magnitude()
                )
            )
        action = {f"{name}.pos": float(command_deg[index]) for index, name in enumerate(ARM_JOINT_NAMES)}
        action[f"{GRIPPER_NAME}.pos"] = self._gripper_command_deg
        ik_result = self._last_ik_result
        status = CartesianControlStatus(
            state=mapping.state,
            tracking=tracking_fresh,
            ik_success=None if ik_result is None else ik_result.success,
            ik_error_m=None if ik_result is None else ik_result.position_error_m,
            ik_reason="" if ik_result is None else ik_result.reason,
            submitted=self.worker.submitted,
            solved=self.worker.solved,
            rejected=self.worker.rejected,
            trigger=trigger,
            primary_button=primary_button,
            secondary_button=secondary_button,
            home_requested=home_requested,
            zero_requested=zero_requested,
            generation=self._generation,
            gripper_actual_deg=gripper_actual_deg,
            gripper_target_deg=self._gripper_goal_deg,
            gripper_command_deg=self._gripper_command_deg,
            actual_deg=np.concatenate((np.rad2deg(q_actual_rad), [gripper_actual_deg])),
            target_deg=np.concatenate((target_deg, [self._gripper_goal_deg])),
            command_deg=np.concatenate((command_deg, [self._gripper_command_deg])),
            orientation_error_deg=orientation_error_deg,
            wrist_clip_deg=float(np.rad2deg(self._last_wrist_violation_rad)),
            feedback_valid=True,
        )
        return action, status

    def _read_feedback(
        self, observation: dict[str, float]
    ) -> tuple[np.ndarray, float, str]:
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
            return (
                q_actual_rad,
                gripper_actual_deg,
                "non_finite:" + ",".join(invalid_names),
            )
        return q_actual_rad, gripper_actual_deg, ""

    def _hold_for_feedback_fault(
        self,
        raw_q_actual_rad: np.ndarray,
        raw_gripper_actual_deg: float,
        reason: str,
    ) -> tuple[dict[str, float] | None, CartesianControlStatus]:
        self._feedback_fault_count += 1
        self._feedback_fault_reason = reason
        if self._feedback_fault_count == 1:
            self.mapper.reset(require_release=True)
            self._begin_generation()
            self._last_state = TeleopState.HOLD
            self._primary_button_down = False
            self._secondary_button_down = False
            if self._q_command_rad is not None:
                self._q_goal_rad = self._q_command_rad.copy()
                assert self._dq_command_rad_s is not None
                self._dq_command_rad_s.fill(0.0)
                self._dq_qp_rad_s.fill(0.0)
                self._gripper_goal_deg = self._gripper_command_deg
                self._gripper_velocity_deg_s = 0.0

        action: dict[str, float] | None = None
        if self._q_command_rad is not None:
            command_deg = np.rad2deg(self._q_command_rad)
            action = {
                f"{name}.pos": float(command_deg[index])
                for index, name in enumerate(ARM_JOINT_NAMES)
            }
            action[f"{GRIPPER_NAME}.pos"] = self._gripper_command_deg
            target_rad = (
                self._q_command_rad
                if self._q_goal_rad is None
                else self._q_goal_rad
            )
        else:
            command_deg = np.full(6, np.nan, dtype=np.float64)
            target_rad = np.full(6, np.nan, dtype=np.float64)

        if self._last_valid_q_actual_rad is not None:
            status_q_actual_rad = self._last_valid_q_actual_rad
        else:
            status_q_actual_rad = raw_q_actual_rad
        status_gripper_actual_deg = (
            self._last_valid_gripper_actual_deg
            if self._last_valid_gripper_actual_deg is not None
            else raw_gripper_actual_deg
        )
        target_deg = np.rad2deg(target_rad)
        status = CartesianControlStatus(
            state=TeleopState.HOLD,
            tracking=False,
            ik_success=None,
            ik_error_m=None,
            ik_reason="feedback_hold",
            submitted=self.worker.submitted,
            solved=self.worker.solved,
            rejected=self.worker.rejected,
            trigger=0.0,
            primary_button=False,
            secondary_button=False,
            home_requested=False,
            zero_requested=False,
            generation=self._generation,
            gripper_actual_deg=float(status_gripper_actual_deg),
            gripper_target_deg=self._gripper_goal_deg,
            gripper_command_deg=self._gripper_command_deg,
            actual_deg=np.concatenate(
                (np.rad2deg(status_q_actual_rad), [status_gripper_actual_deg])
            ),
            target_deg=np.concatenate((target_deg, [self._gripper_goal_deg])),
            command_deg=np.concatenate(
                (command_deg, [self._gripper_command_deg])
            ),
            orientation_error_deg=None,
            wrist_clip_deg=float(np.rad2deg(self._last_wrist_violation_rad)),
            feedback_valid=False,
            feedback_fault_count=self._feedback_fault_count,
            feedback_fault_reason=reason,
            feedback_abort_requested=(
                self._feedback_fault_count
                >= self.config.feedback_fault_max_consecutive
            ),
        )
        return action, status

    def _begin_generation(self) -> None:
        self._generation += 1
        self.worker.clear()
        self._last_submitted_sample = None
        self._last_submitted_sequence = 0
        self._last_submitted_sample_id = None
        self._last_qp_submission_ns = None
        self._last_ik_result = None
        self._last_qp_request_actual_rad = None
        self._last_qp_request_dt_s = None
        self._dq_qp_rad_s.fill(0.0)

    def _consume_latest_result(self, state: TeleopState) -> None:
        result = self.worker.latest_result()
        if result is None or result.sequence <= self._last_consumed_sequence:
            return
        self._last_consumed_sequence = result.sequence
        if (
            result.generation != self._generation
            or state is not TeleopState.ACTIVE
            or result.sequence != self._last_submitted_sequence
            or result.sample_id != self._last_submitted_sample_id
            or result.solve_time_ms > self.config.qp_max_solve_time_ms
        ):
            return
        self._last_ik_result = result
        candidate = np.asarray(result.q_target_rad, dtype=np.float64)
        if candidate.shape != (6,) or not np.all(np.isfinite(candidate)):
            return
        if not result.success:
            # A failed QP keeps the complete previous six-axis goal.
            return
        candidate = np.clip(candidate, self.lower_limit_rad, self.upper_limit_rad)
        self._q_goal_rad = candidate
        if self._last_qp_request_actual_rad is not None and self._last_qp_request_dt_s is not None:
            self._dq_qp_rad_s = (
                candidate - self._last_qp_request_actual_rad
            ) / self._last_qp_request_dt_s

    def _sample_is_fresh(
        self, sample: ControllerSample | VRFrame | None, now_ns: int
    ) -> bool:
        if sample is None or (isinstance(sample, VRFrame) and not sample.is_tracking):
            return False
        age_ns = max(0, now_ns - int(sample.received_monotonic_ns))
        return age_ns <= int(self.config.stale_timeout_s * 1e9)

    @staticmethod
    def _sample_key(
        sample: ControllerSample | VRFrame | None,
    ) -> tuple[int, int, int] | None:
        if sample is None:
            return None
        return (
            int(sample.stream_epoch),
            int(sample.tracking_timestamp_ns),
            int(sample.received_monotonic_ns),
        )

    @staticmethod
    def _trigger(sample: ControllerSample | VRFrame | None) -> float:
        if sample is None:
            return 0.0
        value = sample.trigger
        return float(np.clip(value, 0.0, 1.0))


def vr_frame_from_raw_action(action: dict[str, object]) -> VRFrame:
    return VRFrame(
        grip_pos=np.asarray(action["grip_pos"], dtype=np.float64),
        grip_quat=np.asarray(action["grip_quat"], dtype=np.float64),
        squeeze=float(action.get("squeeze", 0.0)),
        trigger=float(action["trigger"]),
        is_tracking=bool(action["is_tracking"]),
        received_monotonic_ns=int(action["received_monotonic_ns"]),
        tracking_timestamp_ns=int(action.get("tracking_timestamp_ns", 0)),
        stream_epoch=int(action.get("stream_epoch", 0)),
        side=str(action.get("side", "right")),
        primary_button=bool(action.get("primary_button", False)),
        secondary_button=bool(action.get("secondary_button", False)),
        status=action.get("status"),
        head_pos=action.get("head_pos"),
        head_quat=action.get("head_quat"),
    )

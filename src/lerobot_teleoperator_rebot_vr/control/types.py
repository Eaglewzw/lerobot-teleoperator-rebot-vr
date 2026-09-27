from __future__ import annotations

from dataclasses import dataclass, field
from typing import Protocol

import numpy as np

from ..constants import (
    ARM_JOINT_NAMES,
    FOLLOWER_LOWER_DEG,
    FOLLOWER_UPPER_DEG,
    GRIPPER_NAME,
)
from ..ik.async_worker import IKRequest, IKResult
from ..vr.pose_mapping import TeleopState


# Match the LeRobot RebotB601Follower software limits, expressed in radians.
FOLLOWER_LOWER_RAD = np.deg2rad(FOLLOWER_LOWER_DEG)
FOLLOWER_UPPER_RAD = np.deg2rad(FOLLOWER_UPPER_DEG)
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
    ik_mode: str = "pose"
    qp_position_cost: float = 20.0
    qp_orientation_cost: float = 2.0
    qp_orientation_cost_min: float = 0.05
    qp_position_gain: float = 10.0
    qp_orientation_gain: float = 8.0
    qp_damping: float = 1e-3
    qp_damping_max: float = 0.1
    qp_smoothness_cost: float = 0.05
    qp_posture_cost: float = 0.01
    qp_use_sent_velocity: bool = False
    split_contour_weight: float = 1.0
    split_reference_speed_m_s: float = 0.0
    split_reference_acceleration_m_s2: float = 1.5
    singularity_threshold: float = 0.08
    singularity_critical_threshold: float = 0.02
    singularity_characteristic_length_m: float = 0.3
    joint_limit_margin_deg: float = 2.0
    qp_max_solve_time_ms: float = 8.0
    position_scale: float = 1.0
    linear_ff_max_speed_m_s: float = 0.3
    linear_ff_max_acceleration_m_s2: float = 1.0
    linear_ff_max_deceleration_m_s2: float = 4.0
    linear_ff_brake_confirm_s: float = 0.03
    linear_ff_jump_speed_m_s: float = 1.0
    linear_ff_jump_slack_m: float = 0.005
    orientation_scale: float = 1.0
    position_filter_hz: float = 0.0
    orientation_filter_hz: float = 0.0
    position_deadband_m: float = 0.0
    orientation_deadband_rad: float = 0.0
    grip_press_threshold: float = 0.85
    grip_release_threshold: float = 0.75
    stale_timeout_s: float = 0.2
    max_joint_speed_rad_s: float = 5.5
    max_joint_acceleration_rad_s2: float = 20.0
    wrist_speed_rad_s: float | None = 12.0
    wrist_acceleration_rad_s2: float | None = 60.0
    wrist_command_feedback_error_deg: float | None = None
    arm_command_lookahead_s: float = 0.05
    wrist_command_lookahead_s: float = 0.025
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
    gripper_open_deg: float = -180.0
    gripper_closed_deg: float = 0.0
    gripper_max_speed_deg_s: float = 90.0
    gripper_max_acceleration_deg_s2: float = 360.0
    max_command_feedback_error_deg: float | None = None

    def __post_init__(self) -> None:
        if self.qp_solver not in ("scipy", "osqp"):
            raise ValueError("qp_solver must be scipy or osqp")
        non_negative = np.asarray(
            (
                self.position_scale,
                self.orientation_scale,
                self.position_filter_hz,
                self.orientation_filter_hz,
                self.position_deadband_m,
                self.orientation_deadband_rad,
                self.qp_position_cost,
                self.qp_orientation_cost,
                self.qp_orientation_cost_min,
                self.qp_damping,
                self.qp_damping_max,
                self.qp_smoothness_cost,
                self.qp_posture_cost,
                self.split_reference_speed_m_s,
                self.singularity_threshold,
                self.singularity_critical_threshold,
                self.joint_limit_margin_deg,
                self.qp_max_solve_time_ms,
            ),
            dtype=np.float64,
        )
        if not np.all(np.isfinite(non_negative)) or np.any(non_negative < 0.0):
            raise ValueError("mapping values and IK damping must be non-negative")
        positive = np.asarray(
            (
                self.linear_ff_max_speed_m_s,
                self.split_contour_weight,
                self.split_reference_acceleration_m_s2,
                self.linear_ff_max_acceleration_m_s2,
                self.linear_ff_max_deceleration_m_s2,
                self.linear_ff_brake_confirm_s,
                self.linear_ff_jump_speed_m_s,
                self.linear_ff_jump_slack_m,
                self.stale_timeout_s,
                self.max_joint_speed_rad_s,
                self.max_joint_acceleration_rad_s2,
                self.gripper_max_speed_deg_s,
                self.gripper_max_acceleration_deg_s2,
                self.qp_max_solve_time_ms,
                self.singularity_characteristic_length_m,
                self.qp_position_gain,
                self.qp_orientation_gain,
                self.arm_command_lookahead_s,
                self.wrist_command_lookahead_s,
            ),
            dtype=np.float64,
        )
        if not np.all(np.isfinite(positive)) or np.any(positive <= 0.0):
            raise ValueError("control rates and motion limits must be finite and positive")
        if self.qp_position_cost <= 0.0:
            raise ValueError("QP position cost must be finite and positive")
        if self.split_contour_weight < 1:
            raise ValueError("split_contour_weight must be at least 1")
        optional_positive = np.asarray(
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
        if not np.all(np.isfinite(optional_positive)) or np.any(
            optional_positive <= 0.0
        ):
            raise ValueError("wrist and gripper control limits must be finite and positive")
        if self.ik_mode not in ("pose", "position", "split"):
            raise ValueError("ik_mode must be pose, position, or split")
        if (
            self.qp_orientation_cost > 0
            and self.qp_orientation_cost_min > self.qp_orientation_cost
        ):
            raise ValueError("minimum orientation cost cannot exceed normal cost")
        if self.qp_damping_max < self.qp_damping:
            raise ValueError("maximum QP damping cannot be below minimum damping")
        if self.singularity_threshold <= self.singularity_critical_threshold:
            raise ValueError(
                "singularity threshold must exceed the critical threshold"
            )
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
    gripper_trigger_active: bool
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
    ik_mode: str = "pose"
    wrist_clip_deg: float | None = None
    sigma_min: float | None = None
    condition_number: float | None = None
    current_damping: float | None = None
    current_orientation_weight: float | None = None
    dq_norm_rad_s: float | None = None
    qp_joint_velocity_rad_s: np.ndarray | None = None
    qp_solve_time_ms: float | None = None
    qp_result_age_ms: float | None = None
    target_linear_velocity_m_s: np.ndarray | None = None
    target_angular_velocity_rad_s: np.ndarray | None = None
    tcp_position_error_m: float | None = None
    tcp_actual_position_m: np.ndarray | None = None
    tcp_target_position_m: np.ndarray | None = None
    controller_position_m: np.ndarray | None = None
    controller_position_frame: str = ""
    mapping_sample_id: int | None = None
    velocity_diagnostics: dict[str, object] = field(default_factory=dict)
    tcp_actual_rotvec_rad: np.ndarray | None = None
    tcp_target_rotvec_rad: np.ndarray | None = None
    feedback_valid: bool = True
    feedback_fault_count: int = 0
    feedback_fault_reason: str = ""
    feedback_abort_requested: bool = False
    tracking_sample_age_ms: float | None = None
    control_loop_hz: float | None = None
    feedback_read_ms: float | None = None
    send_action_ms: float | None = None
    cycle_work_ms: float | None = None
    loop_started_monotonic_ns: int | None = None
    tracking_sample_received_monotonic_ns: int | None = None
    tracking_sample_published_monotonic_ns: int | None = None
    tracking_sample_pickup_monotonic_ns: int | None = None
    tracking_timestamp_ns: int | None = None
    tracking_stream_epoch: int | None = None
    feedback_started_monotonic_ns: int | None = None
    feedback_finished_monotonic_ns: int | None = None
    controller_started_monotonic_ns: int | None = None
    controller_finished_monotonic_ns: int | None = None
    command_send_started_monotonic_ns: int | None = None
    command_send_finished_monotonic_ns: int | None = None
    ik_sequence: int | None = None
    ik_sample_id: int | None = None
    ik_result_consumed_this_cycle: bool = False
    ik_result_applied_this_cycle: bool = False
    ik_sample_received_monotonic_ns: int | None = None
    ik_submitted_monotonic_ns: int | None = None
    ik_worker_started_monotonic_ns: int | None = None
    ik_completed_monotonic_ns: int | None = None
    ik_consumed_monotonic_ns: int | None = None
    vr_decode_ms: float | None = None
    latest_sample_wait_ms: float | None = None
    tracking_receive_to_pickup_ms: float | None = None
    fk_ms: float | None = None
    pose_mapping_ms: float | None = None
    qp_coordination_ms: float | None = None
    command_shaping_ms: float | None = None
    controller_update_ms: float | None = None
    ik_sample_to_submit_ms: float | None = None
    ik_queue_wait_ms: float | None = None
    ik_worker_total_ms: float | None = None
    ik_result_wait_ms: float | None = None
    ik_sample_to_command_ready_ms: float | None = None
    tracking_receive_to_send_ms: float | None = None
    ik_receive_to_send_ms: float | None = None
    command_to_next_feedback_ms: float | None = None
    motor_control_mode: str = "pos_vel"
    mit_desired_velocity_rad_s: np.ndarray | None = None
    mit_gravity_torque_nm: np.ndarray | None = None
    mit_feedforward_torque_nm: np.ndarray | None = None

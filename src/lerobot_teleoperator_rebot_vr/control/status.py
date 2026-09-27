from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation

from ..ik.async_worker import IKResult
from ..ik.coordination import QPRequestCoordinator
from ..vr.adapter import VR_SAMPLE_TYPES
from ..vr.tracking import ControllerSample
from ..vr.pose_mapping import PoseMappingUpdate, TeleopState
from .gripper import GripperController
from .types import CartesianControlConfig, CartesianControlStatus, IKWorker


def finite_ik_diagnostic(
    result: IKResult | None,
    name: str,
    *,
    allow_infinite: bool = False,
) -> float | None:
    if result is None:
        return None
    value = float(getattr(result, name))
    if np.isfinite(value) or (allow_infinite and np.isinf(value)):
        return value
    return None


def elapsed_ms(start_ns: int, end_ns: int) -> float | None:
    if start_ns <= 0 or end_ns <= 0:
        return None
    return max(0.0, (end_ns - start_ns) * 1e-6)


def build_running_status(
    *,
    mapping: PoseMappingUpdate,
    frame: VR_SAMPLE_TYPES | None,
    now_ns: int,
    dt_s: float,
    tracking_fresh: bool,
    trigger: float,
    primary_button: bool,
    secondary_button: bool,
    home_requested: bool,
    zero_requested: bool,
    q_actual_rad: np.ndarray,
    q_goal_rad: np.ndarray,
    q_command_rad: np.ndarray,
    gripper_actual_deg: float,
    gripper: GripperController,
    tcp_position_m: np.ndarray,
    ee_rotation: np.ndarray,
    worker: IKWorker,
    qp: QPRequestCoordinator,
    config: CartesianControlConfig,
) -> CartesianControlStatus:
    ik_result = qp.last_result
    orientation_error_deg: float | None = None
    tcp_position_error_m: float | None = None
    if mapping.state is TeleopState.ACTIVE and mapping.target is not None:
        tcp_position_error_m = float(
            np.linalg.norm(mapping.target.position - tcp_position_m)
        )
        orientation_error_deg = float(
            np.rad2deg(
                Rotation.from_matrix(
                    ee_rotation @ mapping.target.rotation.T
                ).magnitude()
            )
        )
        if (
            config.ik_mode == "split"
            and ik_result is not None
            and np.isfinite(ik_result.orientation_error_rad)
        ):
            orientation_error_deg = float(
                np.rad2deg(ik_result.orientation_error_rad)
            )

    consumed_ns = qp.last_result_consumed_monotonic_ns
    sample_received_ns = (
        0 if ik_result is None else ik_result.sample_received_monotonic_ns
    )
    submitted_ns = 0 if ik_result is None else ik_result.submitted_monotonic_ns
    worker_started_ns = (
        0 if ik_result is None else ik_result.worker_started_monotonic_ns
    )
    completed_ns = 0 if ik_result is None else ik_result.completed_monotonic_ns
    command_deg = np.rad2deg(q_command_rad)
    target_deg = np.rad2deg(q_goal_rad)
    return CartesianControlStatus(
        state=mapping.state,
        velocity_diagnostics={**qp.request_diagnostics, **qp.result_diagnostics},
        tracking=tracking_fresh,
        ik_success=None if ik_result is None else ik_result.success,
        ik_error_m=None if ik_result is None else ik_result.position_error_m,
        ik_reason="" if ik_result is None else ik_result.reason,
        submitted=worker.submitted,
        solved=worker.solved,
        rejected=worker.rejected,
        trigger=trigger,
        gripper_trigger_active=tracking_fresh and gripper.trigger_active,
        primary_button=primary_button,
        secondary_button=secondary_button,
        home_requested=home_requested,
        zero_requested=zero_requested,
        generation=qp.generation,
        gripper_actual_deg=gripper_actual_deg,
        gripper_target_deg=gripper.goal_deg,
        gripper_command_deg=gripper.command_deg,
        actual_deg=np.concatenate(
            (np.rad2deg(q_actual_rad), [gripper_actual_deg])
        ),
        target_deg=np.concatenate((target_deg, [gripper.goal_deg])),
        command_deg=np.concatenate((command_deg, [gripper.command_deg])),
        orientation_error_deg=orientation_error_deg,
        ik_mode=config.ik_mode,
        wrist_clip_deg=(
            None
            if ik_result is None or not np.isfinite(ik_result.wrist_clip_rad)
            else float(np.rad2deg(ik_result.wrist_clip_rad))
        ),
        sigma_min=finite_ik_diagnostic(ik_result, "sigma_min"),
        condition_number=finite_ik_diagnostic(
            ik_result, "condition_number", allow_infinite=True
        ),
        current_damping=finite_ik_diagnostic(ik_result, "damping"),
        current_orientation_weight=finite_ik_diagnostic(
            ik_result, "orientation_weight"
        ),
        dq_norm_rad_s=(
            None
            if ik_result is None or ik_result.joint_velocity_rad_s is None
            else float(np.linalg.norm(ik_result.joint_velocity_rad_s))
        ),
        qp_joint_velocity_rad_s=(
            None if ik_result is None else qp.accepted_joint_velocity_rad_s
        ),
        qp_solve_time_ms=finite_ik_diagnostic(ik_result, "solve_time_ms"),
        qp_result_age_ms=qp.last_result_age_ms,
        target_linear_velocity_m_s=qp.target_linear_velocity_m_s.copy(),
        target_angular_velocity_rad_s=qp.target_angular_velocity_rad_s.copy(),
        tcp_position_error_m=tcp_position_error_m,
        tcp_actual_position_m=tcp_position_m.copy(),
        tcp_target_position_m=(
            None if mapping.target is None else mapping.target.position.copy()
        ),
        # Snapshot the fresh input before axis mapping/scaling/filtering.
        # Legacy VRFrame positions have already been transformed upstream.
        controller_position_m=(
            None if frame is None or not tracking_fresh else np.asarray(
                frame.position if isinstance(frame, ControllerSample) else frame.grip_pos,
                dtype=np.float64,
            ).copy()
        ),
        controller_position_frame=(
            "" if frame is None or not tracking_fresh else
            "xr" if isinstance(frame, ControllerSample) else "robot_base"
        ),
        mapping_sample_id=None if mapping.target is None else mapping.target.sample_id,
        tcp_actual_rotvec_rad=Rotation.from_matrix(ee_rotation).as_rotvec(),
        tcp_target_rotvec_rad=(
            None
            if mapping.target is None
            else Rotation.from_matrix(mapping.target.rotation).as_rotvec()
        ),
        feedback_valid=True,
        tracking_sample_age_ms=(
            None
            if frame is None
            else max(0.0, (now_ns - int(frame.received_monotonic_ns)) * 1e-6)
        ),
        control_loop_hz=1.0 / dt_s,
        tracking_sample_received_monotonic_ns=(
            None if frame is None else int(frame.received_monotonic_ns)
        ),
        tracking_sample_published_monotonic_ns=(
            None
            if frame is None or int(getattr(frame, "published_monotonic_ns", 0)) <= 0
            else int(getattr(frame, "published_monotonic_ns"))
        ),
        tracking_timestamp_ns=(
            None if frame is None else int(frame.tracking_timestamp_ns)
        ),
        tracking_stream_epoch=(
            None if frame is None else int(frame.stream_epoch)
        ),
        ik_sequence=None if ik_result is None else ik_result.sequence,
        ik_sample_id=None if ik_result is None else ik_result.sample_id,
        # The controller compares the consumed timestamp at cycle boundaries
        # and replaces this value after building the status.
        ik_result_consumed_this_cycle=False,
        ik_sample_received_monotonic_ns=(
            None if sample_received_ns <= 0 else sample_received_ns
        ),
        ik_submitted_monotonic_ns=None if submitted_ns <= 0 else submitted_ns,
        ik_worker_started_monotonic_ns=(
            None if worker_started_ns <= 0 else worker_started_ns
        ),
        ik_completed_monotonic_ns=None if completed_ns <= 0 else completed_ns,
        ik_consumed_monotonic_ns=consumed_ns,
        ik_sample_to_submit_ms=elapsed_ms(sample_received_ns, submitted_ns),
        ik_queue_wait_ms=elapsed_ms(submitted_ns, worker_started_ns),
        ik_worker_total_ms=elapsed_ms(worker_started_ns, completed_ns),
        ik_result_wait_ms=(
            None
            if consumed_ns is None
            else elapsed_ms(completed_ns, consumed_ns)
        ),
        ik_sample_to_command_ready_ms=(
            None
            if consumed_ns is None
            else elapsed_ms(sample_received_ns, consumed_ns)
        ),
    )

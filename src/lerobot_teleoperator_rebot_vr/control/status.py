from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation

from ..ik.async_worker import IKResult
from ..ik.coordination import QPRequestCoordinator
from ..vr.adapter import VR_SAMPLE_TYPES
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

    ik_result = qp.last_result
    command_deg = np.rad2deg(q_command_rad)
    target_deg = np.rad2deg(q_goal_rad)
    return CartesianControlStatus(
        state=mapping.state,
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
            None
            if ik_result is None or ik_result.joint_velocity_rad_s is None
            else ik_result.joint_velocity_rad_s.copy()
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
    )

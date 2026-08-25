from __future__ import annotations

import time

import numpy as np

from .arm_command import update_arm_position_command
from ..ik.async_worker import LatestOnlyQPIKWorker
from ..ik.coordination import QPRequestCoordinator
from ..ik.kinematics import FullBodyQPIKSolver
from ..vr.adapter import sample_is_fresh, trigger_value, vr_frame_from_raw_action
from ..vr.models import VRFrame
from ..vr.pose_mapping import RelativePoseMapper, TeleopState
from ..vr.tracking import ControllerSample
from .types import (
    ARM_JOINT_NAMES,
    FEEDBACK_LIMIT_TOLERANCE_RAD,
    FOLLOWER_LOWER_RAD,
    FOLLOWER_UPPER_RAD,
    GRIPPER_NAME,
    CartesianControlConfig,
    CartesianControlStatus,
    IKWorker,
)
from .feedback import feedback_limit_error, read_robot_feedback
from .gripper import GripperController
from .joint_command import (
    bound_position_command_to_feedback,
    shape_joint_position_command,
)
from .startup import reference_initial_q_to_dm
from .status import build_running_status


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
                ik_mode=self.config.ik_mode,
                position_cost=self.config.qp_position_cost,
                orientation_cost=self.config.qp_orientation_cost,
                orientation_cost_min=self.config.qp_orientation_cost_min,
                position_gain=self.config.qp_position_gain,
                orientation_gain=self.config.qp_orientation_gain,
                damping_min=self.config.qp_damping,
                damping_max=self.config.qp_damping_max,
                smoothness_cost=self.config.qp_smoothness_cost,
                posture_cost=self.config.qp_posture_cost,
                singularity_threshold=self.config.singularity_threshold,
                singularity_critical_threshold=(
                    self.config.singularity_critical_threshold
                ),
                singularity_characteristic_length_m=(
                    self.config.singularity_characteristic_length_m
                ),
                joint_limit_margin_rad=np.deg2rad(self.config.joint_limit_margin_deg),
                max_solve_time_ms=self.config.qp_max_solve_time_ms,
            )
            speed = np.concatenate(
                (
                    np.full(3, self.config.max_joint_speed_rad_s),
                    np.full(
                        3,
                        self.config.wrist_speed_rad_s
                        or self.config.max_joint_speed_rad_s,
                    ),
                )
            )
            acceleration = np.concatenate(
                (
                    np.full(3, self.config.max_joint_acceleration_rad_s2),
                    np.full(
                        3,
                        self.config.wrist_acceleration_rad_s2
                        or self.config.max_joint_acceleration_rad_s2,
                    ),
                )
            )
            self.worker = LatestOnlyQPIKWorker(
                qp,
                max_joint_speed_rad_s=speed,
                max_joint_acceleration_rad_s2=acceleration,
            )

        self.qp = QPRequestCoordinator(
            self.worker,
            self.config,
            self.lower_limit_rad,
            self.upper_limit_rad,
        )
        self.gripper = GripperController()
        self._last_state = TeleopState.WAITING
        self._q_goal_rad: np.ndarray | None = None
        self._qp_nominal_rad: np.ndarray | None = None
        self._q_command_rad: np.ndarray | None = None
        self._dq_command_rad_s: np.ndarray | None = None
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
        q_actual_rad, gripper_actual_deg, feedback_error = read_robot_feedback(
            observation
        )
        if feedback_error:
            return self._hold_for_feedback_fault(
                q_actual_rad, gripper_actual_deg, feedback_error
            )
        limit_error = feedback_limit_error(
            q_actual_rad,
            self.feedback_lower_limit_rad,
            self.feedback_upper_limit_rad,
        )
        if limit_error:
            return self._hold_for_feedback_fault(
                q_actual_rad,
                gripper_actual_deg,
                limit_error,
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
            self.gripper.synchronize_to_feedback(gripper_actual_deg)

        if recovering_feedback:
            self.mapper.reset(require_release=True)
            self._begin_generation()
            self._q_goal_rad = q_control_actual_rad.copy()
            self._qp_nominal_rad = q_control_actual_rad.copy()
            self._q_command_rad = q_control_actual_rad.copy()
            self._dq_command_rad_s.fill(0.0)
            self.qp.reset_velocity()
            self.gripper.reset_after_feedback_recovery(gripper_actual_deg)
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
        sample_fresh = sample_is_fresh(
            frame, now_value_ns, self.config.stale_timeout_s
        )
        trigger = trigger_value(frame)
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
                self.qp.reset_velocity()
            else:
                self._q_goal_rad = q_control_actual_rad.copy()
                self._q_command_rad = q_control_actual_rad.copy()
                self._dq_command_rad_s.fill(0.0)
                self.qp.reset_velocity()
                if mapping.state in (TeleopState.WAITING, TeleopState.STALE):
                    self.gripper.reset_for_unavailable_tracking(
                        gripper_actual_deg
                    )
            self._last_state = mapping.state

        if mapping.reference_captured and mapping.target is not None:
            self.qp.capture_velocity_reference(mapping.target, frame)

        if return_requested:
            self._begin_generation()
            target = self.zero_q_rad if zero_requested else self.home_q_rad
            self._q_goal_rad = target.copy()
            if zero_requested:
                # B/Y returns the complete robot to zero and disarms Trigger,
                # so analog noise cannot overwrite the closed gripper target.
                self.gripper.request_closed(
                    self.config.gripper_closed_deg, trigger
                )

        # Consume a completed result before deciding whether the next request
        # is still in flight. This permits one new QP request per fresh sample.
        qp_goal_rad = self.qp.consume_latest(
            state=mapping.state,
            q_actual_rad=q_control_actual_rad,
            now_ns=now_value_ns,
        )
        if qp_goal_rad is not None:
            self._q_goal_rad = qp_goal_rad

        if (
            mapping.state is TeleopState.ACTIVE
            and mapping.target is not None
            and not mapping.reference_captured
        ):
            submitted = self.qp.submit_if_ready(
                target=mapping.target,
                frame=frame,
                q_seed_rad=self._q_goal_rad.copy(),
                q_actual_rad=q_control_actual_rad,
                q_nominal_rad=(
                    q_control_actual_rad
                    if self._qp_nominal_rad is None
                    else self._qp_nominal_rad
                ),
                dt_s=dt_s,
                now_ns=now_value_ns,
            )
            if submitted:
                # Immediate/test workers can finish synchronously. Consume
                # that result as well without delaying the visible status.
                qp_goal_rad = self.qp.consume_latest(
                    state=mapping.state,
                    q_actual_rad=q_control_actual_rad,
                    now_ns=now_value_ns,
                )
                if qp_goal_rad is not None:
                    self._q_goal_rad = qp_goal_rad

        tracking_fresh = mapping.state in (TeleopState.IDLE, TeleopState.ACTIVE)
        self.gripper.update_trigger_target(
            tracking_fresh=tracking_fresh,
            trigger=trigger,
            open_deg=self.config.gripper_open_deg,
            closed_deg=self.config.gripper_closed_deg,
        )

        self._q_command_rad, self._dq_command_rad_s = update_arm_position_command(
            previous_position_rad=self._q_command_rad,
            previous_velocity_rad_s=self._dq_command_rad_s,
            target_position_rad=self._q_goal_rad,
            actual_position_rad=q_actual_rad,
            state=mapping.state,
            dt_s=dt_s,
            lower_limit_rad=self.lower_limit_rad,
            upper_limit_rad=self.upper_limit_rad,
            config=self.config,
            shape_fn=shape_joint_position_command,
            bound_fn=bound_position_command_to_feedback,
        )
        gripper_feedback_error_deg: float | None = None
        if self.config.max_command_feedback_error_deg is not None:
            gripper_feedback_error_deg = (
                self.config.max_command_feedback_error_deg
                if self.config.gripper_command_feedback_error_deg is None
                else self.config.gripper_command_feedback_error_deg
            )
        self.gripper.update_command(
            actual_deg=gripper_actual_deg,
            dt_s=dt_s,
            open_deg=self.config.gripper_open_deg,
            closed_deg=self.config.gripper_closed_deg,
            max_speed_deg_s=self.config.gripper_max_speed_deg_s,
            max_acceleration_deg_s2=(
                self.config.gripper_max_acceleration_deg_s2
            ),
            feedback_error_deg=gripper_feedback_error_deg,
            shape_fn=shape_joint_position_command,
            bound_fn=bound_position_command_to_feedback,
        )

        command_deg = np.rad2deg(self._q_command_rad)
        action = {
            f"{name}.pos": float(command_deg[index])
            for index, name in enumerate(ARM_JOINT_NAMES)
        }
        action[f"{GRIPPER_NAME}.pos"] = self.gripper.command_deg
        status = build_running_status(
            mapping=mapping,
            frame=frame,
            now_ns=now_value_ns,
            dt_s=dt_s,
            tracking_fresh=tracking_fresh,
            trigger=trigger,
            primary_button=primary_button,
            secondary_button=secondary_button,
            home_requested=home_requested,
            zero_requested=zero_requested,
            q_actual_rad=q_actual_rad,
            q_goal_rad=self._q_goal_rad,
            q_command_rad=self._q_command_rad,
            gripper_actual_deg=gripper_actual_deg,
            gripper=self.gripper,
            tcp_position_m=tcp_position,
            ee_rotation=ee_rotation,
            worker=self.worker,
            qp=self.qp,
            config=self.config,
        )
        return action, status

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
            self.gripper.enter_feedback_hold(
                hold_command=self._q_command_rad is not None
            )
            self._last_state = TeleopState.HOLD
            self._primary_button_down = False
            self._secondary_button_down = False
            if self._q_command_rad is not None:
                self._q_goal_rad = self._q_command_rad.copy()
                assert self._dq_command_rad_s is not None
                self._dq_command_rad_s.fill(0.0)
                self.qp.reset_velocity()

        action: dict[str, float] | None = None
        if self._q_command_rad is not None:
            command_deg = np.rad2deg(self._q_command_rad)
            action = {
                f"{name}.pos": float(command_deg[index])
                for index, name in enumerate(ARM_JOINT_NAMES)
            }
            action[f"{GRIPPER_NAME}.pos"] = self.gripper.command_deg
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
            gripper_trigger_active=False,
            primary_button=False,
            secondary_button=False,
            home_requested=False,
            zero_requested=False,
            generation=self.qp.generation,
            gripper_actual_deg=float(status_gripper_actual_deg),
            gripper_target_deg=self.gripper.goal_deg,
            gripper_command_deg=self.gripper.command_deg,
            actual_deg=np.concatenate(
                (np.rad2deg(status_q_actual_rad), [status_gripper_actual_deg])
            ),
            target_deg=np.concatenate((target_deg, [self.gripper.goal_deg])),
            command_deg=np.concatenate(
                (command_deg, [self.gripper.command_deg])
            ),
            orientation_error_deg=None,
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
        self.qp.begin_generation()

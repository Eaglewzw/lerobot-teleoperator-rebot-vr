"""LeRobot registration wrapper for feedback-synchronized reBot VR control."""

from __future__ import annotations

import logging
import threading
import time
from typing import Any

import numpy as np
from lerobot.teleoperators.teleoperator import Teleoperator

try:
    from lerobot.lerobot_types import RobotAction
except ImportError:  # LeRobot 0.6.0/0.6.1 compatibility
    from lerobot.types import RobotAction

from .cartesian_controller import (
    CartesianControlConfig,
    FullBodyQPIKController,
    vr_frame_from_raw_action,
)
from .config_rebot_vr import (
    DEFAULT_BASE_T_ANCHOR,
    REBOT_JOINTS,
    RebotVRTeleopConfig,
)
from .kinematics import B601Kinematics
from .vr_controller import VRController, make_vr_controller


logger = logging.getLogger(__name__)


class RebotVRTeleop(Teleoperator):
    """Registered feedback-synchronized full-body QP controller.

    LeRobot 0.6's generic B601 loops do not call ``send_feedback``. In those
    loops ``get_action`` fails closed instead of falling back to open-loop joint
    heuristics. The packaged ``rebot-vr-teleoperate`` runner supplies feedback
    every cycle and is the supported real-robot entry point.
    """

    config_class = RebotVRTeleopConfig
    name = "rebot_vr"

    def __init__(
        self,
        config: RebotVRTeleopConfig,
        controller: VRController | None = None,
        arm_controller: FullBodyQPIKController | None = None,
    ) -> None:
        super().__init__(config)
        self.config = config
        self._controller = controller
        self._arm_controller = arm_controller
        self._kinematics: B601Kinematics | None = None
        self._feedback_lock = threading.Lock()
        self._latest_feedback: dict[str, float] | None = None
        self._last_update_s: float | None = None
        self._connected = False

    @property
    def action_features(self) -> dict[str, type]:
        return {f"{joint}.pos": float for joint in REBOT_JOINTS}

    @property
    def feedback_features(self) -> dict[str, type]:
        return {f"{joint}.pos": float for joint in REBOT_JOINTS}

    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        pass

    def connect(self, calibrate: bool = True) -> None:
        del calibrate
        if self.is_connected:
            raise RuntimeError("RebotVRTeleop is already connected")

        if self._arm_controller is None:
            self._kinematics = B601Kinematics()
            control_config = CartesianControlConfig(
                qp_solver=self.config.qp_solver,
                ik_mode=self.config.ik_mode,
                qp_position_cost=self.config.qp_position_cost,
                qp_orientation_cost=self.config.qp_orientation_cost,
                qp_orientation_cost_min=self.config.qp_orientation_cost_min,
                qp_position_gain=self.config.qp_position_gain,
                qp_orientation_gain=self.config.qp_orientation_gain,
                qp_damping=self.config.qp_damping,
                qp_damping_max=self.config.qp_damping_max,
                qp_smoothness_cost=self.config.qp_smoothness_cost,
                qp_posture_cost=self.config.qp_posture_cost,
                singularity_threshold=self.config.singularity_threshold,
                singularity_critical_threshold=(
                    self.config.singularity_critical_threshold
                ),
                singularity_characteristic_length_m=(
                    self.config.singularity_characteristic_length_m
                ),
                joint_limit_margin_deg=self.config.joint_limit_margin_deg,
                qp_max_solve_time_ms=self.config.qp_max_solve_time_ms,
                position_scale=self.config.position_scale,
                orientation_scale=self.config.orientation_scale,
                position_filter_hz=self.config.position_filter_hz,
                orientation_filter_hz=self.config.orientation_filter_hz,
                position_deadband_m=self.config.position_deadband_m,
                orientation_deadband_rad=self.config.orientation_deadband_rad,
                grip_press_threshold=self.config.clutch_threshold,
                grip_release_threshold=self.config.clutch_release_threshold,
                stale_timeout_s=self.config.stale_timeout,
                max_joint_speed_rad_s=self.config.max_joint_speed_rad_s,
                max_joint_acceleration_rad_s2=self.config.max_joint_acceleration_rad_s2,
                wrist_speed_rad_s=self.config.wrist_speed_rad_s,
                wrist_acceleration_rad_s2=self.config.wrist_acceleration_rad_s2,
                arm_command_lookahead_s=self.config.arm_command_lookahead_s,
                wrist_command_lookahead_s=self.config.wrist_command_lookahead_s,
                feedback_fault_max_consecutive=(
                    self.config.feedback_fault_max_consecutive
                ),
                initial_q_rad=tuple(self.config.initial_q_rad),
                gripper_open_deg=self.config.gripper_open,
                gripper_closed_deg=self.config.gripper_closed,
                gripper_max_speed_deg_s=self.config.gripper_max_speed_deg_s,
                gripper_max_acceleration_deg_s2=(
                    self.config.gripper_max_acceleration_deg_s2
                ),
            )
            self._arm_controller = FullBodyQPIKController(
                self._kinematics,
                xr_to_base_rotation=np.asarray(
                    DEFAULT_BASE_T_ANCHOR, dtype=np.float64
                )[:3, :3],
                config=control_config,
            )

        controller = self._controller or make_vr_controller(self.config)
        try:
            controller.connect()
            self._arm_controller.start()
        except Exception:
            if self._arm_controller is not None:
                try:
                    self._arm_controller.stop()
                except Exception:
                    logger.exception("arm controller cleanup failed after connect error")
            try:
                controller.disconnect()
            except Exception:
                logger.exception("VR controller cleanup failed after connect error")
            if self._kinematics is not None:
                self._kinematics.close()
                self._kinematics = None
                self._arm_controller = None
            raise
        self._controller = controller
        with self._feedback_lock:
            self._latest_feedback = None
        self._last_update_s = None
        self._connected = True
        logger.info(
            "reBot VR connected: backend=%s hand=%s; robot feedback is required",
            self.config.vr_backend,
            self.config.hand_side,
        )

    def get_action(self) -> RobotAction:
        if not self.is_connected or self._controller is None or self._arm_controller is None:
            raise RuntimeError("RebotVRTeleop is not connected")
        with self._feedback_lock:
            feedback = (
                None
                if self._latest_feedback is None
                else dict(self._latest_feedback)
            )
            self._latest_feedback = None
        if feedback is None:
            raise RuntimeError(
                "reBot VR requires current robot feedback before each action; "
                "LeRobot's generic B601 loop does not provide it. Use "
                "'rebot-vr-teleoperate' for safe Cartesian control."
            )

        sample = self._controller.latest_sample()
        if sample is None:
            raw = self._controller.get_action()
            frame = vr_frame_from_raw_action(raw)
        else:
            frame = sample
        now_s = time.monotonic()
        dt_s = (
            1.0 / 30.0
            if self._last_update_s is None
            else max(0.0, now_s - self._last_update_s)
        )
        self._last_update_s = now_s
        action, _status = self._arm_controller.update(frame, feedback, dt_s)
        return action

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        if not self.is_connected:
            raise RuntimeError("RebotVRTeleop is not connected")
        required = tuple(f"{joint}.pos" for joint in REBOT_JOINTS)
        try:
            normalized = {key: float(feedback[key]) for key in required}
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError("feedback must contain all seven finite joint positions") from exc
        if not np.all(np.isfinite(tuple(normalized.values()))):
            raise ValueError("feedback must contain all seven finite joint positions")
        with self._feedback_lock:
            self._latest_feedback = normalized

    def disconnect(self) -> None:
        controller = self._controller
        arm_controller = self._arm_controller
        self._connected = False
        with self._feedback_lock:
            self._latest_feedback = None
        self._last_update_s = None
        try:
            if arm_controller is not None:
                arm_controller.stop()
        finally:
            try:
                if controller is not None:
                    controller.disconnect()
            finally:
                if self._kinematics is not None:
                    self._kinematics.close()
                    self._kinematics = None
                    self._arm_controller = None
        self._controller = None
        logger.info("reBot VR disconnected")


__all__ = ["RebotVRTeleop"]

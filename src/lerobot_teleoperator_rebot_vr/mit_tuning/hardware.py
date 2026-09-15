"""LeRobot/MotorBridge boundary for the otherwise hardware-agnostic tuner."""

from __future__ import annotations

import time
from pathlib import Path

import numpy as np

from ..control.mit import MITCommandDispatcher
from ..runtime.shutdown import ManagedFollower, ShutdownPolicy
from .models import (
    ARM_JOINT_NAMES,
    CommandFrame,
    FeedbackFrame,
)


class LeRobotMITTuningRobot:
    """Expose only feedback, MIT command, and shutdown operations to the tuner."""

    def __init__(
        self,
        *,
        port: str,
        robot_id: str,
        can_adapter: str,
        dm_serial_baud: int,
        kp: np.ndarray,
        kd: np.ndarray,
        torque_limit_nm: np.ndarray,
        gravity_scale: float,
        gravity_ramp_s: float,
        dynamics_urdf: str | Path | None,
        max_relative_target_deg: float,
        gripper_torque_ratio: float,
        shutdown_report_path: Path,
    ) -> None:
        try:
            from lerobot.robots.rebot_b601_follower import (
                RebotB601Follower,
                RebotB601FollowerRobotConfig,
            )
        except ImportError as exc:
            raise ImportError(
                "rebot-mit-tune requires LeRobot with rebot_b601_follower support"
            ) from exc

        config = RebotB601FollowerRobotConfig(
            port=port,
            id=robot_id,
            can_adapter=can_adapter,
            dm_serial_baud=dm_serial_baud,
            control_mode="mit",
            mit_kp=[*map(float, kp), 8.0],
            mit_kd=[*map(float, kd), 0.3],
            gripper_control_mode="force_pos",
            gripper_torque_ratio=gripper_torque_ratio,
            pos_vel_velocity=[150.0] * 6 + [900.0],
            max_relative_target=max_relative_target_deg,
            disable_torque_on_disconnect=True,
        )
        self.joint_lower_deg = np.array(
            [config.joint_limits[name][0] for name in ARM_JOINT_NAMES],
            dtype=np.float64,
        )
        self.joint_upper_deg = np.array(
            [config.joint_limits[name][1] for name in ARM_JOINT_NAMES],
            dtype=np.float64,
        )
        self._follower = RebotB601Follower(config)
        self._managed = ManagedFollower(
            self._follower,
            policy=ShutdownPolicy(),
            report_path=shutdown_report_path,
        )
        self._dispatcher = MITCommandDispatcher(
            self._managed,
            kp=np.asarray(kp, dtype=np.float64),
            kd=np.asarray(kd, dtype=np.float64),
            torque_limit_nm=np.asarray(torque_limit_nm, dtype=np.float64),
            arm_velocity_limit_rad_s=np.full(6, 10.0, dtype=np.float64),
            gravity_scale=gravity_scale,
            gravity_ramp_s=gravity_ramp_s,
            dynamics_urdf=dynamics_urdf,
        )
        self._connected = False
        self._gripper_hold_deg: float | None = None

    @property
    def needs_cleanup(self) -> bool:
        return self._managed.needs_cleanup

    def connect(self, *, calibrate: bool) -> None:
        self._managed.connect(calibrate=calibrate)
        self._connected = True
        feedback = self.read_feedback()
        self._gripper_hold_deg = feedback.gripper_position_deg
        self.send_command(feedback.position_rad, np.zeros(6, dtype=np.float64))

    def read_feedback(self) -> FeedbackFrame:
        if not self._connected:
            raise RuntimeError("MIT tuning robot is not connected")
        self._dispatcher.get_observation()
        states = []
        for name in (*ARM_JOINT_NAMES, "gripper"):
            state = self._managed.motors[name].get_state()
            if state is None:
                raise RuntimeError(f"no MotorBridge feedback state for {name}")
            states.append(state)
        return FeedbackFrame(
            monotonic_ns=time.monotonic_ns(),
            position_rad=np.array([state.pos for state in states[:6]], dtype=np.float64),
            velocity_rad_s=np.array([state.vel for state in states[:6]], dtype=np.float64),
            torque_nm=np.array([state.torq for state in states[:6]], dtype=np.float64),
            status_code=np.array(
                [state.status_code for state in states[:6]], dtype=np.int64
            ),
            mos_temperature_c=np.array(
                [state.t_mos for state in states[:6]], dtype=np.float64
            ),
            rotor_temperature_c=np.array(
                [state.t_rotor for state in states[:6]], dtype=np.float64
            ),
            gripper_position_deg=float(np.rad2deg(states[6].pos)),
        )

    def send_command(
        self, position_rad: np.ndarray, velocity_rad_s: np.ndarray
    ) -> CommandFrame:
        if self._gripper_hold_deg is None:
            raise RuntimeError("gripper hold position has not been captured")
        position = np.asarray(position_rad, dtype=np.float64)
        velocity = np.asarray(velocity_rad_s, dtype=np.float64)
        if position.shape != (6,) or velocity.shape != (6,):
            raise ValueError("MIT tuning command must contain six positions and velocities")
        self._dispatcher.set_arm_velocity(velocity)
        action = {
            **{
                f"{name}.pos": float(np.rad2deg(position[index]))
                for index, name in enumerate(ARM_JOINT_NAMES)
            },
            "gripper.pos": self._gripper_hold_deg,
        }
        sent = self._dispatcher.send_action(action)
        sent_position = np.deg2rad(
            [float(sent[f"{name}.pos"]) for name in ARM_JOINT_NAMES]
        )
        return CommandFrame(
            requested_position_rad=position,
            sent_position_rad=sent_position,
            desired_velocity_rad_s=self._dispatcher.desired_velocity_rad_s,
            gravity_torque_nm=self._dispatcher.last_gravity_torque_nm,
            feedforward_torque_nm=self._dispatcher.last_feedforward_torque_nm,
        )

    def disconnect(self) -> None:
        if self._managed.needs_cleanup:
            self._managed.disconnect()
        self._connected = False


__all__ = ["LeRobotMITTuningRobot"]

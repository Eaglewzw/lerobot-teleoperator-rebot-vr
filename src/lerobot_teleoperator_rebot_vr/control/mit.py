"""Plugin-owned MIT arm command path with model-based gravity feedforward."""

from __future__ import annotations

import math
import time
from numbers import Real
from pathlib import Path
from typing import Any

import numpy as np

from .dynamics import B601GravityCompensator
from .types import ARM_JOINT_NAMES, GRIPPER_NAME


MIT_KP_MAX = 500.0
MIT_KD_MAX = 5.0
URDF_EFFORT_LIMIT_NM = np.array([27.0, 27.0, 27.0, 7.0, 7.0, 7.0])


def _six_vector(value: Any, name: str, *, allow_zero: bool) -> np.ndarray:
    array = np.asarray(value, dtype=np.float64)
    valid = array.shape == (6,) and np.all(np.isfinite(array))
    if allow_zero:
        valid = valid and np.all(array >= 0.0)
    else:
        valid = valid and np.all(array > 0.0)
    if not valid:
        qualifier = "non-negative" if allow_zero else "positive"
        raise ValueError(f"{name} must contain six finite {qualifier} values")
    return array.copy()


class MITCommandDispatcher:
    """Proxy a LeRobot follower while replacing only its q1-q6 send path."""

    def __init__(
        self,
        robot: Any,
        *,
        kp: np.ndarray,
        kd: np.ndarray,
        torque_limit_nm: np.ndarray,
        arm_velocity_limit_rad_s: np.ndarray,
        gravity_scale: float = 1.0,
        gravity_ramp_s: float = 1.0,
        dynamics_urdf: str | Path | None = None,
    ) -> None:
        self.robot = robot
        self.kp = _six_vector(kp, "MIT Kp", allow_zero=True)
        self.kd = _six_vector(kd, "MIT Kd", allow_zero=True)
        self.torque_limit_nm = _six_vector(
            torque_limit_nm, "MIT torque limits", allow_zero=False
        )
        self.velocity_limit_rad_s = _six_vector(
            arm_velocity_limit_rad_s,
            "MIT velocity limits",
            allow_zero=False,
        )
        if np.any(self.kp > MIT_KP_MAX):
            raise ValueError(f"MIT Kp cannot exceed {MIT_KP_MAX:g}")
        if np.any(self.kd > MIT_KD_MAX):
            raise ValueError(f"MIT Kd cannot exceed {MIT_KD_MAX:g}")
        if np.any(self.torque_limit_nm > URDF_EFFORT_LIMIT_NM):
            raise ValueError(
                "MIT torque limits cannot exceed URDF effort limits "
                f"{URDF_EFFORT_LIMIT_NM.tolist()} Nm"
            )
        if not np.isfinite(gravity_scale) or not 0.0 <= gravity_scale <= 2.0:
            raise ValueError("MIT gravity scale must be finite and in [0, 2]")
        if not np.isfinite(gravity_ramp_s) or gravity_ramp_s < 0.0:
            raise ValueError("MIT gravity ramp must be finite and non-negative")
        self.gravity_scale = float(gravity_scale)
        self.gravity_ramp_s = float(gravity_ramp_s)
        self.gravity = B601GravityCompensator(dynamics_urdf)
        self._latest_observation: dict[str, Any] = {}
        self._desired_velocity_rad_s = np.zeros(6, dtype=np.float64)
        self._first_command_s: float | None = None
        self.last_gravity_torque_nm = np.zeros(6, dtype=np.float64)
        self.last_feedforward_torque_nm = np.zeros(6, dtype=np.float64)

    def __getattr__(self, name: str) -> Any:
        return getattr(self.robot, name)

    @property
    def config(self) -> Any:
        return self.robot.config

    def get_observation(self) -> dict[str, Any]:
        observation = self.robot.get_observation()
        self.set_observation(observation)
        return observation

    def set_observation(self, observation: dict[str, Any]) -> None:
        self._latest_observation = dict(observation)

    def set_arm_velocity(self, velocity_rad_s: np.ndarray | None) -> None:
        if velocity_rad_s is None:
            self._desired_velocity_rad_s.fill(0.0)
            return
        velocity = np.asarray(velocity_rad_s, dtype=np.float64)
        if velocity.shape != (6,) or not np.all(np.isfinite(velocity)):
            self._desired_velocity_rad_s.fill(0.0)
            return
        self._desired_velocity_rad_s = np.clip(
            velocity, -self.velocity_limit_rad_s, self.velocity_limit_rad_s
        )

    def set_arm_velocity_from_position_error(
        self,
        command_deg: np.ndarray,
        actual_deg: np.ndarray,
        lookahead_s: np.ndarray,
    ) -> None:
        command = np.asarray(command_deg, dtype=np.float64)
        actual = np.asarray(actual_deg, dtype=np.float64)
        lookahead = np.asarray(lookahead_s, dtype=np.float64)
        if (
            command.shape != (6,)
            or actual.shape != (6,)
            or lookahead.shape != (6,)
            or not np.all(np.isfinite(command))
            or not np.all(np.isfinite(actual))
            or not np.all(np.isfinite(lookahead))
            or np.any(lookahead <= 0.0)
        ):
            self._desired_velocity_rad_s.fill(0.0)
            return
        # A newly accepted QP target is q_actual + dq * lookahead, so this
        # recovers dq on that cycle and naturally decays to zero if no newer
        # target arrives. That avoids retaining a stale nonzero MIT velocity.
        self.set_arm_velocity(np.deg2rad(command - actual) / lookahead)

    @property
    def desired_velocity_rad_s(self) -> np.ndarray:
        return self._desired_velocity_rad_s.copy()

    def send_action(self, action: dict[str, float]) -> dict[str, float]:
        goal_deg = {
            key.removesuffix(".pos"): float(value)
            for key, value in action.items()
            if key.endswith(".pos")
        }
        for name in (*ARM_JOINT_NAMES, GRIPPER_NAME):
            if name not in goal_deg:
                raise ValueError(f"MIT action is missing {name}.pos")
            if not np.isfinite(goal_deg[name]):
                raise ValueError(f"MIT action contains non-finite {name}.pos")

        goal_deg = self._clip_joint_limits(goal_deg)
        goal_deg = self._clip_relative_target(goal_deg)
        q_actual_rad = np.deg2rad(
            [self._feedback_deg(name) for name in ARM_JOINT_NAMES]
        )
        now_s = time.monotonic()
        if self._first_command_s is None:
            self._first_command_s = now_s
        ramp = (
            1.0
            if self.gravity_ramp_s == 0.0
            else min(1.0, (now_s - self._first_command_s) / self.gravity_ramp_s)
        )
        gravity_torque = self.gravity.gravity_torque(q_actual_rad)
        feedforward = np.clip(
            gravity_torque * self.gravity_scale * ramp,
            -self.torque_limit_nm,
            self.torque_limit_nm,
        )
        self.last_gravity_torque_nm = gravity_torque
        self.last_feedforward_torque_nm = feedforward

        for index, name in enumerate(ARM_JOINT_NAMES):
            self.robot.motors[name].send_mit(
                math.radians(goal_deg[name]),
                float(self._desired_velocity_rad_s[index]),
                float(self.kp[index]),
                float(self.kd[index]),
                float(feedforward[index]),
            )
        self._send_gripper(goal_deg[GRIPPER_NAME])
        return {f"{name}.pos": value for name, value in goal_deg.items()}

    def _feedback_deg(self, name: str) -> float:
        try:
            value = float(self._latest_observation[f"{name}.pos"])
        except (KeyError, TypeError, ValueError) as exc:
            raise RuntimeError(f"MIT command requires finite feedback for {name}") from exc
        if not np.isfinite(value):
            raise RuntimeError(f"MIT command requires finite feedback for {name}")
        return value

    def _clip_joint_limits(self, goal_deg: dict[str, float]) -> dict[str, float]:
        result = dict(goal_deg)
        for name, value in result.items():
            limits = self.config.joint_limits.get(name)
            if limits is not None:
                result[name] = float(np.clip(value, limits[0], limits[1]))
        return result

    def _clip_relative_target(self, goal_deg: dict[str, float]) -> dict[str, float]:
        limit = self.config.max_relative_target
        if limit is None:
            return goal_deg
        if isinstance(limit, Real):
            limits = {name: float(limit) for name in goal_deg}
        elif isinstance(limit, dict):
            if set(limit) != set(goal_deg):
                raise ValueError(
                    "max_relative_target keys must match MIT action motor names"
                )
            limits = {name: float(value) for name, value in limit.items()}
        else:
            raise TypeError("max_relative_target must be a scalar, dict, or None")
        result: dict[str, float] = {}
        for name, target in goal_deg.items():
            actual = self._feedback_deg(name)
            result[name] = actual + float(
                np.clip(target - actual, -limits[name], limits[name])
            )
        return result

    def _send_gripper(self, position_deg: float) -> None:
        motor = self.robot.motors[GRIPPER_NAME]
        if self.config.gripper_control_mode == "mit":
            motor.send_mit(
                math.radians(position_deg),
                0.0,
                float(self.config.gripper_mit_kp),
                float(self.config.gripper_mit_kd),
                0.0,
            )
            return
        index = self.robot.motor_names.index(GRIPPER_NAME)
        velocity_deg_s = self.config.pos_vel_velocity
        if isinstance(velocity_deg_s, list):
            velocity_deg_s = velocity_deg_s[index]
        motor.send_force_pos(
            math.radians(position_deg),
            math.radians(float(velocity_deg_s)),
            float(self.config.gripper_torque_ratio),
        )

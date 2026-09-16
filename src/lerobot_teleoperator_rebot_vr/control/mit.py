"""Plugin-owned MIT arm command path with model-based gravity feedforward."""

from __future__ import annotations

import math
import time
from numbers import Real
from pathlib import Path
from typing import Any

import numpy as np

from ..constants import ARM_EFFORT_LIMIT_NM
from .dynamics import B601GravityCompensator
from .joint_command import braking_velocity_bounds
from .types import ARM_JOINT_NAMES, GRIPPER_NAME


MIT_KP_MAX = 500.0
MIT_KD_MAX = 5.0


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
        arm_acceleration_limit_rad_s2: np.ndarray | None = None,
        position_lookahead_s: np.ndarray | None = None,
        velocity_aligned_axes: np.ndarray | None = None,
        joint_limit_margin_rad: float = 0.0,
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
        self.acceleration_limit_rad_s2 = (
            None
            if arm_acceleration_limit_rad_s2 is None
            else _six_vector(
                arm_acceleration_limit_rad_s2,
                "MIT acceleration limits",
                allow_zero=False,
            )
        )
        self.position_lookahead_s = (
            None
            if position_lookahead_s is None
            else _six_vector(
                position_lookahead_s,
                "MIT position lookahead",
                allow_zero=False,
            )
        )
        self.velocity_aligned_axes = (
            np.ones(6, dtype=bool)
            if velocity_aligned_axes is None
            else np.asarray(velocity_aligned_axes, dtype=bool)
        )
        if self.velocity_aligned_axes.shape != (6,):
            raise ValueError("MIT velocity-aligned axes must contain six values")
        if np.any(self.kp > MIT_KP_MAX):
            raise ValueError(f"MIT Kp cannot exceed {MIT_KP_MAX:g}")
        if np.any(self.kd > MIT_KD_MAX):
            raise ValueError(f"MIT Kd cannot exceed {MIT_KD_MAX:g}")
        if np.any(self.torque_limit_nm > ARM_EFFORT_LIMIT_NM):
            raise ValueError(
                "MIT torque limits cannot exceed URDF effort limits "
                f"{list(ARM_EFFORT_LIMIT_NM)} Nm"
            )
        if not np.isfinite(gravity_scale) or not 0.0 <= gravity_scale <= 2.0:
            raise ValueError("MIT gravity scale must be finite and in [0, 2]")
        if not np.isfinite(gravity_ramp_s) or gravity_ramp_s < 0.0:
            raise ValueError("MIT gravity ramp must be finite and non-negative")
        if not np.isfinite(joint_limit_margin_rad) or joint_limit_margin_rad < 0.0:
            raise ValueError("MIT joint limit margin must be finite and non-negative")
        joint_limits_deg = np.asarray(
            [self.config.joint_limits[name] for name in ARM_JOINT_NAMES],
            dtype=np.float64,
        )
        if (
            joint_limits_deg.shape != (6, 2)
            or not np.all(np.isfinite(joint_limits_deg))
        ):
            raise ValueError("MIT joint limits must contain six finite pairs")
        self.joint_lower_limit_rad = np.deg2rad(joint_limits_deg[:, 0])
        self.joint_upper_limit_rad = np.deg2rad(joint_limits_deg[:, 1])
        margin = float(joint_limit_margin_rad)
        self.joint_safe_lower_rad = self.joint_lower_limit_rad + margin
        self.joint_safe_upper_rad = self.joint_upper_limit_rad - margin
        if np.any(self.joint_safe_lower_rad >= self.joint_safe_upper_rad):
            raise ValueError("MIT joint limit margin leaves no usable range")
        self.gravity_scale = float(gravity_scale)
        self.gravity_ramp_s = float(gravity_ramp_s)
        self.gravity = B601GravityCompensator(dynamics_urdf)
        self._latest_observation: dict[str, Any] = {}
        self._target_velocity_rad_s = np.zeros(6, dtype=np.float64)
        self._desired_velocity_rad_s = np.zeros(6, dtype=np.float64)
        self._velocity_target_updated_s: float | None = None
        self._last_velocity_step_s: float | None = None
        self._velocity_aligned_position = False
        self._trajectory_velocity_active = False
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
        self._trajectory_velocity_active = False
        if velocity_rad_s is None:
            self.stop_arm_velocity(immediate=True)
            return
        velocity = np.asarray(velocity_rad_s, dtype=np.float64)
        if velocity.shape != (6,) or not np.all(np.isfinite(velocity)):
            self.stop_arm_velocity(immediate=True)
            return
        self._target_velocity_rad_s = np.clip(
            velocity, -self.velocity_limit_rad_s, self.velocity_limit_rad_s
        )
        self._velocity_target_updated_s = time.monotonic()
        self._velocity_aligned_position = self.position_lookahead_s is not None
        if self.acceleration_limit_rad_s2 is None:
            self._desired_velocity_rad_s = self._target_velocity_rad_s.copy()

    def set_arm_trajectory_velocity(self, velocity_rad_s: np.ndarray) -> None:
        """Feed forward a bounded trajectory's velocity without replacing its position.

        Keep the final velocity/acceleration limiter. A stopped trajectory axis
        clears residual velocity immediately, as at a position-limit clamp.
        """
        self.set_arm_velocity(velocity_rad_s)
        self._velocity_aligned_position = False
        self._trajectory_velocity_active = True
        self._desired_velocity_rad_s[self._target_velocity_rad_s == 0.0] = 0.0

    def stop_arm_velocity(self, *, immediate: bool) -> None:
        """Request zero velocity, optionally bypassing the deceleration ramp."""
        self._target_velocity_rad_s.fill(0.0)
        self._velocity_target_updated_s = None
        if immediate:
            self._desired_velocity_rad_s.fill(0.0)
            self._last_velocity_step_s = None
            self._velocity_aligned_position = False
            self._trajectory_velocity_active = False

    def stop_stale_arm_velocity(self, max_age_s: float) -> bool:
        """Ramp toward zero after the last accepted QP velocity becomes stale."""
        if not np.isfinite(max_age_s) or max_age_s <= 0.0:
            raise ValueError("MIT velocity target max age must be finite and positive")
        updated_s = self._velocity_target_updated_s
        if updated_s is None or time.monotonic() - updated_s >= max_age_s:
            self.stop_arm_velocity(immediate=False)
            return True
        return False

    @property
    def target_velocity_rad_s(self) -> np.ndarray:
        return self._target_velocity_rad_s.copy()

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

        q_actual_deg = np.array(
            [self._feedback_deg(name) for name in ARM_JOINT_NAMES],
            dtype=np.float64,
        )
        q_actual_rad = np.deg2rad(q_actual_deg)
        now_s = time.monotonic()
        self._advance_arm_velocity(now_s)
        if self._velocity_aligned_position:
            assert self.position_lookahead_s is not None
            safe_lower = np.where(
                q_actual_rad <= self.joint_safe_lower_rad,
                q_actual_rad,
                self.joint_safe_lower_rad,
            )
            safe_upper = np.where(
                q_actual_rad >= self.joint_safe_upper_rad,
                q_actual_rad,
                self.joint_safe_upper_rad,
            )
            if self.acceleration_limit_rad_s2 is not None:
                braking_lo, braking_hi = braking_velocity_bounds(
                    q_actual_rad,
                    safe_lower,
                    safe_upper,
                    self.acceleration_limit_rad_s2,
                )
                self._desired_velocity_rad_s = np.clip(
                    self._desired_velocity_rad_s, braking_lo, braking_hi
                )
            aligned_position_rad = np.clip(
                q_actual_rad
                + self._desired_velocity_rad_s * self.position_lookahead_s,
                safe_lower,
                safe_upper,
            )
            aligned_position_deg = np.rad2deg(aligned_position_rad)
            for index, name in enumerate(ARM_JOINT_NAMES):
                if self.velocity_aligned_axes[index]:
                    goal_deg[name] = float(aligned_position_deg[index])
        goal_deg = self._clip_joint_limits(goal_deg)
        goal_deg = self._clip_relative_target(goal_deg)
        if self._trajectory_velocity_active:
            # Explicit startup/zero trajectories may reach hard-limit zero, so
            # use hard limits here rather than ACTIVE's interior safety margin.
            if self.acceleration_limit_rad_s2 is not None:
                lo, hi = braking_velocity_bounds(
                    q_actual_rad, self.joint_lower_limit_rad,
                    self.joint_upper_limit_rad, self.acceleration_limit_rad_s2,
                )
                self._desired_velocity_rad_s = np.clip(self._desired_velocity_rad_s, lo, hi)
            position_error = np.deg2rad(
                [goal_deg[name] for name in ARM_JOINT_NAMES]
            ) - q_actual_rad
            self._desired_velocity_rad_s[
                self._desired_velocity_rad_s * position_error <= 0.0
            ] = 0.0
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

    def _advance_arm_velocity(self, now_s: float) -> None:
        if self.acceleration_limit_rad_s2 is None:
            self._desired_velocity_rad_s = self._target_velocity_rad_s.copy()
            self._last_velocity_step_s = now_s
            return
        previous_s = self._last_velocity_step_s
        self._last_velocity_step_s = now_s
        if previous_s is None:
            return
        dt_s = float(np.clip(now_s - previous_s, 0.0, 0.05))
        max_change = self.acceleration_limit_rad_s2 * dt_s
        velocity_change = np.clip(
            self._target_velocity_rad_s - self._desired_velocity_rad_s,
            -max_change,
            max_change,
        )
        self._desired_velocity_rad_s = np.clip(
            self._desired_velocity_rad_s + velocity_change,
            -self.velocity_limit_rad_s,
            self.velocity_limit_rad_s,
        )

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

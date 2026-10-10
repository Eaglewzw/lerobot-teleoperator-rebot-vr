"""Plugin-owned MIT arm command path with model-based gravity feedforward."""

from __future__ import annotations

import math
import time
from numbers import Real
from pathlib import Path
from typing import Any

import numpy as np

from ..hardware.profiles import arm_profile
from .dynamics import B601GravityCompensator
from .joint_command import braking_velocity_bounds
from ..diagnostics.velocity import vector_fields
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
        q1_reference_error_deg: float = 0.0,
        arm_reference_error_deg: float = 0.0,
        joint_limit_margin_rad: float = 0.0,
        gravity_scale: float = 1.0,
        gravity_ramp_s: float = 1.0,
        dynamics_urdf: str | Path | None = None,
        robot_model: str = "b601_dm",
        gripper_velocity_limit_deg_s: float = 0.0,
    ) -> None:
        self.robot = robot
        if not np.isfinite(gripper_velocity_limit_deg_s) or gripper_velocity_limit_deg_s < 0:
            raise ValueError("MIT gripper velocity limit must be finite and non-negative")
        # Opt in for RS only. DM retains its existing zero-velocity PD path.
        self.gripper_velocity_limit_deg_s = (
            float(gripper_velocity_limit_deg_s) if robot_model == "b601_rs" else 0.0)
        self.last_gripper_velocity_deg_s = 0.0
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
        if not np.isfinite(q1_reference_error_deg) or not 0 <= q1_reference_error_deg <= 2:
            raise ValueError("q1 reference error must be finite and within [0, 2] degrees")
        if q1_reference_error_deg and (self.position_lookahead_s is None or
                self.acceleration_limit_rad_s2 is None or not self.velocity_aligned_axes[0]):
            raise ValueError("q1 reference requires velocity alignment and acceleration limits")
        if not np.isfinite(arm_reference_error_deg) or not 0 <= arm_reference_error_deg <= 3:
            raise ValueError("arm reference error must be finite and within [0, 3] degrees")
        if arm_reference_error_deg and q1_reference_error_deg:
            raise ValueError("arm reference and q1 reference are mutually exclusive")
        if arm_reference_error_deg and (self.position_lookahead_s is None
                or self.acceleration_limit_rad_s2 is None
                or not np.all(self.velocity_aligned_axes[:3])):
            raise ValueError(
                "arm reference requires velocity alignment and acceleration limits"
            )
        self.arm_error_limit_rad = math.radians(arm_reference_error_deg)
        self.arm_reference_rad: np.ndarray | None = None
        self.q1_error_limit_rad = math.radians(q1_reference_error_deg)
        self.q1_reference: float | None = None
        if np.any(self.kp > MIT_KP_MAX):
            raise ValueError(f"MIT Kp cannot exceed {MIT_KP_MAX:g}")
        if np.any(self.kd > MIT_KD_MAX):
            raise ValueError(f"MIT Kd cannot exceed {MIT_KD_MAX:g}")
        profile = arm_profile(robot_model)
        effort_limit = profile.effort_nm
        if np.any(self.torque_limit_nm > effort_limit):
            raise ValueError(
                "MIT torque limits cannot exceed URDF effort limits "
                f"{list(effort_limit)} Nm"
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
        from .zero_trim import ZeroPoseTrim
        self.zero_trim = ZeroPoseTrim(self.torque_limit_nm) if robot_model == "b601_rs" else None
        self.gravity_ramp_s = float(gravity_ramp_s)
        self.gravity = B601GravityCompensator(
            profile.model_path(dynamics=True) if dynamics_urdf is None else dynamics_urdf
        )
        self._latest_observation: dict[str, Any] = {}
        self._target_velocity_rad_s = np.zeros(6, dtype=np.float64)
        self._input_velocity_rad_s = np.zeros(6, dtype=np.float64)
        self._speed_limited = np.zeros(6, dtype=bool)
        self._acceleration_limited = np.zeros(6, dtype=bool)
        self._braking_limited = np.zeros(6, dtype=bool)
        self._velocity_step_dt_s = None
        self._desired_velocity_rad_s = np.zeros(6, dtype=np.float64)
        self._velocity_target_updated_s: float | None = None
        self._last_velocity_step_s: float | None = None
        self._velocity_aligned_position = False
        self._first_command_s: float | None = None
        self.last_gravity_torque_nm = np.zeros(6, dtype=np.float64)
        self.last_feedforward_torque_nm = np.zeros(6, dtype=np.float64)
        self._last_sent_goal_deg: dict[str, float] | None = None

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

    def set_zero_return(self, active: bool) -> None:
        if self.zero_trim is not None:
            self.zero_trim.active = active

    def set_arm_velocity(self, velocity_rad_s: np.ndarray | None) -> None:
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
        self._input_velocity_rad_s = velocity.copy()
        self._speed_limited = velocity != self._target_velocity_rad_s
        self._velocity_target_updated_s = time.monotonic()
        self._velocity_aligned_position = self.position_lookahead_s is not None
        if self.acceleration_limit_rad_s2 is None:
            self._desired_velocity_rad_s = self._target_velocity_rad_s.copy()

    def stop_arm_velocity(self, *, immediate: bool) -> None:
        """Request zero velocity, optionally bypassing the deceleration ramp."""
        self.q1_reference = None
        self.arm_reference_rad = None
        self._target_velocity_rad_s.fill(0.0)
        self._input_velocity_rad_s.fill(0.0)
        self._speed_limited.fill(False)
        self._velocity_target_updated_s = None
        if immediate:
            self._acceleration_limited.fill(False)
            self._braking_limited.fill(False)
            self._desired_velocity_rad_s.fill(0.0)
            self._last_velocity_step_s = None
            self._velocity_aligned_position = False

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

    @property
    def velocity_diagnostics(self) -> dict[str, object]:
        return {
            **vector_fields("mit_input_velocity", self._input_velocity_rad_s, "rad_s"),
            **vector_fields("mit_target_velocity", self._target_velocity_rad_s, "rad_s"),
            **vector_fields("mit_speed_limited", self._speed_limited, "flag"),
            **vector_fields("mit_acceleration_limited", self._acceleration_limited, "flag"),
            **vector_fields("mit_braking_limited", self._braking_limited, "flag"),
            "mit_velocity_step_dt_s": self._velocity_step_dt_s,
        }

    def send_action(self, action: dict[str, float], *,
                    gripper_velocity_deg_s: float = 0.0) -> dict[str, float]:
        # Velocity belongs to this action only: startup, return and HOLD cannot
        # accidentally reuse the previous teleoperation velocity.
        if not np.isfinite(gripper_velocity_deg_s):
            raise ValueError("MIT gripper velocity must be finite")
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
        self._braking_limited.fill(False)
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
                before_braking = self._desired_velocity_rad_s
                self._desired_velocity_rad_s = np.clip(
                    self._desired_velocity_rad_s, braking_lo, braking_hi
                )
                self._braking_limited = before_braking != self._desired_velocity_rad_s
            aligned_position_rad = np.clip(
                q_actual_rad
                + self._desired_velocity_rad_s * self.position_lookahead_s,
                safe_lower,
                safe_upper,
            )
            if self.arm_error_limit_rad > 0.0:
                # Bounded absolute position reference for q1-q3: integrate the
                # already-limited desired velocity so a stalled joint keeps
                # building position error (and therefore drive torque) instead
                # of letting the feedback-anchored target chase the stall.
                # The lookahead lead is preserved on top of the reference, so
                # well-tracked high-speed motion keeps the original drive.
                dt = self._velocity_step_dt_s or 0.0
                reference = (
                    q_actual_rad[:3].copy()
                    if self.arm_reference_rad is None
                    else self.arm_reference_rad
                )
                reference = np.clip(
                    reference + self._desired_velocity_rad_s[:3] * dt,
                    q_actual_rad[:3] - self.arm_error_limit_rad,
                    q_actual_rad[:3] + self.arm_error_limit_rad,
                )
                self.arm_reference_rad = reference
                aligned_position_rad[:3] = np.clip(
                    reference
                    + self._desired_velocity_rad_s[:3] * self.position_lookahead_s[:3],
                    safe_lower[:3],
                    safe_upper[:3],
                )
            aligned_position_deg = np.rad2deg(aligned_position_rad)
            for index, name in enumerate(ARM_JOINT_NAMES):
                if self.velocity_aligned_axes[index]:
                    goal_deg[name] = float(aligned_position_deg[index])
        requested_gripper_deg = goal_deg[GRIPPER_NAME]
        goal_deg = self._clip_joint_limits(goal_deg)
        goal_deg = self._clip_relative_target(goal_deg)
        gripper_velocity_deg_s = float(np.clip(gripper_velocity_deg_s,
            -self.gripper_velocity_limit_deg_s, self.gripper_velocity_limit_deg_s))
        if not math.isclose(goal_deg[GRIPPER_NAME], requested_gripper_deg, abs_tol=1e-8):
            # Hardware-level clipping overrides the already shaped trajectory.
            gripper_velocity_deg_s = 0.0
        if self.arm_reference_rad is not None:
            # Re-anchor the reference to what was actually sent so a binding
            # limit clip cannot wind the integrator up behind the boundary.
            sent3 = np.deg2rad(
                [goal_deg[name] for name in ARM_JOINT_NAMES[:3]]
            )
            self.arm_reference_rad = np.clip(
                sent3
                - self._desired_velocity_rad_s[:3] * self.position_lookahead_s[:3],
                q_actual_rad[:3] - self.arm_error_limit_rad,
                q_actual_rad[:3] + self.arm_error_limit_rad,
            )
        if self._first_command_s is None:
            self._first_command_s = now_s
        ramp = (
            1.0
            if self.gravity_ramp_s == 0.0
            else min(1.0, (now_s - self._first_command_s) / self.gravity_ramp_s)
        )
        gravity_torque = self.gravity.gravity_torque(q_actual_rad)
        zero_trim = (np.zeros(6) if self.zero_trim is None else self.zero_trim.update(
            q_actual_rad, np.deg2rad([goal_deg[name] for name in ARM_JOINT_NAMES]), now_s))
        feedforward = np.clip(
            gravity_torque * self.gravity_scale * ramp + zero_trim,
            -self.torque_limit_nm,
            self.torque_limit_nm,
        )
        self.last_gravity_torque_nm = gravity_torque
        self.last_feedforward_torque_nm = feedforward

        return self._send_goal(goal_deg, self._desired_velocity_rad_s, feedforward,
                               gripper_velocity_deg_s=gripper_velocity_deg_s)

    def send_feedback_hold(self) -> dict[str, float]:
        """Repeat the last bounded command at zero velocity without fresh feedback.

        Freeze the last gravity feedforward rather than fabricating measured
        positions or extrapolating motion through a feedback outage.
        """
        self.stop_arm_velocity(immediate=True)
        if self._last_sent_goal_deg is None:
            raise RuntimeError("MIT HOLD requires a previously sent valid command")
        return self._send_goal(self._last_sent_goal_deg, np.zeros(6),
                               self.last_feedforward_torque_nm)

    def _send_goal(self, goal_deg, velocity, feedforward, *, gripper_velocity_deg_s=0.0):
        if self.q1_error_limit_rad > 0 and self._velocity_aligned_position:
            actual = math.radians(self._feedback_deg(ARM_JOINT_NAMES[0]))
            dt = self._velocity_step_dt_s or 0.0
            if self.q1_reference is None:
                self.q1_reference = actual
            self.q1_reference = float(np.clip(
                self.q1_reference + velocity[0] * dt,
                actual - self.q1_error_limit_rad, actual + self.q1_error_limit_rad,
            ))
            proposed = dict(goal_deg)
            proposed[ARM_JOINT_NAMES[0]] = math.degrees(self.q1_reference)
            clipped = self._clip_relative_target(self._clip_joint_limits(proposed))
            goal_deg[ARM_JOINT_NAMES[0]] = clipped[ARM_JOINT_NAMES[0]]
            self.q1_reference = math.radians(goal_deg[ARM_JOINT_NAMES[0]])
        else:
            self.q1_reference = None
        for index, name in enumerate(ARM_JOINT_NAMES):
            self.robot.motors[name].send_mit(
                math.radians(goal_deg[name]),
                float(velocity[index]),
                float(self.kp[index]),
                float(self.kd[index]),
                float(feedforward[index]),
            )
        self._send_gripper(goal_deg[GRIPPER_NAME], gripper_velocity_deg_s)
        self._last_sent_goal_deg = dict(goal_deg)
        return {f"{name}.pos": value for name, value in goal_deg.items()}

    def _advance_arm_velocity(self, now_s: float) -> None:
        self._acceleration_limited.fill(False)
        self._velocity_step_dt_s = None
        if self.acceleration_limit_rad_s2 is None:
            self._desired_velocity_rad_s = self._target_velocity_rad_s.copy()
            self._last_velocity_step_s = now_s
            return
        previous_s = self._last_velocity_step_s
        self._last_velocity_step_s = now_s
        if previous_s is None:
            return
        dt_s = float(np.clip(now_s - previous_s, 0.0, 0.05))
        self._velocity_step_dt_s = dt_s
        max_change = self.acceleration_limit_rad_s2 * dt_s
        velocity_change = np.clip(
            self._target_velocity_rad_s - self._desired_velocity_rad_s,
            -max_change,
            max_change,
        )
        self._acceleration_limited = (
            self._target_velocity_rad_s - self._desired_velocity_rad_s != velocity_change
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

    def _send_gripper(self, position_deg: float, velocity_deg_s: float = 0.0) -> None:
        motor = self.robot.motors[GRIPPER_NAME]
        if self.config.gripper_control_mode == "mit":
            motor.send_mit(
                math.radians(position_deg),
                math.radians(velocity_deg_s),
                float(self.config.gripper_mit_kp),
                float(self.config.gripper_mit_kd),
                0.0,
            )
            self.last_gripper_velocity_deg_s = velocity_deg_s
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

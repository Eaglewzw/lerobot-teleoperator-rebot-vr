"""Hardware-agnostic MIT tuning session runner."""

from __future__ import annotations

import time
from collections.abc import Callable

import numpy as np

from .analysis import TuningAnalyzer
from .logger import TuningCSVLogger
from .models import ARM_JOINT_NAMES, MITTuningConfig, TuningRobot, TuningSample
from .profile import ExcursionProfile, PoseTransition


class MITTuningSession:
    def __init__(
        self,
        robot: TuningRobot,
        *,
        config: MITTuningConfig,
        kp: np.ndarray,
        kd: np.ndarray,
        csv_logger: TuningCSVLogger,
        analyzer: TuningAnalyzer,
        should_stop: Callable[[], bool] = lambda: False,
    ) -> None:
        self.robot = robot
        self.config = config
        self.kp = np.asarray(kp, dtype=np.float64).copy()
        self.kd = np.asarray(kd, dtype=np.float64).copy()
        if self.kp.shape != (6,) or self.kd.shape != (6,):
            raise ValueError("kp and kd must contain six values")
        self.csv_logger = csv_logger
        self.analyzer = analyzer
        self.should_stop = should_stop
        self.profile = ExcursionProfile(
            step_rad=np.deg2rad(config.step_deg),
            cycles=config.cycles,
            transition_s=config.transition_s,
            hold_s=config.hold_s,
            warmup_s=config.warmup_s,
        )

    def run(self) -> bool:
        center = self.robot.read_feedback().position_rad.copy()
        if self.config.prepare_pose_deg is not None:
            prepared = self._move_to_prepare_pose(center)
            if prepared is None:
                return False
            center = prepared
        self._validate_excursion(center)
        started_s = time.monotonic()
        previous_loop_s = started_s
        next_status_s = started_s
        tracking_faults = 0
        last_command = center.copy()

        while not self.should_stop():
            loop_started_s = time.monotonic()
            elapsed_s = loop_started_s - started_s
            profile = self.profile.sample(elapsed_s)
            target = center.copy()
            target[self.config.joint_index] += profile.offset_rad
            desired_velocity = np.zeros(6, dtype=np.float64)
            desired_velocity[self.config.joint_index] = profile.velocity_rad_s

            feedback_started_s = time.monotonic()
            try:
                feedback = self.robot.read_feedback()
            except Exception:
                tracking_faults += 1
                try:
                    self.robot.send_command(
                        last_command, np.zeros(6, dtype=np.float64)
                    )
                except Exception:
                    pass
                if tracking_faults >= self.config.fault_consecutive:
                    raise RuntimeError(
                        "MIT tuning aborted after consecutive feedback failures"
                    )
                self._sleep_until(loop_started_s)
                continue
            feedback_read_ms = (time.monotonic() - feedback_started_s) * 1e3
            reason = self._safety_reason(feedback, target, center)
            if reason:
                tracking_faults += 1
                if tracking_faults >= self.config.fault_consecutive:
                    raise RuntimeError(f"MIT tuning safety abort: {reason}")
                target = feedback.position_rad.copy()
                desired_velocity.fill(0.0)
            else:
                tracking_faults = 0

            command_started_s = time.monotonic()
            command = self.robot.send_command(target, desired_velocity)
            command_send_ms = (time.monotonic() - command_started_s) * 1e3
            last_command = command.sent_position_rad.copy()
            dt_s = loop_started_s - previous_loop_s
            previous_loop_s = loop_started_s
            sample = TuningSample(
                wall_time_ns=time.time_ns(),
                elapsed_s=elapsed_s,
                loop_hz=None if dt_s <= 0.0 else 1.0 / dt_s,
                phase=profile.phase,
                phase_progress=profile.phase_progress,
                cycle=profile.cycle,
                joint_index=self.config.joint_index,
                kp=self.kp,
                kd=self.kd,
                feedback=feedback,
                command=command,
                feedback_read_ms=feedback_read_ms,
                command_send_ms=command_send_ms,
            )
            self.csv_logger.write_sample(sample)
            self.analyzer.add(sample)

            if loop_started_s >= next_status_s:
                index = self.config.joint_index
                print(
                    f"{profile.phase} {ARM_JOINT_NAMES[index]} "
                    f"actual={np.rad2deg(feedback.position_rad[index]):.2f}deg "
                    f"target={np.rad2deg(last_command[index]):.2f}deg "
                    f"vel={feedback.velocity_rad_s[index]:.3f}rad/s",
                    flush=True,
                )
                next_status_s = loop_started_s + 0.2
            if profile.done:
                return True
            self._sleep_until(loop_started_s)
        return False

    def _move_to_prepare_pose(self, start_rad: np.ndarray) -> np.ndarray | None:
        target_rad = np.deg2rad(self.config.prepare_pose_deg)
        self._validate_prepare_pose(target_rad)
        max_delta = float(np.max(np.abs(target_rad - start_rad)))
        duration_s = max(
            0.5,
            1.875 * max_delta / self.config.prepare_speed_rad_s,
        )
        transition = PoseTransition(start_rad, target_rad, duration_s)
        started_s = time.monotonic()
        next_status_s = started_s
        faults = 0
        last_command = start_rad.copy()
        print(
            "Preparing test pose over "
            f"{duration_s:.2f}s: {np.array2string(self.config.prepare_pose_deg, precision=1)}deg",
            flush=True,
        )
        while not self.should_stop():
            loop_started_s = time.monotonic()
            elapsed_s = loop_started_s - started_s
            sample = transition.sample(elapsed_s)
            try:
                feedback = self.robot.read_feedback()
            except Exception:
                faults += 1
                try:
                    self.robot.send_command(last_command, np.zeros(6, dtype=np.float64))
                except Exception:
                    pass
                if faults >= self.config.fault_consecutive:
                    raise RuntimeError(
                        "MIT preparation aborted after consecutive feedback failures"
                    )
                self._sleep_until(loop_started_s)
                continue
            reason = self._prepare_safety_reason(feedback, sample.position_rad)
            if reason:
                faults += 1
                if faults >= self.config.fault_consecutive:
                    raise RuntimeError(f"MIT preparation safety abort: {reason}")
                command = self.robot.send_command(
                    feedback.position_rad, np.zeros(6, dtype=np.float64)
                )
            else:
                faults = 0
                command = self.robot.send_command(
                    sample.position_rad, sample.velocity_rad_s
                )
            last_command = command.sent_position_rad.copy()
            if loop_started_s >= next_status_s:
                print(
                    "prepare actual="
                    f"{np.array2string(np.rad2deg(feedback.position_rad), precision=1)} "
                    "target="
                    f"{np.array2string(np.rad2deg(last_command), precision=1)}",
                    flush=True,
                )
                next_status_s = loop_started_s + 0.5
            if elapsed_s >= duration_s + self.config.prepare_settle_s:
                error_deg = np.abs(np.rad2deg(target_rad - feedback.position_rad))
                if np.max(error_deg) > self.config.prepare_tolerance_deg:
                    joint = int(np.argmax(error_deg))
                    raise RuntimeError(
                        "MIT preparation did not reach its target: "
                        f"{ARM_JOINT_NAMES[joint]} error={error_deg[joint]:.2f}deg"
                    )
                print(
                    "Preparation reached: actual="
                    f"{np.array2string(np.rad2deg(feedback.position_rad), precision=1)}deg",
                    flush=True,
                )
                return target_rad.copy()
            self._sleep_until(loop_started_s)
        return None

    def _validate_prepare_pose(self, target_rad: np.ndarray) -> None:
        target_deg = np.rad2deg(target_rad)
        lower = np.asarray(self.robot.joint_lower_deg, dtype=np.float64) + 2.0
        upper = np.asarray(self.robot.joint_upper_deg, dtype=np.float64) - 2.0
        if np.any(target_deg < lower) or np.any(target_deg > upper):
            raise RuntimeError(
                "preparation pose exceeds the tuning range: "
                f"target={target_deg.tolist()}, lower={lower.tolist()}, upper={upper.tolist()}"
            )

    def _prepare_safety_reason(self, feedback, target_rad: np.ndarray) -> str:
        common = self._common_safety_reason(feedback)
        if common:
            return common
        error_deg = np.abs(np.rad2deg(target_rad - feedback.position_rad))
        if np.max(error_deg) > self.config.max_tracking_error_deg:
            joint = int(np.argmax(error_deg))
            return (
                f"{ARM_JOINT_NAMES[joint]} tracking error reached "
                f"{error_deg[joint]:.1f}deg"
            )
        return ""

    def _validate_excursion(self, center_rad: np.ndarray) -> None:
        index = self.config.joint_index
        lower = float(self.robot.joint_lower_deg[index]) + 2.0
        upper = float(self.robot.joint_upper_deg[index]) - 2.0
        endpoints = np.rad2deg(center_rad[index]) + np.array(
            [-self.config.step_deg, self.config.step_deg]
        )
        if np.any(endpoints < lower) or np.any(endpoints > upper):
            raise RuntimeError(
                f"{ARM_JOINT_NAMES[index]} excursion {endpoints.tolist()}deg exceeds "
                f"the tuning range {lower:.1f}..{upper:.1f}deg"
            )

    def _safety_reason(
        self, feedback, target_rad: np.ndarray, center_rad: np.ndarray
    ) -> str:
        common = self._common_safety_reason(feedback)
        if common:
            return common
        error_deg = np.abs(np.rad2deg(target_rad - feedback.position_rad))
        index = self.config.joint_index
        if error_deg[index] > self.config.max_tracking_error_deg:
            return f"selected-joint error reached {error_deg[index]:.1f}deg"
        hold_error = np.abs(np.rad2deg(center_rad - feedback.position_rad))
        hold_error[index] = 0.0
        if np.max(hold_error) > self.config.max_hold_error_deg:
            joint = int(np.argmax(hold_error))
            return (
                f"held joint {ARM_JOINT_NAMES[joint]} moved "
                f"{hold_error[joint]:.1f}deg"
            )
        return ""

    def _common_safety_reason(self, feedback) -> str:
        if np.any(feedback.status_code != 1):
            return f"motor status is not ENABLED: {feedback.status_code.tolist()}"
        temperatures = np.maximum(
            feedback.mos_temperature_c, feedback.rotor_temperature_c
        )
        if np.max(temperatures) > self.config.max_temperature_c:
            return f"motor temperature reached {np.max(temperatures):.1f}C"
        if np.max(np.abs(feedback.velocity_rad_s)) > self.config.max_velocity_rad_s:
            return (
                "joint velocity exceeded "
                f"{self.config.max_velocity_rad_s:.2f}rad/s"
            )
        return ""

    def _sleep_until(self, loop_started_s: float) -> None:
        remaining = 1.0 / self.config.fps - (time.monotonic() - loop_started_s)
        if remaining > 0.0:
            time.sleep(remaining)


__all__ = ["MITTuningSession"]

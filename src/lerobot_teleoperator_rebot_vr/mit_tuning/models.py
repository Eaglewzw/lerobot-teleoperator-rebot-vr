"""Data contracts shared by the MIT tuning core and hardware adapter."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

import numpy as np
import numpy.typing as npt

from ..constants import ARM_JOINT_NAMES


JOINT_ALIASES = {
    **{f"q{index + 1}": index for index in range(6)},
    **{name: index for index, name in enumerate(ARM_JOINT_NAMES)},
}


def _vector(value: object, name: str, *, length: int = 6) -> npt.NDArray[np.float64]:
    result = np.asarray(value, dtype=np.float64)
    if result.shape != (length,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain {length} finite values")
    return result.copy()


@dataclass(frozen=True)
class FeedbackFrame:
    monotonic_ns: int
    position_rad: npt.NDArray[np.float64]
    velocity_rad_s: npt.NDArray[np.float64]
    torque_nm: npt.NDArray[np.float64]
    status_code: npt.NDArray[np.int64]
    mos_temperature_c: npt.NDArray[np.float64]
    rotor_temperature_c: npt.NDArray[np.float64]
    gripper_position_deg: float

    def __post_init__(self) -> None:
        for name in (
            "position_rad",
            "velocity_rad_s",
            "torque_nm",
            "mos_temperature_c",
            "rotor_temperature_c",
        ):
            object.__setattr__(self, name, _vector(getattr(self, name), name))
        status = np.asarray(self.status_code, dtype=np.int64)
        if status.shape != (6,):
            raise ValueError("status_code must contain six values")
        object.__setattr__(self, "status_code", status.copy())
        if not np.isfinite(self.gripper_position_deg):
            raise ValueError("gripper_position_deg must be finite")


@dataclass(frozen=True)
class CommandFrame:
    requested_position_rad: npt.NDArray[np.float64]
    sent_position_rad: npt.NDArray[np.float64]
    desired_velocity_rad_s: npt.NDArray[np.float64]
    gravity_torque_nm: npt.NDArray[np.float64]
    feedforward_torque_nm: npt.NDArray[np.float64]

    def __post_init__(self) -> None:
        for name in (
            "requested_position_rad",
            "sent_position_rad",
            "desired_velocity_rad_s",
            "gravity_torque_nm",
            "feedforward_torque_nm",
        ):
            object.__setattr__(self, name, _vector(getattr(self, name), name))


@dataclass(frozen=True)
class TuningSample:
    wall_time_ns: int
    elapsed_s: float
    loop_hz: float | None
    phase: str
    phase_progress: float
    cycle: int
    joint_index: int
    kp: npt.NDArray[np.float64]
    kd: npt.NDArray[np.float64]
    feedback: FeedbackFrame
    command: CommandFrame
    feedback_read_ms: float
    command_send_ms: float

    def __post_init__(self) -> None:
        object.__setattr__(self, "kp", _vector(self.kp, "kp"))
        object.__setattr__(self, "kd", _vector(self.kd, "kd"))


@dataclass(frozen=True)
class MITTuningConfig:
    joint_index: int
    step_deg: float = 5.0
    cycles: int = 2
    transition_s: float = 0.5
    hold_s: float = 2.5
    warmup_s: float = 1.0
    fps: float = 90.0
    max_tracking_error_deg: float = 20.0
    max_hold_error_deg: float = 8.0
    max_velocity_rad_s: float = 3.0
    max_temperature_c: float = 65.0
    fault_consecutive: int = 3
    settling_band_deg: float = 0.5
    settling_window_s: float = 0.25
    prepare_pose_deg: npt.NDArray[np.float64] | None = None
    prepare_speed_rad_s: float = 0.3
    prepare_settle_s: float = 2.0
    prepare_tolerance_deg: float = 3.0

    def __post_init__(self) -> None:
        if not 0 <= self.joint_index < 6:
            raise ValueError("joint_index must be in [0, 5]")
        finite_positive = (
            self.step_deg,
            self.transition_s,
            self.hold_s,
            self.warmup_s,
            self.fps,
            self.max_tracking_error_deg,
            self.max_hold_error_deg,
            self.max_velocity_rad_s,
            self.max_temperature_c,
            self.settling_band_deg,
            self.settling_window_s,
            self.prepare_speed_rad_s,
            self.prepare_settle_s,
            self.prepare_tolerance_deg,
        )
        if not np.all(np.isfinite(finite_positive)) or np.any(
            np.asarray(finite_positive) <= 0.0
        ):
            raise ValueError("MIT tuning limits and timing values must be finite and positive")
        max_step_deg = (90.0, 30.0, 30.0, 60.0, 88.0, 88.0)[self.joint_index]
        if self.step_deg > max_step_deg:
            raise ValueError(
                f"step_deg cannot exceed the {max_step_deg:g} degree tuning safety "
                f"limit for {ARM_JOINT_NAMES[self.joint_index]}"
            )
        if self.transition_s < 0.2:
            raise ValueError("transition_s must be at least 0.2 seconds")
        if not 1 <= self.cycles <= 10:
            raise ValueError("cycles must be in [1, 10]")
        if not 20.0 <= self.fps <= 200.0:
            raise ValueError("fps must be in [20, 200]")
        if not 1 <= self.fault_consecutive <= 20:
            raise ValueError("fault_consecutive must be in [1, 20]")
        if self.prepare_pose_deg is not None:
            object.__setattr__(
                self,
                "prepare_pose_deg",
                _vector(self.prepare_pose_deg, "prepare_pose_deg"),
            )


class TuningRobot(Protocol):
    joint_lower_deg: npt.NDArray[np.float64]
    joint_upper_deg: npt.NDArray[np.float64]

    def connect(self, *, calibrate: bool) -> None: ...

    def read_feedback(self) -> FeedbackFrame: ...

    def send_command(
        self,
        position_rad: npt.NDArray[np.float64],
        velocity_rad_s: npt.NDArray[np.float64],
    ) -> CommandFrame: ...

    def disconnect(self) -> None: ...


__all__ = [
    "ARM_JOINT_NAMES",
    "CommandFrame",
    "FeedbackFrame",
    "JOINT_ALIASES",
    "MITTuningConfig",
    "TuningRobot",
    "TuningSample",
]

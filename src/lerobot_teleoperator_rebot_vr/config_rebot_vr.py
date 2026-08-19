"""Configuration for the reBot B601 PICO 4 teleoperator plugin."""

from __future__ import annotations

from dataclasses import dataclass, field
import numpy as np
from lerobot.teleoperators.config import TeleoperatorConfig


REBOT_JOINTS = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_yaw",
    "wrist_roll",
    "gripper",
)

ARM_JOINTS = REBOT_JOINTS[:6]

# OpenXR X=right, Y=up, Z=backward -> reBot X=forward, Y=left, Z=up.
DEFAULT_BASE_T_ANCHOR: list[list[float]] = [
    [0.0, 0.0, -1.0, 0.0],
    [-1.0, 0.0, 0.0, 0.0],
    [0.0, 1.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 1.0],
]


@dataclass(kw_only=True)
class RebotVRConfig:
    """Runtime and mapping parameters shared by the registered config."""

    vr_backend: str = "xrobotoolkit_v1"
    hand_side: str = "right"

    clutch_threshold: float = 0.85
    clutch_release_threshold: float = 0.75
    stale_timeout: float = 0.2

    position_scale: float = 1.0
    orientation_scale: float = 1.0
    position_filter_hz: float = 8.0
    orientation_filter_hz: float = 6.0
    position_deadband_m: float = 5e-4
    orientation_deadband_rad: float = float(np.deg2rad(0.25))
    ik_rate_hz: float = 100.0
    ik_max_iterations: int = 50
    ik_tolerance_m: float = 5e-4
    ik_damping: float = 1e-4
    max_solution_jump_rad: float = 0.5
    max_joint_speed_rad_s: float = 2.0
    max_joint_acceleration_rad_s2: float = 8.0
    feedback_fault_max_consecutive: int = 5
    initial_q_rad: tuple[float, float, float, float, float, float] = (
        0.0,
        0.8,
        0.8,
        0.0,
        0.0,
        0.0,
    )
    gripper_max_speed_deg_s: float = 90.0
    gripper_max_acceleration_deg_s2: float = 360.0

    gripper_open: float = -270.0
    gripper_closed: float = 0.0

    app_name: str = "LeRobot-reBot-VR"
    auto_launch_cloudxr: bool = True
    cloudxr_env_file: str | None = None
    base_T_anchor: list[list[float]] = field(
        default_factory=lambda: [row.copy() for row in DEFAULT_BASE_T_ANCHOR]
    )

    # Kept as ws_host/ws_port for CLI compatibility with the requested interface.
    # The xrobotoolkit_v1 backend listens for the V1 binary TCP stream on this address.
    ws_host: str = "0.0.0.0"
    ws_port: int = 63901

    def __post_init__(self) -> None:
        if self.vr_backend not in ("isaac", "xrobotoolkit_v1"):
            raise ValueError("vr_backend must be 'isaac' or 'xrobotoolkit_v1'")
        if self.hand_side not in ("left", "right"):
            raise ValueError("hand_side must be 'left' or 'right'")
        if not 0.0 <= self.clutch_release_threshold < self.clutch_threshold <= 1.0:
            raise ValueError(
                "clutch thresholds must satisfy 0 <= release < press <= 1"
            )
        if not np.isfinite(self.stale_timeout) or self.stale_timeout <= 0.0:
            raise ValueError("stale_timeout must be finite and positive")
        if not np.all(np.isfinite([self.gripper_open, self.gripper_closed])):
            raise ValueError("gripper_open and gripper_closed must be finite")
        if not 1 <= self.ws_port <= 65535:
            raise ValueError("ws_port must be in [1, 65535]")

        cartesian_non_negative = np.asarray(
            (
                self.position_scale,
                self.orientation_scale,
                self.position_filter_hz,
                self.orientation_filter_hz,
                self.position_deadband_m,
                self.orientation_deadband_rad,
                self.ik_damping,
            ),
            dtype=float,
        )
        if not np.all(np.isfinite(cartesian_non_negative)) or np.any(
            cartesian_non_negative < 0.0
        ):
            raise ValueError("Cartesian scales, filters, deadbands, and damping must be non-negative")
        cartesian_positive = np.asarray(
            (
                self.ik_rate_hz,
                self.ik_tolerance_m,
                self.max_solution_jump_rad,
                self.max_joint_speed_rad_s,
                self.max_joint_acceleration_rad_s2,
                self.gripper_max_speed_deg_s,
                self.gripper_max_acceleration_deg_s2,
            ),
            dtype=float,
        )
        if not np.all(np.isfinite(cartesian_positive)) or np.any(
            cartesian_positive <= 0.0
        ):
            raise ValueError("Cartesian rates and motion limits must be positive")
        if self.ik_max_iterations <= 0:
            raise ValueError("ik_max_iterations must be positive")
        if self.feedback_fault_max_consecutive <= 0:
            raise ValueError("feedback_fault_max_consecutive must be positive")
        if np.asarray(self.initial_q_rad).shape != (6,) or not np.all(
            np.isfinite(self.initial_q_rad)
        ):
            raise ValueError("initial_q_rad must contain six finite values")

        transform = np.asarray(self.base_T_anchor, dtype=float)
        if transform.shape != (4, 4) or not np.all(np.isfinite(transform)):
            raise ValueError("base_T_anchor must be a finite 4x4 matrix")
        rotation = transform[:3, :3]
        if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6):
            raise ValueError("base_T_anchor rotation must be orthonormal")
        if not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-6):
            raise ValueError("base_T_anchor rotation must have determinant +1")
        if not np.allclose(transform[3], [0.0, 0.0, 0.0, 1.0], atol=1e-6):
            raise ValueError("base_T_anchor last row must be [0, 0, 0, 1]")


@TeleoperatorConfig.register_subclass("rebot_vr")
@dataclass(kw_only=True)
class RebotVRTeleopConfig(TeleoperatorConfig, RebotVRConfig):
    """Registered config selected by ``--teleop.type=rebot_vr``."""


__all__ = [
    "ARM_JOINTS",
    "DEFAULT_BASE_T_ANCHOR",
    "REBOT_JOINTS",
    "RebotVRConfig",
    "RebotVRTeleopConfig",
]

"""Compatibility VR frame used by non-V1 pose sources."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


def _unit_quaternion(value: object) -> np.ndarray:
    quaternion = np.asarray(value, dtype=float)
    if quaternion.shape != (4,) or not np.all(np.isfinite(quaternion)):
        raise ValueError("quaternion must be a finite [qx, qy, qz, qw] vector")
    norm = float(np.linalg.norm(quaternion))
    if norm < 1e-8:
        raise ValueError("quaternion norm is too small")
    return quaternion / norm


@dataclass(frozen=True)
class VRFrame:
    """Compatibility sample whose pose is already in the reBot base frame.

    The V1 closed-loop path uses :class:`ControllerSample` directly. This type
    remains the boundary for Isaac Teleop and the registered legacy plugin.
    """

    grip_pos: np.ndarray
    grip_quat: np.ndarray
    squeeze: float
    trigger: float
    is_tracking: bool = True
    received_monotonic_ns: int = 0
    tracking_timestamp_ns: int = 0
    stream_epoch: int = 0
    side: str = "right"
    primary_button: bool = False
    secondary_button: bool = False
    status: object = None
    head_pos: np.ndarray | None = None
    head_quat: np.ndarray | None = None

    def __post_init__(self) -> None:
        position = np.asarray(self.grip_pos, dtype=float)
        if position.shape != (3,) or not np.all(np.isfinite(position)):
            raise ValueError("grip_pos must be a finite three-vector")
        quaternion = _unit_quaternion(self.grip_quat)
        if not np.isfinite(self.squeeze) or not np.isfinite(self.trigger):
            raise ValueError("squeeze and trigger must be finite")
        if self.received_monotonic_ns < 0:
            raise ValueError("received_monotonic_ns must be non-negative")
        if self.tracking_timestamp_ns < 0:
            raise ValueError("tracking_timestamp_ns must be non-negative")
        if self.stream_epoch < 0:
            raise ValueError("stream_epoch must be non-negative")
        side = str(self.side).strip().lower()
        if side not in ("left", "right"):
            raise ValueError("side must be 'left' or 'right'")

        head_position = None
        head_quaternion = None
        if self.head_pos is not None or self.head_quat is not None:
            if self.head_pos is None or self.head_quat is None:
                raise ValueError("head_pos and head_quat must be provided together")
            head_position = np.asarray(self.head_pos, dtype=float)
            if head_position.shape != (3,) or not np.all(np.isfinite(head_position)):
                raise ValueError("head_pos must be a finite three-vector")
            head_quaternion = _unit_quaternion(self.head_quat)

        position = position.copy()
        position.flags.writeable = False
        quaternion.flags.writeable = False
        object.__setattr__(self, "grip_pos", position)
        object.__setattr__(self, "grip_quat", quaternion)
        object.__setattr__(self, "squeeze", float(np.clip(self.squeeze, 0.0, 1.0)))
        object.__setattr__(self, "trigger", float(np.clip(self.trigger, 0.0, 1.0)))
        object.__setattr__(self, "is_tracking", bool(self.is_tracking))
        object.__setattr__(self, "received_monotonic_ns", int(self.received_monotonic_ns))
        object.__setattr__(self, "tracking_timestamp_ns", int(self.tracking_timestamp_ns))
        object.__setattr__(self, "stream_epoch", int(self.stream_epoch))
        object.__setattr__(self, "side", side)
        object.__setattr__(self, "primary_button", bool(self.primary_button))
        object.__setattr__(self, "secondary_button", bool(self.secondary_button))
        if head_position is not None:
            head_position = head_position.copy()
            head_position.flags.writeable = False
        if head_quaternion is not None:
            head_quaternion.flags.writeable = False
        object.__setattr__(self, "head_pos", head_position)
        object.__setattr__(self, "head_quat", head_quaternion)
__all__ = ["VRFrame"]

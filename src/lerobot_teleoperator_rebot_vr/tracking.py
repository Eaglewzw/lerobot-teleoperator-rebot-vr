"""Validated XR controller samples and a thread-safe latest-only slot."""

from __future__ import annotations

import threading
import time
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from typing import Any, Literal

import numpy as np


ControllerSide = Literal["left", "right"]


class TrackingSampleError(ValueError):
    """A Tracking payload cannot be converted into a safe controller sample."""


def normalize_controller_side(side: str) -> ControllerSide:
    normalized = str(side).strip().lower()
    if normalized not in ("left", "right"):
        raise ValueError("controller side must be 'left' or 'right'")
    return normalized  # type: ignore[return-value]


def _readonly_vector(value: object, shape: tuple[int, ...], name: str) -> np.ndarray:
    array = np.asarray(value, dtype=np.float64)
    if array.shape != shape:
        raise TrackingSampleError(f"{name} must have shape {shape}, got {array.shape}")
    if not np.all(np.isfinite(array)):
        raise TrackingSampleError(f"{name} contains NaN or Inf")
    result = array.copy()
    result.flags.writeable = False
    return result


def _nonnegative_int(value: object, name: str) -> int:
    if isinstance(value, (bool, np.bool_)):
        raise TrackingSampleError(f"{name} must be an integer")
    if isinstance(value, (float, np.floating)):
        if not np.isfinite(value):
            raise TrackingSampleError(f"{name} contains NaN or Inf")
        if not float(value).is_integer():
            raise TrackingSampleError(f"{name} must be an integer")
    try:
        parsed = int(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise TrackingSampleError(f"{name} must be an integer") from exc
    if parsed < 0:
        raise TrackingSampleError(f"{name} must be non-negative")
    return parsed


def _parse_button(value: object, *, field_name: str) -> bool:
    if value is None:
        return False
    if isinstance(value, (bool, np.bool_)):
        return bool(value)
    if isinstance(value, (int, float, np.integer, np.floating)) and value in (0, 1):
        return bool(value)
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in ("true", "1"):
            return True
        if normalized in ("false", "0"):
            return False
    raise TrackingSampleError(f"{field_name} is not boolean")


@dataclass(frozen=True)
class ControllerSample:
    """One validated controller sample in the original XR coordinate frame.

    Quaternions are always stored as ``[qx, qy, qz, qw]``. Array fields own
    read-only copies, so publishing a sample across threads cannot expose later
    mutations by the receiver.
    """

    received_monotonic_ns: int
    tracking_timestamp_ns: int
    stream_epoch: int
    side: ControllerSide
    position: np.ndarray
    quaternion_xyzw: np.ndarray
    grip: float
    trigger: float
    primary_button: bool = False
    secondary_button: bool = False
    status: Any = None

    def __post_init__(self) -> None:
        received_ns = _nonnegative_int(
            self.received_monotonic_ns, "received_monotonic_ns"
        )
        tracking_ns = _nonnegative_int(
            self.tracking_timestamp_ns, "tracking_timestamp_ns"
        )
        epoch = _nonnegative_int(self.stream_epoch, "stream_epoch")
        side = normalize_controller_side(self.side)
        position = _readonly_vector(self.position, (3,), "position")
        quaternion = _readonly_vector(
            self.quaternion_xyzw, (4,), "quaternion_xyzw"
        )
        quaternion_norm = float(np.linalg.norm(quaternion))
        if quaternion_norm <= 1e-8:
            raise TrackingSampleError("quaternion norm must be greater than 1e-8")
        quaternion = quaternion / quaternion_norm
        quaternion.flags.writeable = False

        grip = float(self.grip)
        trigger = float(self.trigger)
        if not np.isfinite(grip) or not np.isfinite(trigger):
            raise TrackingSampleError("grip and trigger must be finite")
        if isinstance(self.status, (int, float, np.integer, np.floating)):
            if not np.isfinite(float(self.status)):
                raise TrackingSampleError("status contains NaN or Inf")

        object.__setattr__(self, "received_monotonic_ns", received_ns)
        object.__setattr__(self, "tracking_timestamp_ns", tracking_ns)
        object.__setattr__(self, "stream_epoch", epoch)
        object.__setattr__(self, "side", side)
        object.__setattr__(self, "position", position)
        object.__setattr__(self, "quaternion_xyzw", quaternion)
        object.__setattr__(self, "grip", float(np.clip(grip, 0.0, 1.0)))
        object.__setattr__(self, "trigger", float(np.clip(trigger, 0.0, 1.0)))
        object.__setattr__(
            self,
            "primary_button",
            _parse_button(self.primary_button, field_name="primary_button"),
        )
        object.__setattr__(
            self,
            "secondary_button",
            _parse_button(self.secondary_button, field_name="secondary_button"),
        )


def _parse_pose(value: object) -> np.ndarray:
    if isinstance(value, str):
        parts: Sequence[object] = [part.strip() for part in value.split(",")]
    elif isinstance(value, (Sequence, np.ndarray)) and not isinstance(
        value, (bytes, bytearray)
    ):
        parts = value
    else:
        raise TrackingSampleError(
            "controller pose must be a comma-separated string or a sequence"
        )
    if len(parts) != 7:
        raise TrackingSampleError(
            f"controller pose must contain 7 values, got {len(parts)}"
        )
    try:
        pose = np.asarray([float(part) for part in parts], dtype=np.float64)
    except (TypeError, ValueError) as exc:
        raise TrackingSampleError("controller pose contains a non-numeric value") from exc
    if not np.all(np.isfinite(pose)):
        raise TrackingSampleError("controller pose contains NaN or Inf")
    return pose


def _parse_scalar(value: object, *, default: float, field_name: str) -> float:
    if value is None:
        return default
    try:
        parsed = float(value)
    except (TypeError, ValueError) as exc:
        raise TrackingSampleError(f"{field_name} is not numeric") from exc
    if not np.isfinite(parsed):
        raise TrackingSampleError(f"{field_name} contains NaN or Inf")
    return parsed


def _parse_tracking_timestamp(value: object) -> int:
    if value is None:
        return 0
    return _nonnegative_int(value, "timeStampNs")


def parse_controller_sample(
    tracking: Mapping[str, object],
    side: str,
    *,
    received_monotonic_ns: int | None = None,
    stream_epoch: int = 0,
) -> ControllerSample:
    """Extract the selected hand from a decoded Tracking JSON object."""

    normalized_side = normalize_controller_side(side)
    controllers = tracking.get("Controller")
    if not isinstance(controllers, Mapping):
        raise TrackingSampleError("Tracking.Controller is missing or invalid")
    controller = controllers.get(normalized_side)
    if not isinstance(controller, Mapping):
        raise TrackingSampleError(
            f"Tracking.Controller.{normalized_side} is missing or invalid"
        )

    pose = _parse_pose(controller.get("pose"))
    return ControllerSample(
        received_monotonic_ns=(
            time.monotonic_ns()
            if received_monotonic_ns is None
            else received_monotonic_ns
        ),
        tracking_timestamp_ns=_parse_tracking_timestamp(
            tracking.get("timeStampNs")
        ),
        stream_epoch=stream_epoch,
        side=normalized_side,
        position=pose[:3],
        quaternion_xyzw=pose[3:],
        grip=_parse_scalar(controller.get("grip"), default=0.0, field_name="grip"),
        trigger=_parse_scalar(
            controller.get("trigger"), default=0.0, field_name="trigger"
        ),
        primary_button=_parse_button(
            controller.get("primaryButton"), field_name="primaryButton"
        ),
        secondary_button=_parse_button(
            controller.get("secondaryButton"), field_name="secondaryButton"
        ),
        status=controller.get("status"),
    )


class LatestSampleBuffer:
    """A thread-safe single slot; publishing replaces rather than queues."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._sample: ControllerSample | None = None
        self._sequence = 0

    def publish(self, sample: ControllerSample) -> int:
        if not isinstance(sample, ControllerSample):
            raise TypeError("sample must be a ControllerSample")
        with self._lock:
            self._sample = sample
            self._sequence += 1
            return self._sequence

    def clear(self) -> None:
        with self._lock:
            self._sample = None
            self._sequence += 1

    def latest(self) -> tuple[ControllerSample | None, int]:
        with self._lock:
            return self._sample, self._sequence


__all__ = [
    "ControllerSample",
    "ControllerSide",
    "LatestSampleBuffer",
    "TrackingSampleError",
    "normalize_controller_side",
    "parse_controller_sample",
]

"""Clutch-relative XR controller pose mapping for the robot end effector."""

from __future__ import annotations

import time
from dataclasses import dataclass
from enum import Enum

import numpy as np
from scipy.spatial.transform import Rotation

from .processor import VRFrame
from .tracking import ControllerSample, normalize_controller_side


DEFAULT_XR_TO_WORLD = np.array(
    [
        [0.0, 0.0, -1.0],
        [-1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
    ],
    dtype=np.float64,
)


class TeleopState(str, Enum):
    WAITING = "waiting"
    IDLE = "idle"
    ACTIVE = "active"
    STALE = "stale"
    HOLD = "hold"


def _readonly_array(value: object, shape: tuple[int, ...], name: str) -> np.ndarray:
    array = np.asarray(value, dtype=np.float64)
    if array.shape != shape:
        raise ValueError(f"{name} must have shape {shape}, got {array.shape}")
    if not np.all(np.isfinite(array)):
        raise ValueError(f"{name} contains NaN or Inf")
    result = array.copy()
    result.flags.writeable = False
    return result


def _rotation_matrix(value: object, name: str) -> np.ndarray:
    rotation = _readonly_array(value, (3, 3), name)
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6):
        raise ValueError(f"{name} must be orthonormal")
    if not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-6):
        raise ValueError(f"{name} must be right-handed")
    return rotation


@dataclass(frozen=True)
class PoseTarget:
    sample_id: int
    position: np.ndarray
    rotation: np.ndarray

    def __post_init__(self) -> None:
        if int(self.sample_id) < 0:
            raise ValueError("sample_id must be non-negative")
        object.__setattr__(self, "sample_id", int(self.sample_id))
        object.__setattr__(
            self, "position", _readonly_array(self.position, (3,), "target position")
        )
        object.__setattr__(
            self, "rotation", _rotation_matrix(self.rotation, "target rotation")
        )


@dataclass(frozen=True)
class OrientationDiagnostics:
    xr_delta_rotation: np.ndarray
    mapped_delta_rotation: np.ndarray
    ee_reference_rotation: np.ndarray

    def __post_init__(self) -> None:
        for field_name in (
            "xr_delta_rotation",
            "mapped_delta_rotation",
            "ee_reference_rotation",
        ):
            object.__setattr__(
                self,
                field_name,
                _rotation_matrix(getattr(self, field_name), field_name),
            )


@dataclass(frozen=True)
class PoseMappingUpdate:
    state: TeleopState
    target: PoseTarget | None
    reference_captured: bool = False
    require_release: bool = False
    orientation_diagnostics: OrientationDiagnostics | None = None


class RelativePoseMapper:
    """Map one hand's relative XR motion to an end-effector pose target."""

    def __init__(
        self,
        *,
        side: str = "right",
        xr_to_world: np.ndarray = DEFAULT_XR_TO_WORLD,
        position_scale: float = 1.0,
        orientation_scale: float = 1.0,
        position_filter_hz: float = 8.0,
        orientation_filter_hz: float = 6.0,
        position_deadband_m: float = 5e-4,
        orientation_deadband_rad: float = np.deg2rad(0.25),
        grip_press_threshold: float = 0.85,
        grip_release_threshold: float = 0.75,
        stale_timeout_s: float = 0.2,
        position_filter_cutoff_hz: float | None = None,
        orientation_filter_cutoff_hz: float | None = None,
        stale_timeout: float | None = None,
    ) -> None:
        self.side = normalize_controller_side(side)
        self.xr_to_world = _rotation_matrix(xr_to_world, "xr_to_world")
        if position_filter_cutoff_hz is not None:
            position_filter_hz = position_filter_cutoff_hz
        if orientation_filter_cutoff_hz is not None:
            orientation_filter_hz = orientation_filter_cutoff_hz
        if stale_timeout is not None:
            stale_timeout_s = stale_timeout

        values = np.asarray(
            (
                position_scale,
                orientation_scale,
                position_filter_hz,
                orientation_filter_hz,
                position_deadband_m,
                orientation_deadband_rad,
            ),
            dtype=np.float64,
        )
        if not np.all(np.isfinite(values)) or np.any(values < 0.0):
            raise ValueError("pose scales, filters, and deadbands must be non-negative")
        if not 0.0 <= grip_release_threshold < grip_press_threshold <= 1.0:
            raise ValueError("grip thresholds must satisfy 0 <= release < press <= 1")
        if not np.isfinite(stale_timeout_s) or stale_timeout_s <= 0.0:
            raise ValueError("stale timeout must be finite and positive")

        self.position_scale = float(position_scale)
        self.orientation_scale = float(orientation_scale)
        self.position_filter_hz = float(position_filter_hz)
        self.orientation_filter_hz = float(orientation_filter_hz)
        self.position_deadband_m = float(position_deadband_m)
        self.orientation_deadband_rad = float(orientation_deadband_rad)
        self.grip_press_threshold = float(grip_press_threshold)
        self.grip_release_threshold = float(grip_release_threshold)
        self.stale_timeout_ns = int(float(stale_timeout_s) * 1e9)

        self.state = TeleopState.WAITING
        self._ever_received_tracking = False
        self._stream_epoch: int | None = None
        self._require_release = True
        self._last_received_monotonic_ns = -1
        self._controller_position_ref: np.ndarray | None = None
        self._controller_rotation_ref: np.ndarray | None = None
        self._sample_to_world: np.ndarray | None = None
        self._ee_position_ref: np.ndarray | None = None
        self._ee_rotation_ref: np.ndarray | None = None
        self._filtered_target: PoseTarget | None = None
        self._last_filtered_sample_key: tuple[int, int, int] | None = None

    @property
    def require_release(self) -> bool:
        return self._require_release

    def reset(self, *, require_release: bool = True) -> None:
        self._clear_references()
        self.state = TeleopState.WAITING
        self._ever_received_tracking = False
        self._stream_epoch = None
        self._require_release = bool(require_release)
        self._last_received_monotonic_ns = -1

    def update(
        self,
        sample: ControllerSample | VRFrame | None,
        ee_position: np.ndarray,
        ee_rotation: np.ndarray,
        *,
        now_ns: int | None = None,
    ) -> PoseMappingUpdate:
        now = time.monotonic_ns() if now_ns is None else int(now_ns)
        ee_position_array = _readonly_array(ee_position, (3,), "ee_position")
        ee_rotation_array = _rotation_matrix(ee_rotation, "ee_rotation")

        if sample is None or (isinstance(sample, VRFrame) and not sample.is_tracking):
            self._enter_unavailable()
            return self._result(None)

        sample_side = sample.side
        if sample_side != self.side:
            raise ValueError(
                f"received {sample_side} controller sample for {self.side} mapper"
            )
        received_ns = int(sample.received_monotonic_ns)
        sample_age_ns = max(0, now - received_ns)
        if sample_age_ns > self.stale_timeout_ns:
            self._enter_unavailable()
            return self._result(None)

        self._ever_received_tracking = True
        if (
            self._stream_epoch is None
            or sample.stream_epoch != self._stream_epoch
            or received_ns < self._last_received_monotonic_ns
        ):
            self._stream_epoch = int(sample.stream_epoch)
            self._clear_references()
            self._require_release = True
            self.state = TeleopState.IDLE
        self._last_received_monotonic_ns = received_ns

        grip = sample.grip if isinstance(sample, ControllerSample) else sample.squeeze
        if self._require_release:
            self.state = TeleopState.IDLE
            self._clear_references()
            if grip <= self.grip_release_threshold:
                self._require_release = False
            return self._result(None)

        if self.state is TeleopState.ACTIVE:
            if grip <= self.grip_release_threshold:
                self._clear_references()
                self.state = TeleopState.IDLE
                return self._result(None)
            target, diagnostics = self._compute_target(sample)
            return self._result(target, orientation_diagnostics=diagnostics)

        if grip < self.grip_press_threshold:
            self.state = TeleopState.IDLE
            return self._result(None)

        self._capture_references(sample, ee_position_array, ee_rotation_array)
        self.state = TeleopState.ACTIVE
        initial_target = PoseTarget(received_ns, ee_position_array, ee_rotation_array)
        self._filtered_target = initial_target
        self._last_filtered_sample_key = self._sample_key(sample)
        return PoseMappingUpdate(
            state=self.state,
            target=initial_target,
            reference_captured=True,
            require_release=False,
            orientation_diagnostics=OrientationDiagnostics(
                xr_delta_rotation=np.eye(3),
                mapped_delta_rotation=np.eye(3),
                ee_reference_rotation=ee_rotation_array,
            ),
        )

    def _enter_unavailable(self) -> None:
        self._clear_references()
        self._require_release = True
        self.state = (
            TeleopState.STALE
            if self._ever_received_tracking
            else TeleopState.WAITING
        )

    def _capture_references(
        self,
        sample: ControllerSample | VRFrame,
        ee_position: np.ndarray,
        ee_rotation: np.ndarray,
    ) -> None:
        if isinstance(sample, ControllerSample):
            controller_position = sample.position
            controller_quaternion = sample.quaternion_xyzw
            sample_to_world = self.xr_to_world
        else:
            controller_position = sample.grip_pos
            controller_quaternion = sample.grip_quat
            sample_to_world = np.eye(3)
        self._controller_position_ref = np.asarray(
            controller_position, dtype=np.float64
        ).copy()
        self._controller_rotation_ref = Rotation.from_quat(
            controller_quaternion
        ).as_matrix()
        self._sample_to_world = np.asarray(sample_to_world, dtype=np.float64).copy()
        self._ee_position_ref = ee_position.copy()
        self._ee_rotation_ref = ee_rotation.copy()

    def _compute_target(
        self, sample: ControllerSample | VRFrame
    ) -> tuple[PoseTarget, OrientationDiagnostics]:
        if any(
            value is None
            for value in (
                self._controller_position_ref,
                self._controller_rotation_ref,
                self._sample_to_world,
                self._ee_position_ref,
                self._ee_rotation_ref,
            )
        ):
            raise RuntimeError("ACTIVE teleop state is missing reference poses")
        controller_position_ref = self._controller_position_ref
        controller_rotation_ref = self._controller_rotation_ref
        sample_to_world = self._sample_to_world
        ee_position_ref = self._ee_position_ref
        ee_rotation_ref = self._ee_rotation_ref
        assert controller_position_ref is not None
        assert controller_rotation_ref is not None
        assert sample_to_world is not None
        assert ee_position_ref is not None
        assert ee_rotation_ref is not None

        if isinstance(sample, ControllerSample):
            controller_position = sample.position
            controller_quaternion = sample.quaternion_xyzw
        else:
            controller_position = sample.grip_pos
            controller_quaternion = sample.grip_quat
        delta_position_world = self.position_scale * (
            sample_to_world @ (controller_position - controller_position_ref)
        )
        controller_rotation = Rotation.from_quat(controller_quaternion).as_matrix()
        delta_rotation_sample = controller_rotation @ controller_rotation_ref.T
        delta_rotation_world = (
            sample_to_world @ delta_rotation_sample @ sample_to_world.T
        )
        if not np.isclose(self.orientation_scale, 1.0):
            rotation_vector = Rotation.from_matrix(delta_rotation_world).as_rotvec()
            delta_rotation_world = Rotation.from_rotvec(
                self.orientation_scale * rotation_vector
            ).as_matrix()

        received_ns = int(sample.received_monotonic_ns)
        raw_target = PoseTarget(
            received_ns,
            ee_position_ref + delta_position_world,
            delta_rotation_world @ ee_rotation_ref,
        )
        filtered = self._filter_target(raw_target, sample)
        return filtered, OrientationDiagnostics(
            xr_delta_rotation=delta_rotation_sample,
            mapped_delta_rotation=delta_rotation_world,
            ee_reference_rotation=ee_rotation_ref,
        )

    def _filter_target(
        self, target: PoseTarget, sample: ControllerSample | VRFrame
    ) -> PoseTarget:
        sample_key = self._sample_key(sample)
        previous = self._filtered_target
        previous_key = self._last_filtered_sample_key
        if previous is None or previous_key is None:
            self._filtered_target = target
            self._last_filtered_sample_key = sample_key
            return target
        if sample_key == previous_key:
            return previous

        # Some V1 senders omit timeStampNs (parsed as zero) or restart their
        # upstream clock. Filtering must remain tied to PC receive time in
        # those cases, otherwise alpha becomes 1 and the low-pass is bypassed.
        if sample_key[1] > 0 and previous_key[1] > 0 and sample_key[1] > previous_key[1]:
            elapsed_s = (sample_key[1] - previous_key[1]) * 1e-9
        else:
            elapsed_s = max(0.0, (sample_key[2] - previous_key[2]) * 1e-9)
        position_delta = target.position - previous.position
        if np.linalg.norm(position_delta) < self.position_deadband_m:
            position_delta = np.zeros(3, dtype=np.float64)
        filtered_position = previous.position + self._filter_alpha(
            self.position_filter_hz, elapsed_s
        ) * position_delta

        rotation_delta = Rotation.from_matrix(
            target.rotation @ previous.rotation.T
        ).as_rotvec()
        if np.linalg.norm(rotation_delta) < self.orientation_deadband_rad:
            rotation_delta = np.zeros(3, dtype=np.float64)
        filtered_rotation = (
            Rotation.from_rotvec(
                self._filter_alpha(self.orientation_filter_hz, elapsed_s)
                * rotation_delta
            ).as_matrix()
            @ previous.rotation
        )
        filtered = PoseTarget(target.sample_id, filtered_position, filtered_rotation)
        self._filtered_target = filtered
        self._last_filtered_sample_key = sample_key
        return filtered

    @staticmethod
    def _sample_key(sample: ControllerSample | VRFrame) -> tuple[int, int, int]:
        return (
            int(sample.stream_epoch),
            int(sample.tracking_timestamp_ns),
            int(sample.received_monotonic_ns),
        )

    @staticmethod
    def _filter_alpha(cutoff_hz: float, dt_s: float) -> float:
        if cutoff_hz <= 0.0 or dt_s <= 0.0:
            return 1.0
        return float(1.0 - np.exp(-2.0 * np.pi * cutoff_hz * dt_s))

    def _clear_references(self) -> None:
        self._controller_position_ref = None
        self._controller_rotation_ref = None
        self._sample_to_world = None
        self._ee_position_ref = None
        self._ee_rotation_ref = None
        self._filtered_target = None
        self._last_filtered_sample_key = None

    def _result(
        self,
        target: PoseTarget | None,
        *,
        orientation_diagnostics: OrientationDiagnostics | None = None,
    ) -> PoseMappingUpdate:
        return PoseMappingUpdate(
            state=self.state,
            target=target,
            reference_captured=False,
            require_release=self._require_release,
            orientation_diagnostics=orientation_diagnostics,
        )


# Reference-project name retained alongside the package's earlier public name.
MappingUpdate = PoseMappingUpdate


__all__ = [
    "DEFAULT_XR_TO_WORLD",
    "MappingUpdate",
    "OrientationDiagnostics",
    "PoseMappingUpdate",
    "PoseTarget",
    "RelativePoseMapper",
    "TeleopState",
]

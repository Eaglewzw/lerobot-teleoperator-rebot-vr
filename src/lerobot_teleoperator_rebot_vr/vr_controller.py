"""PICO 4 pose sources for Isaac Teleop and XRoboToolkit V1 TCP."""

from __future__ import annotations

import logging
import os
import threading
import time
from collections.abc import Mapping
from pathlib import Path
from typing import Any, Protocol

import numpy as np
from scipy.spatial.transform import Rotation

from .config_rebot_vr import RebotVRConfig
from .processor import VRFrame
from .tracking import ControllerSample
from .xr_v1 import V1TrackingSource


logger = logging.getLogger(__name__)


class VRController(Protocol):
    @property
    def is_connected(self) -> bool: ...

    @property
    def is_tracking(self) -> bool: ...

    def connect(self) -> None: ...

    def get_action(self) -> dict[str, Any]: ...

    def latest_sample(self) -> ControllerSample | None: ...

    def disconnect(self) -> None: ...


def _safe_vr_action(frame: VRFrame | None = None) -> dict[str, Any]:
    if frame is None:
        return {
            "grip_pos": np.zeros(3, dtype=float),
            "grip_quat": np.array([0.0, 0.0, 0.0, 1.0], dtype=float),
            "squeeze": 0.0,
            "trigger": 0.0,
            "is_tracking": False,
            "received_monotonic_ns": time.monotonic_ns(),
            "tracking_timestamp_ns": 0,
            "stream_epoch": 0,
            "side": "right",
            "primary_button": False,
            "secondary_button": False,
            "status": None,
            "head_pos": None,
            "head_quat": None,
        }
    return {
        "grip_pos": frame.grip_pos.copy(),
        "grip_quat": frame.grip_quat.copy(),
        "squeeze": frame.squeeze,
        "trigger": frame.trigger,
        "is_tracking": frame.is_tracking,
        "received_monotonic_ns": frame.received_monotonic_ns,
        "tracking_timestamp_ns": frame.tracking_timestamp_ns,
        "stream_epoch": frame.stream_epoch,
        "side": frame.side,
        "primary_button": frame.primary_button,
        "secondary_button": frame.secondary_button,
        "status": frame.status,
        "head_pos": None if frame.head_pos is None else frame.head_pos.copy(),
        "head_quat": None if frame.head_quat is None else frame.head_quat.copy(),
    }


class Pico4VRController:
    """Raw PICO controller reader backed by Isaac Teleop and CloudXR."""

    _BASE_T_ANCHOR_INPUT = "base_T_anchor"
    _is_isaac_fix_available = False

    def __init__(self, config: RebotVRConfig) -> None:
        self.config = config
        self._session: Any = None
        self._cloudxr_launcher: Any = None
        self._external_inputs: dict[str, Any] | None = None
        self._is_tracking = False
        self._sdk: dict[str, Any] = {}

    @property
    def is_connected(self) -> bool:
        return self._session is not None

    @property
    def is_tracking(self) -> bool:
        return self._is_tracking

    @staticmethod
    def _load_sdk() -> dict[str, Any]:
        try:
            from isaacteleop.cloudxr import CloudXRLauncher
            from isaacteleop.retargeting_engine.deviceio_source_nodes import ControllersSource
            from isaacteleop.retargeting_engine.interface import (
                ExecutionEvents,
                ExecutionState,
                OutputCombiner,
                TensorGroup,
                ValueInput,
            )
            from isaacteleop.retargeting_engine.tensor_types import TransformMatrix
            from isaacteleop.retargeting_engine.tensor_types.indices import ControllerInputIndex
            from isaacteleop.teleop_session_manager import TeleopSession, TeleopSessionConfig
        except ImportError as exc:
            raise ImportError(
                "Isaac Teleop backend is unavailable. Install "
                "'isaacteleop[cloudxr,retargeters-lite]~=1.3.131', or use "
                "--teleop.vr_backend=xrobotoolkit_v1."
            ) from exc
        return locals()

    def connect(self) -> None:
        if self.is_connected:
            raise RuntimeError("Pico4VRController is already connected")
        self._sdk = self._load_sdk()
        self._ensure_cloudxr_runtime()
        try:
            pipeline = self._build_pipeline()
            session_config = self._sdk["TeleopSessionConfig"](
                app_name=self.config.app_name,
                pipeline=pipeline,
            )
            self._session = self._sdk["TeleopSession"](session_config)
            self._session.__enter__()
            self._external_inputs = self._build_external_inputs()
        except Exception:
            self._external_inputs = None
            try:
                if self._session is not None:
                    session = self._session
                    self._session = None
                    session.__exit__(None, None, None)
            finally:
                self._stop_cloudxr_runtime()
            raise

    def _build_pipeline(self) -> Any:
        controllers = self._sdk["ControllersSource"](name="controllers")
        transform = self._sdk["ValueInput"](
            self._BASE_T_ANCHOR_INPUT,
            self._sdk["TransformMatrix"](),
        )
        transformed = controllers.transformed(transform.output("value"))
        controller = transformed.output(f"controller_{self.config.hand_side}")
        return self._sdk["OutputCombiner"]({"controller": controller})

    def _build_external_inputs(self) -> dict[str, Any]:
        tensor_group = self._sdk["TensorGroup"](self._sdk["TransformMatrix"]())
        tensor_group[0] = np.asarray(self.config.base_T_anchor, dtype=np.float32)
        return {self._BASE_T_ANCHOR_INPUT: {"value": tensor_group}}

    def get_action(self) -> dict[str, Any]:
        if self._session is None:
            raise RuntimeError("Pico4VRController is not connected")
        events = self._sdk["ExecutionEvents"](
            execution_state=self._sdk["ExecutionState"].RUNNING,
            reset=False,
        )
        result = self._session.step(
            execution_events=events,
            external_inputs=self._external_inputs,
        )
        info = self._session.last_step_info
        if info is not None and info.worker_exception is not None:
            raise RuntimeError("Isaac Teleop worker failed") from info.worker_exception
        if info is not None and info.frame_deadline_miss:
            self._is_tracking = False
            logger.warning(
                "Isaac Teleop frame deadline missed (age=%s frames); disarming",
                info.returned_age_frames,
            )
            return _safe_vr_action()

        controller = result["controller"]
        self._is_tracking = not getattr(controller, "is_none", False)
        if not self._is_tracking:
            return _safe_vr_action()

        index = self._sdk["ControllerInputIndex"]
        try:
            frame = VRFrame(
                grip_pos=np.asarray(controller[index.GRIP_POSITION], dtype=float),
                grip_quat=np.asarray(controller[index.GRIP_ORIENTATION], dtype=float),
                squeeze=float(controller[index.SQUEEZE_VALUE]),
                trigger=float(controller[index.TRIGGER_VALUE]),
                received_monotonic_ns=time.monotonic_ns(),
                stream_epoch=1,
            )
        except (IndexError, KeyError, TypeError, ValueError):
            self._is_tracking = False
            return _safe_vr_action()
        return _safe_vr_action(frame)

    def latest_sample(self) -> ControllerSample | None:
        # Isaac's pipeline already applies base_T_anchor, so it exposes only the
        # base-frame compatibility action rather than pretending it is raw XR.
        return None

    def disconnect(self) -> None:
        self._is_tracking = False
        self._external_inputs = None
        try:
            if self._session is not None:
                session = self._session
                self._session = None
                session.__exit__(None, None, None)
        finally:
            self._stop_cloudxr_runtime()

    def _ensure_cloudxr_runtime(self) -> None:
        if self._cloudxr_launcher is not None:
            return
        if os.environ.get("LEROBOT_CLOUDXR_SKIP_AUTOLAUNCH", "").strip() == "1":
            return
        if not self.config.auto_launch_cloudxr:
            return
        self._cloudxr_launcher = self._sdk["CloudXRLauncher"](
            install_dir=str(Path.home() / ".cloudxr"),
            env_config=self.config.cloudxr_env_file,
            accept_eula=False,
        )

    def _stop_cloudxr_runtime(self) -> None:
        if self._cloudxr_launcher is None:
            return
        launcher = self._cloudxr_launcher
        try:
            launcher.stop()
        except RuntimeError:
            logger.warning("CloudXR runtime will be cleaned up by its atexit handler")
        else:
            self._cloudxr_launcher = None

    @staticmethod
    def _repair_urdf(path: str | Path) -> Path:
        """Compatibility hook retained for isaacteleop releases that require URDF repair."""
        return Path(path)


def _transform_pose(
    position: np.ndarray, quaternion: np.ndarray, transform: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    rotation = transform[:3, :3]
    transformed_position = rotation @ position + transform[:3, 3]
    source_rotation = Rotation.from_quat(quaternion).as_matrix()
    transformed_rotation = rotation @ source_rotation @ rotation.T
    return transformed_position, Rotation.from_matrix(transformed_rotation).as_quat()


class XRoboToolkitV1Controller:
    """VRController adapter over the validated latest-only V1 source."""

    def __init__(self, config: RebotVRConfig) -> None:
        self.config = config
        self._source = V1TrackingSource(
            host=config.ws_host,
            port=config.ws_port,
            side=config.hand_side,
            on_status=logger.info,
            on_sample=self._on_sample,
        )
        self._running = threading.Event()
        self._lock = threading.Lock()
        self._latest: VRFrame | None = None
        self._latest_sample: ControllerSample | None = None

    @property
    def is_connected(self) -> bool:
        return self._running.is_set() or self._source.running

    @property
    def is_tracking(self) -> bool:
        sample = self.latest_sample()
        return bool(sample is not None and self._is_fresh(sample))

    def connect(self) -> None:
        if self.is_connected:
            raise RuntimeError("XRoboToolkitV1Controller is already connected")
        self._source.start()
        self._running.set()

    def get_action(self) -> dict[str, Any]:
        if not self.is_connected:
            raise RuntimeError("XRoboToolkitV1Controller is not connected")
        sample = self._source.latest_sample()
        if sample is None:
            return _safe_vr_action()
        with self._lock:
            frame = self._latest
        if frame is None or not self._is_fresh(frame):
            return _safe_vr_action()
        return _safe_vr_action(frame)

    def disconnect(self) -> None:
        self._running.clear()
        self._source.stop()
        with self._lock:
            self._latest = None
            self._latest_sample = None

    def feed_bytes(self, data: bytes, *, received_monotonic_ns: int | None = None) -> None:
        self._source.feed_bytes(data, received_monotonic_ns=received_monotonic_ns)

    def latest_sample(self) -> ControllerSample | None:
        return self._source.latest_sample()

    def stats(self):
        return self._source.stats()

    def _on_sample(
        self, sample: ControllerSample, _tracking: Mapping[str, object]
    ) -> None:
        frame = self._tracking_to_frame(sample)
        with self._lock:
            self._latest_sample = sample
            self._latest = frame

    def _tracking_to_frame(self, sample: ControllerSample) -> VRFrame:
        transform = np.asarray(self.config.base_T_anchor, dtype=float)
        position, quaternion = _transform_pose(
            sample.position, sample.quaternion_xyzw, transform
        )
        return VRFrame(
            grip_pos=position,
            grip_quat=quaternion,
            squeeze=sample.grip,
            trigger=sample.trigger,
            received_monotonic_ns=sample.received_monotonic_ns,
            tracking_timestamp_ns=sample.tracking_timestamp_ns,
            stream_epoch=sample.stream_epoch,
            side=sample.side,
            primary_button=sample.primary_button,
            secondary_button=sample.secondary_button,
            status=sample.status,
        )

    def _is_fresh(self, frame: VRFrame | ControllerSample) -> bool:
        age_ns = max(0, time.monotonic_ns() - frame.received_monotonic_ns)
        return age_ns <= int(self.config.stale_timeout * 1e9)


def make_vr_controller(config: RebotVRConfig) -> VRController:
    if config.vr_backend == "isaac":
        return Pico4VRController(config)
    if config.vr_backend == "xrobotoolkit_v1":
        return XRoboToolkitV1Controller(config)
    raise ValueError(f"unsupported VR backend: {config.vr_backend}")


__all__ = [
    "Pico4VRController",
    "VRController",
    "XRoboToolkitV1Controller",
    "make_vr_controller",
]

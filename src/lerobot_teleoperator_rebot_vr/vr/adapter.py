from __future__ import annotations

from typing import overload

import numpy as np

from .models import VRFrame
from .tracking import ControllerSample


VR_SAMPLE_TYPES = ControllerSample | VRFrame


def sample_is_fresh(sample: VR_SAMPLE_TYPES | None, now_ns: int, timeout_s: float) -> bool:
    if sample is None or (isinstance(sample, VRFrame) and not sample.is_tracking):
        return False
    age_ns = max(0, now_ns - int(sample.received_monotonic_ns))
    return age_ns <= int(timeout_s * 1e9)


@overload
def sample_key(sample: VR_SAMPLE_TYPES) -> tuple[int, int, int]: ...


@overload
def sample_key(sample: None) -> None: ...


def sample_key(sample: VR_SAMPLE_TYPES | None) -> tuple[int, int, int] | None:
    if sample is None:
        return None
    return (
        int(sample.stream_epoch),
        int(sample.tracking_timestamp_ns),
        int(sample.received_monotonic_ns),
    )


def trigger_value(sample: VR_SAMPLE_TYPES | None) -> float:
    if sample is None:
        return 0.0
    return float(np.clip(sample.trigger, 0.0, 1.0))


def vr_frame_from_raw_action(action: dict[str, object]) -> VRFrame:
    """Convert the LeRobot action-shaped VR payload to the internal frame type."""
    return VRFrame(
        grip_pos=np.asarray(action["grip_pos"], dtype=np.float64),
        grip_quat=np.asarray(action["grip_quat"], dtype=np.float64),
        squeeze=float(action.get("squeeze", 0.0)),
        trigger=float(action["trigger"]),
        is_tracking=bool(action["is_tracking"]),
        received_monotonic_ns=int(action["received_monotonic_ns"]),
        published_monotonic_ns=int(action.get("published_monotonic_ns", 0)),
        tracking_timestamp_ns=int(action.get("tracking_timestamp_ns", 0)),
        stream_epoch=int(action.get("stream_epoch", 0)),
        side=str(action.get("side", "right")),
        primary_button=bool(action.get("primary_button", False)),
        secondary_button=bool(action.get("secondary_button", False)),
        status=action.get("status"),
        head_pos=action.get("head_pos"),
        head_quat=action.get("head_quat"),
    )

"""Deterministic single-axis excursion profile used for gain comparisons."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import numpy.typing as npt


@dataclass(frozen=True)
class ProfileSample:
    offset_rad: float
    velocity_rad_s: float
    phase: str
    phase_progress: float
    cycle: int
    done: bool = False


@dataclass(frozen=True)
class _Segment:
    start_rad: float
    end_rad: float
    duration_s: float
    phase: str
    cycle: int
    moving: bool


@dataclass(frozen=True)
class PoseTransitionSample:
    position_rad: npt.NDArray[np.float64]
    velocity_rad_s: npt.NDArray[np.float64]
    progress: float
    done: bool


class PoseTransition:
    """Minimum-jerk transition between two six-axis joint poses."""

    def __init__(self, start_rad: npt.ArrayLike, target_rad: npt.ArrayLike, duration_s: float):
        self.start_rad = np.asarray(start_rad, dtype=np.float64).copy()
        self.target_rad = np.asarray(target_rad, dtype=np.float64).copy()
        if (
            self.start_rad.shape != (6,)
            or self.target_rad.shape != (6,)
            or not np.all(np.isfinite(self.start_rad))
            or not np.all(np.isfinite(self.target_rad))
        ):
            raise ValueError("pose transition endpoints must contain six finite values")
        if not np.isfinite(duration_s) or duration_s <= 0.0:
            raise ValueError("pose transition duration must be finite and positive")
        self.duration_s = float(duration_s)

    def sample(self, elapsed_s: float) -> PoseTransitionSample:
        progress = float(np.clip(elapsed_s / self.duration_s, 0.0, 1.0))
        blend = 10.0 * progress**3 - 15.0 * progress**4 + 6.0 * progress**5
        blend_rate = (
            30.0 * progress**2 - 60.0 * progress**3 + 30.0 * progress**4
        ) / self.duration_s
        delta = self.target_rad - self.start_rad
        return PoseTransitionSample(
            position_rad=self.start_rad + delta * blend,
            velocity_rad_s=delta * blend_rate,
            progress=progress,
            done=progress >= 1.0,
        )


class ExcursionProfile:
    """Move center -> positive -> center -> negative -> center with smooth edges."""

    def __init__(
        self,
        *,
        step_rad: float,
        cycles: int,
        transition_s: float,
        hold_s: float,
        warmup_s: float,
    ) -> None:
        if step_rad <= 0.0:
            raise ValueError("step_rad must be positive")
        self._segments = [
            _Segment(0.0, 0.0, warmup_s, "warmup", 0, False)
        ]
        previous = 0.0
        for cycle in range(1, cycles + 1):
            for label, target in (
                ("positive", step_rad),
                ("center_after_positive", 0.0),
                ("negative", -step_rad),
                ("center_after_negative", 0.0),
            ):
                self._segments.append(
                    _Segment(
                        previous,
                        target,
                        transition_s,
                        f"move_{label}",
                        cycle,
                        True,
                    )
                )
                self._segments.append(
                    _Segment(target, target, hold_s, f"hold_{label}", cycle, False)
                )
                previous = target
        self.duration_s = sum(segment.duration_s for segment in self._segments)

    def sample(self, elapsed_s: float) -> ProfileSample:
        if elapsed_s <= 0.0:
            segment = self._segments[0]
            return ProfileSample(0.0, 0.0, segment.phase, 0.0, segment.cycle)
        if elapsed_s >= self.duration_s:
            final = self._segments[-1]
            return ProfileSample(0.0, 0.0, final.phase, 1.0, final.cycle, done=True)
        cursor = 0.0
        for segment in self._segments:
            end = cursor + segment.duration_s
            if elapsed_s < end:
                progress = (elapsed_s - cursor) / segment.duration_s
                if not segment.moving:
                    return ProfileSample(
                        segment.end_rad,
                        0.0,
                        segment.phase,
                        progress,
                        segment.cycle,
                    )
                blend = 10.0 * progress**3 - 15.0 * progress**4 + 6.0 * progress**5
                blend_rate = (
                    30.0 * progress**2
                    - 60.0 * progress**3
                    + 30.0 * progress**4
                ) / segment.duration_s
                delta = segment.end_rad - segment.start_rad
                return ProfileSample(
                    segment.start_rad + delta * blend,
                    delta * blend_rate,
                    segment.phase,
                    progress,
                    segment.cycle,
                )
            cursor = end
        final = self._segments[-1]
        return ProfileSample(0.0, 0.0, final.phase, 1.0, final.cycle, done=True)


__all__ = [
    "ExcursionProfile",
    "PoseTransition",
    "PoseTransitionSample",
    "ProfileSample",
]

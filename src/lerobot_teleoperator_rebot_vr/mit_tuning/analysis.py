"""Small, dependency-free metrics for comparing MIT tuning runs."""

from __future__ import annotations

import json
from collections import defaultdict
from pathlib import Path

import numpy as np

from .models import ARM_JOINT_NAMES, MITTuningConfig, TuningSample


class TuningAnalyzer:
    def __init__(self, config: MITTuningConfig) -> None:
        self.config = config
        self._samples: list[TuningSample] = []

    def add(self, sample: TuningSample) -> None:
        self._samples.append(sample)

    def summary(self) -> dict[str, object]:
        if not self._samples:
            return {"samples": 0, "selected_joint": ARM_JOINT_NAMES[self.config.joint_index]}
        index = self.config.joint_index
        actual = np.array(
            [np.rad2deg(sample.feedback.position_rad[index]) for sample in self._samples]
        )
        target = np.array(
            [np.rad2deg(sample.command.sent_position_rad[index]) for sample in self._samples]
        )
        velocity = np.array(
            [sample.feedback.velocity_rad_s[index] for sample in self._samples]
        )
        error = target - actual
        hold_indices = np.array(
            [sample.phase.startswith("hold_") for sample in self._samples], dtype=bool
        )
        steady_indices = np.array(
            [
                sample.phase.startswith("hold_") and sample.phase_progress >= 0.5
                for sample in self._samples
            ],
            dtype=bool,
        )
        groups: dict[tuple[int, str], list[int]] = defaultdict(list)
        for sample_index, sample in enumerate(self._samples):
            if sample.phase.startswith("hold_"):
                groups[(sample.cycle, sample.phase)].append(sample_index)

        settling_times: list[float] = []
        overshoots: list[float] = []
        peak_to_peak: list[float] = []
        for (_cycle, phase), indices in groups.items():
            group_error = error[indices]
            group_actual = actual[indices]
            progress = np.array([self._samples[item].phase_progress for item in indices])
            late = group_actual[progress >= 0.5]
            if late.size:
                peak_to_peak.append(float(np.ptp(late)))
            direction = self._phase_direction(phase)
            overshoots.append(float(max(0.0, np.max(-direction * group_error))))
            settled_at = self._settled_at(indices, error)
            if settled_at is not None:
                settling_times.append(settled_at)

        hold_error = error[hold_indices]
        steady_error = error[steady_indices]
        return {
            "samples": len(self._samples),
            "selected_joint": ARM_JOINT_NAMES[index],
            "kp": float(self._samples[0].kp[index]),
            "kd": float(self._samples[0].kd[index]),
            "tracking_rmse_deg": float(np.sqrt(np.mean(error**2))),
            "hold_rmse_deg": (
                float(np.sqrt(np.mean(hold_error**2))) if hold_error.size else None
            ),
            "steady_bias_deg": (
                float(np.mean(-steady_error)) if steady_error.size else None
            ),
            "max_abs_error_deg": float(np.max(np.abs(error))),
            "max_abs_velocity_rad_s": float(np.max(np.abs(velocity))),
            "max_overshoot_deg": max(overshoots, default=None),
            "max_late_hold_peak_to_peak_deg": max(peak_to_peak, default=None),
            "mean_settling_time_s": (
                float(np.mean(settling_times)) if settling_times else None
            ),
            "settled_holds": len(settling_times),
            "total_holds": len(groups),
        }

    def write_summary(self, path: str | Path) -> Path:
        output = Path(path).expanduser()
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(
            json.dumps(self.summary(), indent=2, ensure_ascii=True) + "\n",
            encoding="utf-8",
        )
        return output

    def _settled_at(self, indices: list[int], error: np.ndarray) -> float | None:
        elapsed = np.array([self._samples[item].elapsed_s for item in indices])
        within = np.abs(error[indices]) <= self.config.settling_band_deg
        for offset, is_within in enumerate(within):
            if not is_within:
                continue
            window_end = elapsed[offset] + self.config.settling_window_s
            end = int(np.searchsorted(elapsed, window_end, side="left"))
            if end < len(within) and np.all(within[offset : end + 1]):
                return float(elapsed[offset] - elapsed[0])
        return None

    @staticmethod
    def _phase_direction(phase: str) -> float:
        if phase in ("hold_positive", "hold_center_after_negative"):
            return 1.0
        return -1.0


__all__ = ["TuningAnalyzer"]

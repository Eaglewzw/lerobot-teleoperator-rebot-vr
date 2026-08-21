"""Offline loading and numerical analysis of reBot teleoperation CSV logs."""

from __future__ import annotations

import csv
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import numpy.typing as npt

from .csv_logger import JOINT_NAMES


KINDS = ("actual", "target", "command")
JOINT_LABELS = {
    joint: (f"q{index} {joint}" if index <= 6 else "gripper")
    for index, joint in enumerate(JOINT_NAMES, start=1)
}


@dataclass(frozen=True)
class SignalSpec:
    key: str
    label: str
    unit: str
    group: str
    column: str | None = None
    scale: float = 1.0
    minuend: str | None = None
    subtrahend: str | None = None


def _build_signal_specs() -> tuple[SignalSpec, ...]:
    specs: list[SignalSpec] = []
    for joint in JOINT_NAMES:
        joint_label = JOINT_LABELS[joint]
        for kind in KINDS:
            column = f"{kind}_{joint}_deg"
            specs.append(
                SignalSpec(
                    key=column,
                    label=f"{joint_label} {kind}",
                    unit="deg",
                    group="Joint position",
                    column=column,
                )
            )

    for joint in JOINT_NAMES:
        joint_label = JOINT_LABELS[joint]
        specs.extend(
            (
                SignalSpec(
                    key=f"actual_command_error_{joint}",
                    label=f"{joint_label} actual - command",
                    unit="deg",
                    group="Tracking error",
                    minuend=f"actual_{joint}_deg",
                    subtrahend=f"command_{joint}_deg",
                ),
                SignalSpec(
                    key=f"actual_target_error_{joint}",
                    label=f"{joint_label} actual - target",
                    unit="deg",
                    group="Tracking error",
                    minuend=f"actual_{joint}_deg",
                    subtrahend=f"target_{joint}_deg",
                ),
            )
        )

    specs.extend(
        (
            SignalSpec(
                "control_loop_hz",
                "Control loop frequency",
                "Hz",
                "IK and control diagnostics",
                column="control_loop_hz",
            ),
            SignalSpec(
                "ik_success",
                "IK success",
                "0/1",
                "IK and control diagnostics",
                column="ik_success",
            ),
            SignalSpec(
                "position_error_mm",
                "TCP position error",
                "mm",
                "IK and control diagnostics",
                column="position_error_m",
                scale=1000.0,
            ),
            SignalSpec(
                "orientation_error_deg",
                "TCP orientation error",
                "deg",
                "IK and control diagnostics",
                column="orientation_error_deg",
            ),
            SignalSpec(
                "sigma_min",
                "Jacobian sigma_min",
                "1",
                "IK and control diagnostics",
                column="sigma_min",
            ),
            SignalSpec(
                "condition_number",
                "Jacobian condition number",
                "1",
                "IK and control diagnostics",
                column="condition_number",
            ),
            SignalSpec(
                "qp_solve_time_ms",
                "QP solve time",
                "ms",
                "IK and control diagnostics",
                column="qp_solve_time_ms",
            ),
            SignalSpec(
                "dq_norm_rad_s",
                "Joint velocity norm",
                "rad/s",
                "IK and control diagnostics",
                column="dq_norm_rad_s",
            ),
        )
    )
    return tuple(specs)


SIGNAL_SPECS = _build_signal_specs()
SIGNAL_BY_KEY = {spec.key: spec for spec in SIGNAL_SPECS}
DEFAULT_SIGNAL_KEYS = tuple(
    f"{kind}_{JOINT_NAMES[0]}_deg" for kind in KINDS
)


def _numeric(value: str | None) -> float:
    if value is None or not value.strip():
        return math.nan
    try:
        result = float(value)
    except ValueError:
        return math.nan
    return result if math.isfinite(result) else math.nan


def _boolean_numeric(value: str | None) -> float:
    if value is None:
        return math.nan
    normalized = value.strip().lower()
    if normalized in ("true", "1"):
        return 1.0
    if normalized in ("false", "0"):
        return 0.0
    return math.nan


@dataclass(frozen=True)
class SignalStatistics:
    count: int
    minimum: float
    maximum: float
    mean: float
    rms: float
    peak_abs: float
    peak_time_s: float


@dataclass(frozen=True)
class TelemetryDataset:
    path: Path
    timestamp_ns: npt.NDArray[np.int64]
    time_s: npt.NDArray[np.float64]
    state: tuple[str, ...]
    ik_reason: tuple[str, ...]
    values: dict[str, npt.NDArray[np.float64]]
    invalid_rows: int

    @classmethod
    def load(cls, csv_path: str | Path) -> TelemetryDataset:
        path = Path(csv_path).expanduser().resolve()
        if not path.is_file():
            raise ValueError(f"CSV file does not exist: {path}")

        direct_columns = {
            spec.column for spec in SIGNAL_SPECS if spec.column is not None
        }
        required = {
            "timestamp_ns",
            *(f"{kind}_{joint}_deg" for kind in KINDS for joint in JOINT_NAMES),
        }
        timestamps: list[int] = []
        states: list[str] = []
        reasons: list[str] = []
        raw_values = {column: [] for column in direct_columns}
        invalid_rows = 0

        try:
            stream = path.open("r", newline="", encoding="utf-8-sig")
        except OSError as exc:
            raise ValueError(f"cannot open CSV file: {exc}") from exc
        with stream:
            reader = csv.DictReader(stream)
            if reader.fieldnames is None:
                raise ValueError("CSV file has no header")
            missing = sorted(required.difference(reader.fieldnames))
            if missing:
                raise ValueError("CSV header is missing fields: " + ", ".join(missing))

            for row in reader:
                try:
                    timestamp_ns = int(row.get("timestamp_ns", ""))
                except (TypeError, ValueError):
                    invalid_rows += 1
                    continue
                if timestamp_ns <= 0:
                    invalid_rows += 1
                    continue
                timestamps.append(timestamp_ns)
                states.append(row.get("teleop_state", "") or "unknown")
                reasons.append(row.get("ik_reason", ""))
                for column in direct_columns:
                    if column == "ik_success":
                        value = _boolean_numeric(row.get(column))
                    else:
                        value = _numeric(row.get(column))
                    raw_values[column].append(value)

        if not timestamps:
            raise ValueError("CSV file contains no valid telemetry rows")

        timestamp_array = np.asarray(timestamps, dtype=np.int64)
        order = np.argsort(timestamp_array, kind="stable")
        timestamp_array = timestamp_array[order]
        time_s = (timestamp_array - timestamp_array[0]).astype(np.float64) * 1e-9
        sorted_states = tuple(states[index] for index in order)
        sorted_reasons = tuple(reasons[index] for index in order)

        direct_arrays = {
            column: np.asarray(values, dtype=np.float64)[order]
            for column, values in raw_values.items()
        }
        signal_values: dict[str, npt.NDArray[np.float64]] = {}
        for spec in SIGNAL_SPECS:
            if spec.column is not None:
                signal_values[spec.key] = direct_arrays[spec.column] * spec.scale
            else:
                assert spec.minuend is not None and spec.subtrahend is not None
                signal_values[spec.key] = (
                    direct_arrays[spec.minuend] - direct_arrays[spec.subtrahend]
                )

        return cls(
            path=path,
            timestamp_ns=timestamp_array,
            time_s=time_s,
            state=sorted_states,
            ik_reason=sorted_reasons,
            values=signal_values,
            invalid_rows=invalid_rows,
        )

    @property
    def row_count(self) -> int:
        return int(self.time_s.size)

    @property
    def duration_s(self) -> float:
        return float(self.time_s[-1] - self.time_s[0])

    @property
    def median_sample_hz(self) -> float | None:
        delta = np.diff(self.time_s)
        valid = delta[np.isfinite(delta) & (delta > 0.0)]
        if not valid.size:
            return None
        return float(1.0 / np.median(valid))

    def index_at(self, time_s: float) -> int:
        index = int(np.searchsorted(self.time_s, time_s, side="left"))
        if index <= 0:
            return 0
        if index >= self.row_count:
            return self.row_count - 1
        before = index - 1
        return before if abs(self.time_s[before] - time_s) <= abs(self.time_s[index] - time_s) else index

    def range_indices(self, start_s: float, end_s: float) -> tuple[int, int]:
        lower, upper = sorted((float(start_s), float(end_s)))
        start = int(np.searchsorted(self.time_s, lower, side="left"))
        stop = int(np.searchsorted(self.time_s, upper, side="right"))
        start = min(max(start, 0), self.row_count - 1)
        stop = min(max(stop, start + 1), self.row_count)
        return start, stop

    def statistics(
        self,
        signal_key: str,
        start_s: float,
        end_s: float,
    ) -> SignalStatistics | None:
        start, stop = self.range_indices(start_s, end_s)
        values = self.values[signal_key][start:stop]
        times = self.time_s[start:stop]
        finite = np.isfinite(values)
        if not np.any(finite):
            return None
        finite_values = values[finite]
        finite_times = times[finite]
        peak_index = int(np.argmax(np.abs(finite_values)))
        return SignalStatistics(
            count=int(finite_values.size),
            minimum=float(np.min(finite_values)),
            maximum=float(np.max(finite_values)),
            mean=float(np.mean(finite_values)),
            rms=float(np.sqrt(np.mean(np.square(finite_values)))),
            peak_abs=float(abs(finite_values[peak_index])),
            peak_time_s=float(finite_times[peak_index]),
        )


def decimate_minmax(
    x: npt.NDArray[np.float64],
    y: npt.NDArray[np.float64],
    max_points: int,
) -> tuple[npt.NDArray[np.float64], npt.NDArray[np.float64]]:
    """Reduce a series while preserving each time bucket's extrema."""

    if max_points <= 0:
        raise ValueError("max_points must be positive")
    finite = np.isfinite(x) & np.isfinite(y)
    x = x[finite]
    y = y[finite]
    count = int(x.size)
    if count <= max_points:
        return x, y
    if max_points < 4:
        indices = np.linspace(0, count - 1, max_points, dtype=np.int64)
        return x[indices], y[indices]

    bucket_count = max(1, max_points // 2)
    edges = np.linspace(0, count, bucket_count + 1, dtype=np.int64)
    selected: list[int] = []
    for start, stop in zip(edges[:-1], edges[1:], strict=True):
        if stop <= start:
            continue
        bucket = y[start:stop]
        low = start + int(np.argmin(bucket))
        high = start + int(np.argmax(bucket))
        if low == high:
            selected.append(low)
        elif low < high:
            selected.extend((low, high))
        else:
            selected.extend((high, low))
    indices = np.asarray(selected, dtype=np.int64)
    return x[indices], y[indices]


__all__ = [
    "DEFAULT_SIGNAL_KEYS",
    "JOINT_LABELS",
    "SIGNAL_BY_KEY",
    "SIGNAL_SPECS",
    "SignalSpec",
    "SignalStatistics",
    "TelemetryDataset",
    "decimate_minmax",
]

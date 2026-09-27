"""Asynchronous per-frame CSV logging for real-robot teleoperation."""

from __future__ import annotations

import csv
import queue
import threading
import time
from collections.abc import Mapping
from pathlib import Path
from typing import TextIO

import numpy as np

from ..constants import JOINT_NAMES
from .motion import MOTION_FIELDNAMES
from .velocity import VELOCITY_FIELDNAMES
from ..control.types import (
    ARM_JOINT_NAMES,
    CartesianControlStatus,
)


TIMESTAMP_FIELDNAMES = (
    "monotonic_timestamp_ns",
    "loop_started_monotonic_ns",
    "tracking_sample_received_monotonic_ns",
    "tracking_sample_published_monotonic_ns",
    "tracking_sample_pickup_monotonic_ns",
    "tracking_timestamp_ns",
    "tracking_stream_epoch",
    "feedback_started_monotonic_ns",
    "feedback_finished_monotonic_ns",
    "controller_started_monotonic_ns",
    "controller_finished_monotonic_ns",
    "command_send_started_monotonic_ns",
    "command_send_finished_monotonic_ns",
    "ik_sequence",
    "ik_sample_id",
    "ik_sample_received_monotonic_ns",
    "ik_submitted_monotonic_ns",
    "ik_worker_started_monotonic_ns",
    "ik_completed_monotonic_ns",
    "ik_consumed_monotonic_ns",
)
LATENCY_FIELDNAMES = (
    "tracking_sample_age_ms",
    "vr_decode_ms",
    "latest_sample_wait_ms",
    "tracking_receive_to_pickup_ms",
    "feedback_read_ms",
    "fk_ms",
    "pose_mapping_ms",
    "qp_coordination_ms",
    "ik_sample_to_submit_ms",
    "ik_queue_wait_ms",
    "qp_solve_time_ms",
    "ik_worker_total_ms",
    "ik_result_wait_ms",
    "ik_sample_to_command_ready_ms",
    "command_shaping_ms",
    "controller_update_ms",
    "send_action_ms",
    "tracking_receive_to_send_ms",
    "ik_receive_to_send_ms",
    "command_to_next_feedback_ms",
    "cycle_work_ms",
)
MIT_FIELDNAMES = (
    "motor_control_mode",
    *(f"mit_desired_velocity_{joint}_rad_s" for joint in ARM_JOINT_NAMES),
    *(f"mit_gravity_{joint}_nm" for joint in ARM_JOINT_NAMES),
    *(f"mit_feedforward_{joint}_nm" for joint in ARM_JOINT_NAMES),
)
SUMMARY_FIELDNAMES = (
    "metric",
    "samples",
    "mean_ms",
    "min_ms",
    "p50_ms",
    "p95_ms",
    "p99_ms",
    "max_ms",
)
POSITION_FIELDNAMES = (
    "controller_position_frame",
    "mapping_sample_id",
    *(f"{kind}_{axis}_m" for kind in (
        "controller_position", "tcp_actual_position", "tcp_target_position",
        "tcp_position_error",
    ) for axis in "xyz"),
)
CSV_FIELDNAMES = (
    "timestamp_ns",
    *TIMESTAMP_FIELDNAMES,
    "control_loop_hz",
    "teleop_state",
    "ik_mode",
    "ik_success",
    "ik_reason",
    "wrist_clip_deg",
    "ik_result_consumed_this_cycle",
    *(f"{kind}_{joint}_deg" for kind in ("actual", "target", "command") for joint in JOINT_NAMES),
    "position_error_m",
    "orientation_error_deg",
    "sigma_min",
    "condition_number",
    "dq_norm_rad_s",
    *MIT_FIELDNAMES,
    *LATENCY_FIELDNAMES,
    *POSITION_FIELDNAMES,
    *VELOCITY_FIELDNAMES,
    *MOTION_FIELDNAMES,
)

_STOP = object()


def _optional_float(value: float | None) -> float | str:
    return "" if value is None else float(value)


def _optional_int(value: int | None) -> int | str:
    return "" if value is None else int(value)


def build_csv_row(status: CartesianControlStatus) -> dict[str, object]:
    """Flatten one immutable controller status into a CSV-compatible row."""

    vectors = {
        "actual": np.asarray(status.actual_deg, dtype=np.float64),
        "target": np.asarray(status.target_deg, dtype=np.float64),
        "command": np.asarray(status.command_deg, dtype=np.float64),
    }
    for kind, values in vectors.items():
        if values.shape != (len(JOINT_NAMES),):
            raise ValueError(
                f"status.{kind}_deg must contain {len(JOINT_NAMES)} joints"
            )

    row: dict[str, object] = {"timestamp_ns": time.time_ns()}
    for field_name in TIMESTAMP_FIELDNAMES:
        if field_name == "monotonic_timestamp_ns":
            row[field_name] = time.monotonic_ns()
        else:
            row[field_name] = _optional_int(getattr(status, field_name))
    row.update(
        {
            "control_loop_hz": _optional_float(status.control_loop_hz),
            "teleop_state": status.state.value,
            "ik_mode": status.ik_mode,
            "ik_success": "" if status.ik_success is None else status.ik_success,
            "ik_reason": status.ik_reason,
            "wrist_clip_deg": _optional_float(status.wrist_clip_deg),
            "ik_result_consumed_this_cycle": (
                status.ik_result_consumed_this_cycle
            ),
        }
    )
    for kind, values in vectors.items():
        for joint, value in zip(JOINT_NAMES, values, strict=True):
            row[f"{kind}_{joint}_deg"] = float(value)
    row.update(
        {
            "position_error_m": _optional_float(status.tcp_position_error_m),
            "orientation_error_deg": _optional_float(status.orientation_error_deg),
            "sigma_min": _optional_float(status.sigma_min),
            "condition_number": _optional_float(status.condition_number),
            "dq_norm_rad_s": _optional_float(status.dq_norm_rad_s),
            "motor_control_mode": status.motor_control_mode,
        }
    )
    for prefix, values in (
        ("mit_desired_velocity", status.mit_desired_velocity_rad_s),
        ("mit_gravity", status.mit_gravity_torque_nm),
        ("mit_feedforward", status.mit_feedforward_torque_nm),
    ):
        vector = None if values is None else np.asarray(values, dtype=np.float64)
        if vector is not None and vector.shape != (len(ARM_JOINT_NAMES),):
            raise ValueError(f"status.{prefix} must contain six arm joints")
        for index, joint in enumerate(ARM_JOINT_NAMES):
            suffix = "rad_s" if prefix == "mit_desired_velocity" else "nm"
            row[f"{prefix}_{joint}_{suffix}"] = (
                "" if vector is None else float(vector[index])
            )
    for field_name in LATENCY_FIELDNAMES:
        row[field_name] = _optional_float(getattr(status, field_name))
    row["controller_position_frame"] = status.controller_position_frame
    row["mapping_sample_id"] = _optional_int(status.mapping_sample_id)
    positions = {}
    for kind in ("controller_position", "tcp_actual_position", "tcp_target_position"):
        value = getattr(status, f"{kind}_m")
        vector = None if value is None else np.asarray(value, dtype=np.float64)
        if vector is not None and vector.shape != (3,):
            raise ValueError(f"status.{kind}_m must contain three coordinates")
        positions[kind] = vector
    actual, target = positions["tcp_actual_position"], positions["tcp_target_position"]
    positions["tcp_position_error"] = (
        None if actual is None or target is None else target - actual
    )
    for kind, vector in positions.items():
        for index, axis in enumerate("xyz"):
            row[f"{kind}_{axis}_m"] = "" if vector is None else float(vector[index])
    row.update({key: status.velocity_diagnostics.get(key, "") for key in VELOCITY_FIELDNAMES})
    row["ik_result_applied_this_cycle"] = status.ik_result_applied_this_cycle
    row.update(dict.fromkeys(MOTION_FIELDNAMES, ""))
    row.update(phase="teleop", row_kind="sample")
    return row


class CSVLogger:
    """Write CSV rows on a daemon thread without disk I/O in the control loop."""

    def __init__(self, output_path: str | Path) -> None:
        self.output_path = Path(output_path).expanduser()
        self.summary_path = self.output_path.with_name(
            f"{self.output_path.stem}_latency_summary.csv"
        )
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        self._file: TextIO = self.output_path.open("w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(self._file, fieldnames=CSV_FIELDNAMES)
        self._writer.writeheader()
        self._file.flush()
        self._queue: queue.Queue[dict[str, object] | object] = queue.Queue()
        self._closed = False
        self._error: BaseException | None = None
        self._thread = threading.Thread(
            target=self._run,
            name="rebot-vr-csv",
            daemon=True,
        )
        self._thread.start()

    def write_row(self, row: Mapping[str, object]) -> None:
        """Enqueue a row snapshot without waiting for the writer thread."""

        if self._closed:
            raise RuntimeError("CSV logger is closed")
        self._queue.put_nowait(dict(row))

    def close(self) -> None:
        """Drain all queued rows, stop the writer, and close the file."""

        if self._closed:
            return
        self._closed = True
        self._queue.put_nowait(_STOP)
        self._thread.join()
        if self._error is not None:
            raise RuntimeError(f"CSV writer failed: {self._error}") from self._error

    def _run(self) -> None:
        latency_values: dict[str, list[float]] = {
            name: [] for name in LATENCY_FIELDNAMES
        }
        try:
            while True:
                item = self._queue.get()
                if item is _STOP:
                    break
                assert isinstance(item, dict)
                self._writer.writerow(item)
                for field_name, values in latency_values.items():
                    value = item.get(field_name)
                    if value in (None, ""):
                        continue
                    parsed = float(value)
                    if np.isfinite(parsed) and parsed >= 0.0:
                        values.append(parsed)
                self._file.flush()
            self._write_latency_summary(latency_values)
        except BaseException as exc:
            self._error = exc
        finally:
            self._file.close()

    def _write_latency_summary(
        self, latency_values: Mapping[str, list[float]]
    ) -> None:
        with self.summary_path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=SUMMARY_FIELDNAMES)
            writer.writeheader()
            for metric in LATENCY_FIELDNAMES:
                values = np.asarray(latency_values[metric], dtype=np.float64)
                if values.size == 0:
                    writer.writerow({"metric": metric, "samples": 0})
                    continue
                writer.writerow(
                    {
                        "metric": metric,
                        "samples": int(values.size),
                        "mean_ms": float(np.mean(values)),
                        "min_ms": float(np.min(values)),
                        "p50_ms": float(np.percentile(values, 50)),
                        "p95_ms": float(np.percentile(values, 95)),
                        "p99_ms": float(np.percentile(values, 99)),
                        "max_ms": float(np.max(values)),
                    }
                )


__all__ = [
    "CSV_FIELDNAMES",
    "CSVLogger",
    "JOINT_NAMES",
    "LATENCY_FIELDNAMES",
    "MIT_FIELDNAMES",
    "SUMMARY_FIELDNAMES",
    "TIMESTAMP_FIELDNAMES",
    "build_csv_row",
]

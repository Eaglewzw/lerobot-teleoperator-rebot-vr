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

from .cartesian_controller import (
    ARM_JOINT_NAMES,
    GRIPPER_NAME,
    CartesianControlStatus,
)


JOINT_NAMES = (*ARM_JOINT_NAMES, GRIPPER_NAME)
CSV_FIELDNAMES = (
    "timestamp_ns",
    "control_loop_hz",
    "teleop_state",
    "ik_success",
    "ik_reason",
    *(f"{kind}_{joint}_deg" for kind in ("actual", "target", "command") for joint in JOINT_NAMES),
    "position_error_m",
    "orientation_error_deg",
    "sigma_min",
    "condition_number",
    "qp_solve_time_ms",
    "dq_norm_rad_s",
)

_STOP = object()


def _optional_float(value: float | None) -> float | str:
    return "" if value is None else float(value)


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

    row: dict[str, object] = {
        "timestamp_ns": time.time_ns(),
        "control_loop_hz": _optional_float(status.control_loop_hz),
        "teleop_state": status.state.value,
        "ik_success": "" if status.ik_success is None else status.ik_success,
        "ik_reason": status.ik_reason,
    }
    for kind, values in vectors.items():
        for joint, value in zip(JOINT_NAMES, values, strict=True):
            row[f"{kind}_{joint}_deg"] = float(value)
    row.update(
        {
            "position_error_m": _optional_float(status.tcp_position_error_m),
            "orientation_error_deg": _optional_float(status.orientation_error_deg),
            "sigma_min": _optional_float(status.sigma_min),
            "condition_number": _optional_float(status.condition_number),
            "qp_solve_time_ms": _optional_float(status.qp_solve_time_ms),
            "dq_norm_rad_s": _optional_float(status.dq_norm_rad_s),
        }
    )
    return row


class CSVLogger:
    """Write CSV rows on a daemon thread without disk I/O in the control loop."""

    def __init__(self, output_path: str | Path) -> None:
        self.output_path = Path(output_path).expanduser()
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
        try:
            while True:
                item = self._queue.get()
                if item is _STOP:
                    break
                assert isinstance(item, dict)
                self._writer.writerow(item)
                self._file.flush()
        except BaseException as exc:
            self._error = exc
        finally:
            self._file.close()


__all__ = ["CSV_FIELDNAMES", "CSVLogger", "JOINT_NAMES", "build_csv_row"]

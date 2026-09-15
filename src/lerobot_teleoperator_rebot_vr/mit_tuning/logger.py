"""Asynchronous CSV writer dedicated to MIT tuning sessions."""

from __future__ import annotations

import csv
import queue
import threading
from pathlib import Path

import numpy as np

from .models import ARM_JOINT_NAMES, TuningSample


BASE_FIELDS = (
    "wall_time_ns",
    "monotonic_ns",
    "elapsed_s",
    "loop_hz",
    "phase",
    "phase_progress",
    "cycle",
    "selected_joint",
    "feedback_read_ms",
    "command_send_ms",
)
JOINT_FIELDS = tuple(
    f"{field}_{joint}{suffix}"
    for joint in ARM_JOINT_NAMES
    for field, suffix in (
        ("kp", ""),
        ("kd", ""),
        ("actual", "_deg"),
        ("target", "_deg"),
        ("sent", "_deg"),
        ("error", "_deg"),
        ("actual_velocity", "_rad_s"),
        ("desired_velocity", "_rad_s"),
        ("actual_torque", "_nm"),
        ("gravity", "_nm"),
        ("feedforward", "_nm"),
        ("status", ""),
        ("mos_temperature", "_c"),
        ("rotor_temperature", "_c"),
    )
)
CSV_FIELDS = (*BASE_FIELDS, *JOINT_FIELDS)
_STOP = object()


def build_row(sample: TuningSample) -> dict[str, object]:
    feedback = sample.feedback
    command = sample.command
    actual_deg = np.rad2deg(feedback.position_rad)
    target_deg = np.rad2deg(command.requested_position_rad)
    sent_deg = np.rad2deg(command.sent_position_rad)
    row: dict[str, object] = {
        "wall_time_ns": sample.wall_time_ns,
        "monotonic_ns": feedback.monotonic_ns,
        "elapsed_s": sample.elapsed_s,
        "loop_hz": "" if sample.loop_hz is None else sample.loop_hz,
        "phase": sample.phase,
        "phase_progress": sample.phase_progress,
        "cycle": sample.cycle,
        "selected_joint": ARM_JOINT_NAMES[sample.joint_index],
        "feedback_read_ms": sample.feedback_read_ms,
        "command_send_ms": sample.command_send_ms,
    }
    for index, joint in enumerate(ARM_JOINT_NAMES):
        values = (
            (f"kp_{joint}", sample.kp[index]),
            (f"kd_{joint}", sample.kd[index]),
            (f"actual_{joint}_deg", actual_deg[index]),
            (f"target_{joint}_deg", target_deg[index]),
            (f"sent_{joint}_deg", sent_deg[index]),
            (f"error_{joint}_deg", sent_deg[index] - actual_deg[index]),
            (f"actual_velocity_{joint}_rad_s", feedback.velocity_rad_s[index]),
            (f"desired_velocity_{joint}_rad_s", command.desired_velocity_rad_s[index]),
            (f"actual_torque_{joint}_nm", feedback.torque_nm[index]),
            (f"gravity_{joint}_nm", command.gravity_torque_nm[index]),
            (f"feedforward_{joint}_nm", command.feedforward_torque_nm[index]),
            (f"status_{joint}", int(feedback.status_code[index])),
            (f"mos_temperature_{joint}_c", feedback.mos_temperature_c[index]),
            (f"rotor_temperature_{joint}_c", feedback.rotor_temperature_c[index]),
        )
        for field, value in values:
            row[field] = value if field.startswith("status_") else float(value)
    return row


class TuningCSVLogger:
    def __init__(self, output_path: str | Path, *, queue_size: int = 8192) -> None:
        self.output_path = Path(output_path).expanduser()
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        self._file = self.output_path.open("w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(self._file, fieldnames=CSV_FIELDS)
        self._writer.writeheader()
        self._queue: queue.Queue[dict[str, object] | object] = queue.Queue(
            maxsize=queue_size
        )
        self._closed = False
        self._error: BaseException | None = None
        self.dropped_rows = 0
        self._thread = threading.Thread(
            target=self._run,
            name="rebot-mit-tuning-csv",
            daemon=True,
        )
        self._thread.start()

    def write_sample(self, sample: TuningSample) -> None:
        if self._closed:
            raise RuntimeError("MIT tuning CSV logger is closed")
        try:
            self._queue.put_nowait(build_row(sample))
        except queue.Full:
            self.dropped_rows += 1

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        while self._thread.is_alive():
            try:
                self._queue.put(_STOP, timeout=0.05)
                break
            except queue.Full:
                continue
        self._thread.join()
        if self._error is not None:
            raise RuntimeError(f"MIT tuning CSV writer failed: {self._error}") from self._error

    def _run(self) -> None:
        try:
            while True:
                item = self._queue.get()
                if item is _STOP:
                    break
                assert isinstance(item, dict)
                self._writer.writerow(item)
                if self._queue.empty():
                    self._file.flush()
        except BaseException as exc:
            self._error = exc
        finally:
            self._file.close()


__all__ = ["CSV_FIELDS", "TuningCSVLogger", "build_row"]

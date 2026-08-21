from __future__ import annotations

import csv
from pathlib import Path

import numpy as np
import pytest

from lerobot_teleoperator_rebot_vr.csv_analysis import (
    SIGNAL_SPECS,
    TelemetryDataset,
    decimate_minmax,
)
from lerobot_teleoperator_rebot_vr.csv_logger import CSV_FIELDNAMES, JOINT_NAMES
from lerobot_teleoperator_rebot_vr.csv_viewer import build_parser


def _row(timestamp_ns: int | str, offset: float = 0.0) -> dict[str, object]:
    row: dict[str, object] = {field: "" for field in CSV_FIELDNAMES}
    row.update(
        {
            "timestamp_ns": timestamp_ns,
            "control_loop_hz": 70.0 + offset,
            "teleop_state": "active",
            "ik_success": offset != 2.0,
            "ik_reason": "timeout" if offset == 2.0 else "",
            "position_error_m": 0.001 * (offset + 1.0),
            "orientation_error_deg": 0.5 + offset,
            "sigma_min": 0.1 - offset * 0.01,
            "condition_number": 10.0 + offset,
            "qp_solve_time_ms": 1.0 + offset,
            "dq_norm_rad_s": 0.2 + offset,
        }
    )
    for index, joint in enumerate(JOINT_NAMES):
        row[f"actual_{joint}_deg"] = index + offset
        row[f"target_{joint}_deg"] = index + offset + 1.0
        row[f"command_{joint}_deg"] = index + offset + 0.25
    return row


def _write(path: Path, rows: list[dict[str, object]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=CSV_FIELDNAMES)
        writer.writeheader()
        writer.writerows(rows)


def test_offline_dataset_loads_sorts_and_derives_all_signals(tmp_path) -> None:
    path = tmp_path / "session.csv"
    base = 2_000_000_000_000_000_000
    _write(
        path,
        [
            _row(base + 200_000_000, 2.0),
            _row("bad", 9.0),
            _row(base, 0.0),
            _row(base + 100_000_000, 1.0),
        ],
    )

    dataset = TelemetryDataset.load(path)

    assert dataset.row_count == 3
    assert dataset.invalid_rows == 1
    assert dataset.time_s == pytest.approx([0.0, 0.1, 0.2])
    assert dataset.duration_s == pytest.approx(0.2)
    assert dataset.median_sample_hz == pytest.approx(10.0)
    assert len(dataset.values) == len(SIGNAL_SPECS) == 43
    assert dataset.values["actual_shoulder_pan_deg"] == pytest.approx([0.0, 1.0, 2.0])
    assert dataset.values["actual_command_error_shoulder_pan"] == pytest.approx(
        [-0.25, -0.25, -0.25]
    )
    assert dataset.values["actual_target_error_shoulder_pan"] == pytest.approx(
        [-1.0, -1.0, -1.0]
    )
    assert dataset.values["position_error_mm"] == pytest.approx([1.0, 2.0, 3.0])
    assert dataset.values["ik_success"] == pytest.approx([1.0, 1.0, 0.0])
    assert dataset.index_at(0.16) == 2


def test_visible_range_statistics_use_original_rows(tmp_path) -> None:
    path = tmp_path / "session.csv"
    base = 1_000_000_000
    _write(path, [_row(base + index * 1_000_000_000, float(index)) for index in range(5)])
    dataset = TelemetryDataset.load(path)

    stats = dataset.statistics("actual_shoulder_pan_deg", 1.0, 3.0)

    assert stats is not None
    assert stats.count == 3
    assert stats.minimum == pytest.approx(1.0)
    assert stats.maximum == pytest.approx(3.0)
    assert stats.mean == pytest.approx(2.0)
    assert stats.rms == pytest.approx(np.sqrt(14.0 / 3.0))
    assert stats.peak_abs == pytest.approx(3.0)
    assert stats.peak_time_s == pytest.approx(3.0)


def test_minmax_decimation_preserves_short_spikes() -> None:
    x = np.linspace(0.0, 10.0, 10_001)
    y = np.sin(x)
    y[2_345] = 50.0
    y[7_654] = -40.0

    reduced_x, reduced_y = decimate_minmax(x, y, max_points=200)

    assert reduced_x.size <= 200
    assert np.max(reduced_y) == pytest.approx(50.0)
    assert np.min(reduced_y) == pytest.approx(-40.0)
    assert np.all(np.diff(reduced_x) >= 0.0)


def test_dataset_rejects_missing_joint_columns(tmp_path) -> None:
    path = tmp_path / "invalid.csv"
    path.write_text("timestamp_ns,teleop_state\n1,active\n", encoding="utf-8")

    with pytest.raises(ValueError, match="missing fields"):
        TelemetryDataset.load(path)


def test_desktop_analyzer_cli() -> None:
    args = build_parser().parse_args(["logs/session.csv"])
    assert args.csv_path == Path("logs/session.csv")
    assert args.geometry == "1500x900"

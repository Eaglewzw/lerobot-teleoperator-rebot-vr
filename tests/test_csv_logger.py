from __future__ import annotations

import csv

import numpy as np
import pytest

from lerobot_teleoperator_rebot_vr.cartesian_controller import CartesianControlStatus
from lerobot_teleoperator_rebot_vr.csv_logger import (
    CSV_FIELDNAMES,
    CSVLogger,
    build_csv_row,
)
from lerobot_teleoperator_rebot_vr.pose_mapping import TeleopState
from lerobot_teleoperator_rebot_vr.teleop_cli import build_parser


def _status() -> CartesianControlStatus:
    return CartesianControlStatus(
        state=TeleopState.ACTIVE,
        tracking=True,
        ik_success=True,
        ik_error_m=0.002,
        ik_reason="",
        submitted=3,
        solved=2,
        rejected=0,
        trigger=0.25,
        gripper_trigger_active=True,
        primary_button=False,
        secondary_button=False,
        home_requested=False,
        zero_requested=False,
        generation=1,
        gripper_actual_deg=-40.0,
        gripper_target_deg=-30.0,
        gripper_command_deg=-35.0,
        actual_deg=np.arange(7, dtype=np.float64),
        target_deg=np.arange(10, 17, dtype=np.float64),
        command_deg=np.arange(20, 27, dtype=np.float64),
        orientation_error_deg=1.25,
        sigma_min=0.08,
        condition_number=12.5,
        dq_norm_rad_s=0.6,
        qp_solve_time_ms=1.5,
        tcp_position_error_m=0.003,
        control_loop_hz=59.9,
    )


def test_build_csv_row_flattens_seven_joint_status_and_diagnostics() -> None:
    row = build_csv_row(_status())

    assert tuple(row) == CSV_FIELDNAMES
    assert row["teleop_state"] == "active"
    assert row["ik_success"] is True
    assert row["control_loop_hz"] == pytest.approx(59.9)
    assert row["actual_shoulder_pan_deg"] == pytest.approx(0.0)
    assert row["actual_gripper_deg"] == pytest.approx(6.0)
    assert row["target_wrist_roll_deg"] == pytest.approx(15.0)
    assert row["command_gripper_deg"] == pytest.approx(26.0)
    assert row["position_error_m"] == pytest.approx(0.003)
    assert row["orientation_error_deg"] == pytest.approx(1.25)
    assert row["sigma_min"] == pytest.approx(0.08)
    assert row["condition_number"] == pytest.approx(12.5)
    assert row["qp_solve_time_ms"] == pytest.approx(1.5)
    assert row["dq_norm_rad_s"] == pytest.approx(0.6)


def test_csv_logger_close_drains_all_queued_rows(tmp_path) -> None:
    output = tmp_path / "logs" / "teleop.csv"
    logger = CSVLogger(output)
    logger.write_row(build_csv_row(_status()))
    logger.write_row(build_csv_row(_status()))
    logger.close()
    logger.close()

    with output.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    assert len(rows) == 2
    assert tuple(rows[0]) == CSV_FIELDNAMES
    assert rows[0]["teleop_state"] == "active"
    assert float(rows[0]["command_gripper_deg"]) == pytest.approx(26.0)
    with pytest.raises(RuntimeError, match="closed"):
        logger.write_row(build_csv_row(_status()))


def test_csv_log_cli_is_optional_path() -> None:
    parser = build_parser()
    assert parser.parse_args([]).csv_log is None
    assert str(parser.parse_args(["--csv-log", "/tmp/log.csv"]).csv_log) == (
        "/tmp/log.csv"
    )
    assert "--csv-log" in parser.format_help()

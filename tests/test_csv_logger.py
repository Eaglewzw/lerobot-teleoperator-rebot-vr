from __future__ import annotations

import csv

import numpy as np
import pytest

from lerobot_teleoperator_rebot_vr.cartesian_controller import CartesianControlStatus
from lerobot_teleoperator_rebot_vr.csv_logger import (
    CSV_FIELDNAMES,
    CSVLogger,
    LATENCY_FIELDNAMES,
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
        tracking_sample_age_ms=4.0,
        vr_decode_ms=0.2,
        latest_sample_wait_ms=1.0,
        tracking_receive_to_pickup_ms=1.2,
        feedback_read_ms=1.5,
        fk_ms=0.1,
        pose_mapping_ms=0.2,
        qp_coordination_ms=0.1,
        ik_sample_to_submit_ms=2.0,
        ik_queue_wait_ms=0.3,
        ik_worker_total_ms=1.7,
        ik_result_wait_ms=0.5,
        ik_sample_to_command_ready_ms=4.5,
        command_shaping_ms=0.2,
        controller_update_ms=2.5,
        send_action_ms=2.0,
        tracking_receive_to_send_ms=7.0,
        ik_receive_to_send_ms=6.5,
        command_to_next_feedback_ms=14.0,
        cycle_work_ms=6.0,
        ik_result_consumed_this_cycle=True,
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
    assert row["motor_control_mode"] == "pos_vel"
    assert row["mit_gravity_elbow_flex_nm"] == ""
    assert row["vr_decode_ms"] == pytest.approx(0.2)
    assert row["ik_receive_to_send_ms"] == pytest.approx(6.5)
    assert row["ik_result_consumed_this_cycle"] is True


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

    with logger.summary_path.open(newline="", encoding="utf-8") as stream:
        summary = {row["metric"]: row for row in csv.DictReader(stream)}
    assert tuple(summary) == LATENCY_FIELDNAMES
    assert int(summary["vr_decode_ms"]["samples"]) == 2
    assert float(summary["vr_decode_ms"]["mean_ms"]) == pytest.approx(0.2)
    assert float(summary["ik_receive_to_send_ms"]["p95_ms"]) == pytest.approx(6.5)


def test_csv_log_cli_is_optional_path() -> None:
    parser = build_parser()
    assert parser.parse_args([]).csv_log is None
    assert str(parser.parse_args(["--csv-log", "/tmp/log.csv"]).csv_log) == (
        "/tmp/log.csv"
    )
    assert "--csv-log" in parser.format_help()

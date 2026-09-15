from __future__ import annotations

import csv

import numpy as np
import pytest

from lerobot_teleoperator_rebot_vr.mit_tuning.analysis import TuningAnalyzer
from lerobot_teleoperator_rebot_vr.mit_tuning.cli import _validated, build_parser
from lerobot_teleoperator_rebot_vr.mit_tuning.logger import (
    CSV_FIELDS,
    TuningCSVLogger,
    build_row,
)
from lerobot_teleoperator_rebot_vr.mit_tuning.models import (
    CommandFrame,
    FeedbackFrame,
    MITTuningConfig,
    TuningSample,
)
from lerobot_teleoperator_rebot_vr.mit_tuning.profile import (
    ExcursionProfile,
    PoseTransition,
)


def _sample(
    *,
    actual_deg: float = 1.0,
    target_deg: float = 2.0,
    phase: str = "hold_positive",
    phase_progress: float = 0.75,
    elapsed_s: float = 1.0,
) -> TuningSample:
    actual = np.zeros(6)
    target = np.zeros(6)
    actual[0] = np.deg2rad(actual_deg)
    target[0] = np.deg2rad(target_deg)
    feedback = FeedbackFrame(
        monotonic_ns=123,
        position_rad=actual,
        velocity_rad_s=np.zeros(6),
        torque_nm=np.arange(6, dtype=float),
        status_code=np.ones(6, dtype=int),
        mos_temperature_c=np.full(6, 30.0),
        rotor_temperature_c=np.full(6, 31.0),
        gripper_position_deg=-100.0,
    )
    command = CommandFrame(
        requested_position_rad=target,
        sent_position_rad=target,
        desired_velocity_rad_s=np.zeros(6),
        gravity_torque_nm=np.zeros(6),
        feedforward_torque_nm=np.zeros(6),
    )
    return TuningSample(
        wall_time_ns=456,
        elapsed_s=elapsed_s,
        loop_hz=90.0,
        phase=phase,
        phase_progress=phase_progress,
        cycle=1,
        joint_index=0,
        kp=np.array([20.0, 10.0, 30.0, 10.0, 10.0, 10.0]),
        kd=np.array([5.0, 2.0, 2.0, 0.5, 0.5, 0.5]),
        feedback=feedback,
        command=command,
        feedback_read_ms=2.0,
        command_send_ms=4.0,
    )


def test_excursion_profile_is_smooth_bounded_and_returns_to_center() -> None:
    profile = ExcursionProfile(
        step_rad=0.1,
        cycles=1,
        transition_s=0.4,
        hold_s=0.2,
        warmup_s=0.1,
    )

    values = [profile.sample(float(t)) for t in np.linspace(0, profile.duration_s, 501)]

    assert max(abs(item.offset_rad) for item in values) <= 0.1 + 1e-12
    assert profile.sample(0.3).velocity_rad_s > 0.0
    assert values[-1].done
    assert values[-1].offset_rad == pytest.approx(0.0)
    assert values[-1].velocity_rad_s == pytest.approx(0.0)


def test_pose_transition_reaches_requested_pose_with_zero_terminal_velocity() -> None:
    start = np.zeros(6)
    target = np.deg2rad([0.0, -45.0, -45.0, 0.0, 0.0, 0.0])
    transition = PoseTransition(start, target, duration_s=5.0)

    initial = transition.sample(0.0)
    middle = transition.sample(2.5)
    final = transition.sample(5.0)

    assert initial.position_rad == pytest.approx(start)
    assert initial.velocity_rad_s == pytest.approx(np.zeros(6))
    assert middle.position_rad == pytest.approx(target * 0.5)
    assert final.position_rad == pytest.approx(target)
    assert final.velocity_rad_s == pytest.approx(np.zeros(6), abs=1e-12)
    assert final.done


def test_tuning_config_enforces_small_supervised_excursions() -> None:
    MITTuningConfig(
        joint_index=0,
        step_deg=90.0,
    )
    with pytest.raises(ValueError, match="90 degree"):
        MITTuningConfig(
            joint_index=0,
            step_deg=91.0,
        )
    MITTuningConfig(joint_index=1, step_deg=30.0)
    with pytest.raises(ValueError, match="30 degree"):
        MITTuningConfig(joint_index=1, step_deg=31.0)
    MITTuningConfig(joint_index=2, step_deg=30.0)
    with pytest.raises(ValueError, match="30 degree"):
        MITTuningConfig(joint_index=2, step_deg=31.0)
    MITTuningConfig(joint_index=3, step_deg=60.0)
    with pytest.raises(ValueError, match="60 degree"):
        MITTuningConfig(joint_index=3, step_deg=61.0)
    MITTuningConfig(joint_index=4, step_deg=88.0)
    with pytest.raises(ValueError, match="88 degree"):
        MITTuningConfig(joint_index=4, step_deg=89.0)
    MITTuningConfig(joint_index=5, step_deg=88.0)
    with pytest.raises(ValueError, match="88 degree"):
        MITTuningConfig(joint_index=5, step_deg=89.0)
    config = MITTuningConfig(
        joint_index=0,
        step_deg=90.0,
        max_tracking_error_deg=20.0,
    )
    assert config.max_tracking_error_deg == 20.0


def test_cli_selected_gain_only_replaces_requested_axis() -> None:
    args = build_parser().parse_args(
        ["--joint", "q5", "--kp", "12", "--kd", "1.5"]
    )

    config, kp, kd, torque = _validated(args)

    assert config.joint_index == 4
    assert kp == pytest.approx([25.0, 30.0, 30.0, 10.0, 12.0, 10.0])
    assert kd == pytest.approx([5.0, 5.0, 4.0, 0.5, 1.5, 0.5])
    assert torque == pytest.approx([27.0, 27.0, 27.0, 7.0, 7.0, 7.0])


def test_cli_accepts_automatic_preparation_pose() -> None:
    args = build_parser().parse_args(
        [
            "--joint",
            "q2",
            "--prepare-pose-deg",
            "0",
            "-45",
            "-45",
            "0",
            "0",
            "0",
        ]
    )

    config, *_ = _validated(args)

    assert config.prepare_pose_deg == pytest.approx([0, -45, -45, 0, 0, 0])


def test_tuning_csv_schema_matches_flattened_row(tmp_path) -> None:
    sample = _sample()
    row = build_row(sample)
    assert set(row) == set(CSV_FIELDS)
    assert row["actual_shoulder_pan_deg"] == pytest.approx(1.0)
    assert row["target_shoulder_pan_deg"] == pytest.approx(2.0)
    assert row["error_shoulder_pan_deg"] == pytest.approx(1.0)

    path = tmp_path / "q1.csv"
    logger = TuningCSVLogger(path)
    logger.write_sample(sample)
    logger.close()

    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    assert len(rows) == 1
    assert rows[0]["selected_joint"] == "shoulder_pan"


def test_analyzer_reports_selected_joint_tracking_metrics() -> None:
    config = MITTuningConfig(joint_index=0, settling_window_s=0.01)
    analyzer = TuningAnalyzer(config)
    analyzer.add(_sample(actual_deg=1.0, target_deg=2.0, elapsed_s=1.0))
    analyzer.add(_sample(actual_deg=2.0, target_deg=2.0, elapsed_s=1.02))
    analyzer.add(_sample(actual_deg=2.0, target_deg=2.0, elapsed_s=1.04))

    summary = analyzer.summary()

    assert summary["selected_joint"] == "shoulder_pan"
    assert summary["tracking_rmse_deg"] == pytest.approx(np.sqrt(1.0 / 3.0))
    assert summary["max_abs_error_deg"] == pytest.approx(1.0)
    assert summary["settled_holds"] == 1

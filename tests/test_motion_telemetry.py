from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from lerobot_teleoperator_rebot_vr.diagnostics.motion import cached_motor_row, mit_command_row


def test_cache_snapshot_does_not_request_or_poll_feedback():
    state = SimpleNamespace(pos=-0.117, vel=0.01, torq=3.2, status_code=1, t_mos=32., t_rotor=34.)
    motor = SimpleNamespace(get_state=lambda: state, request_feedback=Mock())
    robot = SimpleNamespace(motors={"elbow_flex": motor}, get_observation=Mock())
    row = cached_motor_row(robot)
    assert row["cached_position_elbow_flex_rad"] == -0.117
    assert row["reported_torque_elbow_flex_nm"] == 3.2
    assert row["actual_velocity_elbow_flex_rad_s"] == .01
    assert row["feedback_freshness"] == "unknown_no_receive_timestamp"
    assert "feedback_cache_error_shoulder_pan" in row
    motor.request_feedback.assert_not_called()
    robot.get_observation.assert_not_called()


def test_cache_read_failure_is_explicit_not_fabricated_zero():
    motor = SimpleNamespace(get_state=Mock(side_effect=RuntimeError("cache error")))
    row = cached_motor_row(SimpleNamespace(motors={"elbow_flex": motor}))
    assert row["feedback_cache_error_elbow_flex"] == "cache error"
    assert "reported_torque_elbow_flex_nm" not in row


def test_mit_torque_is_labelled_estimate_and_uses_radians():
    import numpy as np
    from lerobot_teleoperator_rebot_vr.control.mit import MITCommandDispatcher

    robot = object.__new__(MITCommandDispatcher)
    robot.kp = np.full(6, 30.)
    robot.kd = np.full(6, 4.)
    robot._desired_velocity_rad_s = np.zeros(6)
    robot.last_gravity_torque_nm = np.ones(6)
    robot.last_feedforward_torque_nm = np.ones(6)
    row = mit_command_row(robot, np.full(6, -6.7), np.zeros(6),
                          {"actual_velocity_elbow_flex_rad_s": .01})
    assert row["estimated_pd_ff_elbow_flex_nm"] == pytest.approx(30 * np.deg2rad(6.7) - .04 + 1.)
    assert "estimated_pd_ff_shoulder_pan_nm" not in row

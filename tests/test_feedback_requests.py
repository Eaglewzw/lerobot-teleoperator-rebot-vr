from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from lerobot_teleoperator_rebot_vr.runtime.cli import build_parser, validate_args
from lerobot_teleoperator_rebot_vr.runtime.feedback_requests import limit_startup_feedback_requests


def test_two_reads_per_cycle_share_limit_without_gating_other_operations():
    now = [0.0]
    originals = {str(i): Mock() for i in range(7)}
    robot = SimpleNamespace(motors=originals.copy())
    with limit_startup_feedback_requests(robot, 20, clock=lambda: now[0]):
        for now[0] in (0.0, 0.01, 0.03, 0.051, 0.07, 0.103):
            for motor in robot.motors.values():
                motor.request_feedback()  # get_observation
                motor.get_state()
                motor.request_feedback()  # send_action relative-target check
                motor.get_state()
                motor.send_pos_vel(0.4, 12.0)
        for motor in originals.values():
            assert motor.request_feedback.call_count == 3
            assert motor.get_state.call_count == 12
            assert motor.send_pos_vel.call_count == 6
    assert robot.motors == originals
    for motor in robot.motors.values():
        motor.request_feedback()
        motor.request_feedback()
        assert motor.request_feedback.call_count == 5


@pytest.mark.parametrize("exception", [RuntimeError, KeyboardInterrupt])
def test_exception_restores_original_handles_for_shutdown(exception):
    motor = Mock()
    robot = SimpleNamespace(motors={"q5": motor})
    with pytest.raises(exception):
        with limit_startup_feedback_requests(robot, 20):
            robot.motors["q5"].request_feedback()
            raise exception()
    assert robot.motors["q5"] is motor
    robot.motors["q5"].disable()
    motor.disable.assert_called_once()


def test_send_error_is_propagated_and_retry_not_suppressed():
    motor = Mock()
    motor.request_feedback.side_effect = [OSError("TX failed"), None]
    robot = SimpleNamespace(motors={"q5": motor})
    with limit_startup_feedback_requests(robot, 20, clock=lambda: 0.0):
        with pytest.raises(OSError):
            robot.motors["q5"].request_feedback()
        robot.motors["q5"].request_feedback()
    assert motor.request_feedback.call_count == 2


def test_zero_preserves_original_behavior():
    motor = Mock()
    robot = SimpleNamespace(motors={"q5": motor})
    with limit_startup_feedback_requests(robot, 0):
        assert robot.motors["q5"] is motor
        for _ in range(4):
            robot.motors["q5"].request_feedback()
    assert motor.request_feedback.call_count == 4


@pytest.mark.parametrize("value", ["-1", "nan", "inf"])
def test_cli_rejects_invalid_rate(value):
    with pytest.raises(ValueError, match="initial-feedback-request-hz"):
        validate_args(build_parser().parse_args(["--initial-feedback-request-hz", value]))


def test_cli_default_and_unlimited():
    assert build_parser().parse_args([]).initial_feedback_request_hz == 20
    validate_args(build_parser().parse_args(["--initial-feedback-request-hz", "0"]))

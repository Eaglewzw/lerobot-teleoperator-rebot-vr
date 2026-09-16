import json
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from lerobot_teleoperator_rebot_vr.control.types import ARM_JOINT_NAMES, GRIPPER_NAME
from lerobot_teleoperator_rebot_vr.runtime import shutdown
from lerobot_teleoperator_rebot_vr.runtime.cli import build_parser, validate_args
from lerobot_teleoperator_rebot_vr.runtime.shutdown import ManagedFollower, ShutdownFeedback, ShutdownPolicy


@pytest.fixture
def rig(monkeypatch):
    now = [100.0]
    events = []

    def sleep(duration):
        now[0] += duration

    monkeypatch.setattr(shutdown, "time", SimpleNamespace(
        monotonic=lambda: now[0], monotonic_ns=lambda: int(now[0] * 1e9), sleep=sleep))

    class Motor:
        def __init__(self, name, mid):
            self.name = name
            self.state = SimpleNamespace(can_id=mid, arbitration_id=mid + 16, status_code=1)
            self.fail = set()
            self.disable = Mock(side_effect=lambda: self.call("disable"))
            self.request_feedback = Mock(side_effect=lambda: self.call("request_feedback"))
            self.close = Mock(side_effect=lambda: self.call("close"))
            self.clear_error = Mock(side_effect=AssertionError("must not clear errors"))
            self.enable = Mock(side_effect=AssertionError("must not enable"))
            self.set_zero_position = Mock(side_effect=AssertionError("must not calibrate"))

        def call(self, operation):
            events.append((self.name, operation, now[0]))
            if operation in self.fail:
                raise OSError(f"{self.name} {operation} failed")

        def get_state(self):
            self.call("get_state")
            return self.state

    names = (*ARM_JOINT_NAMES, GRIPPER_NAME)
    ids = {name: (i, i + 16) for i, name in enumerate(names, 1)}
    motors = {name: Motor(name, pair[0]) for name, pair in ids.items()}
    bus = SimpleNamespace(
        poll_feedback_once=Mock(side_effect=lambda: events.append(("bus", "poll", now[0]))),
        close_bus=Mock(side_effect=lambda: events.append(("bus", "close_bus", now[0]))),
        close=Mock(side_effect=lambda: events.append(("bus", "free", now[0]))))
    robot = SimpleNamespace(
        config=SimpleNamespace(motor_can_ids=ids, disable_torque_on_disconnect=True),
        motors=motors.copy(), bus=bus, cameras={},
        disconnect=Mock(side_effect=AssertionError("must bypass LeRobot disconnect")),
        send_action=Mock(return_value={"wrist_yaw.pos": 0}))
    return SimpleNamespace(now=now, events=events, motors=motors, bus=bus, robot=robot)


def test_all_axes_are_spaced_retried_and_closed_after_feedback_without_clearing(rig, tmp_path, caplog):
    for motor in rig.motors.values():
        motor.state.status_code = 0
    adapter = ManagedFollower(rig.robot, report_path=tmp_path / "shutdown.json")
    result = adapter.disconnect()
    sends = [e for e in rig.events if e[1] == "disable"]
    assert [e[0] for e in sends] == list(rig.motors) * 3
    assert all(b[2] - a[2] >= 0.02 - 1e-9 for a, b in zip(sends, sends[1:]))
    assert all(r.disable_sent == r.attempts == 3 for r in result.motors)
    assert all(not r.confirmed and r.cached_status_code == 0 for r in result.motors)
    assert all("no feedback receive timestamp" in r.reason for r in result.motors)
    assert "confirmed=NO" in caplog.text
    close_index = next(i for i, e in enumerate(rig.events) if e[1] == "close_bus")
    assert all(e[1] in ("close", "free") for e in rig.events[close_index + 1:])
    assert result.close_bus_completed
    assert not result.cleanup_errors
    assert not adapter.needs_cleanup
    for motor in rig.motors.values():
        motor.clear_error.assert_not_called()
        motor.enable.assert_not_called()
        motor.set_zero_position.assert_not_called()
        motor.close.assert_called_once()
    assert json.loads((tmp_path / "shutdown.json").read_text())["motors"][0]["confirmed"] is False
    event_count = len(rig.events)
    assert adapter.disconnect() is result
    assert len(rig.events) == event_count
    with pytest.raises(RuntimeError, match="blocked"):
        adapter.send_action({})
    rig.robot.send_action.assert_not_called()
    rig.robot.disconnect.assert_not_called()


@pytest.mark.parametrize("operation", ["disable", "request_feedback", "get_state"])
def test_single_axis_io_failure_does_not_block_other_axes(rig, operation):
    rig.motors["shoulder_pan"].fail.add(operation)
    result = ManagedFollower(rig.robot).disconnect()
    assert any(operation in error for error in result.motors[0].errors)
    assert all(r.disable_sent == 3 for r in result.motors[1:])
    assert not any(r.confirmed for r in result.motors)
    rig.bus.close.assert_called_once()


def test_cleanup_errors_do_not_mask_original_failure_or_skip_remaining_resources(rig):
    rig.bus.close_bus.side_effect = OSError("bus close failed")
    rig.bus.close.side_effect = OSError("bus free failed")
    rig.motors["shoulder_pan"].fail.add("close")
    camera1 = SimpleNamespace(is_connected=True, disconnect=Mock(side_effect=OSError("camera failed")))
    camera2 = SimpleNamespace(is_connected=True, disconnect=Mock())
    rig.robot.cameras = {"one": camera1, "two": camera2}
    adapter = ManagedFollower(rig.robot)
    with pytest.raises(RuntimeError, match="startup stalled"):
        try:
            raise RuntimeError("startup stalled")
        finally:
            result = adapter.disconnect()
    assert not result.close_bus_completed
    assert len(result.cleanup_errors) == 4
    rig.motors[GRIPPER_NAME].close.assert_called_once()
    camera2.disconnect.assert_called_once()


def test_only_fresh_matching_disabled_feedback_can_confirm_and_end_retries(rig):
    def read(motor):
        motor.state.status_code = 0
        rx_ns = int(rig.now[0] * 1e9)
        if motor.name == "wrist_flex":
            rx_ns = 1  # Pre-disable cache.
        if motor.name == "wrist_yaw":
            motor.state.status_code = 1
        if motor.name == "wrist_roll":
            motor.state.can_id = 1  # Incorrect motor.
        if motor.name == GRIPPER_NAME:
            rx_ns += 1_000_000_000  # Invalid clock domain/future timestamp.
        return ShutdownFeedback(motor.get_state(), rx_ns)

    result = ManagedFollower(rig.robot, feedback_reader=read).disconnect()
    assert all(r.confirmed and r.attempts == 1 for r in result.motors[:3])
    assert all(not r.confirmed and r.attempts == 3 for r in result.motors[3:])


def test_missing_state_partial_connect_and_poll_failure_are_reported(rig):
    rig.robot.motors.pop(GRIPPER_NAME)
    rig.motors["wrist_yaw"].state = None
    rig.bus.poll_feedback_once.side_effect = OSError("poll failed")
    adapter = ManagedFollower(rig.robot)
    assert adapter.needs_cleanup
    result = adapter.disconnect()
    assert result.motors[-1].attempts == 0
    assert result.motors[-1].reason == "motor handle unavailable"
    assert result.motors[4].reason == "no state available"
    assert any("poll failed" in e for e in result.motors[0].errors)
    assert result.close_bus_completed


def test_retention_closes_transport_without_any_disable_or_feedback(rig):
    rig.robot.config.disable_torque_on_disconnect = False
    result = ManagedFollower(rig.robot).disconnect()
    assert not result.disable_requested
    assert all(r.attempts == 0 and not r.confirmed and "SKIPPED" in r.reason for r in result.motors)
    assert all(e[1] in ("close_bus", "close", "free") for e in rig.events)


@pytest.mark.parametrize("request_feedback", [True, False])
def test_request_comparison_preserves_disable_order_round_timing_and_receive(rig, request_feedback):
    report = ManagedFollower(rig.robot, policy=ShutdownPolicy(
        request_feedback=request_feedback)).disconnect()
    assert report.request_feedback is request_feedback
    sends = [e for e in rig.events if e[1] == "disable"]
    assert [e[0] for e in sends] == list(rig.motors) * 3
    assert [e[2] for e in sends] == pytest.approx([
        100.0 + round_index * 0.38 + motor_index * 0.02
        for round_index in range(3) for motor_index in range(7)])
    assert rig.now[0] == pytest.approx(101.14)
    for motor in rig.motors.values():
        assert motor.request_feedback.call_count == (3 if request_feedback else 0)
    assert rig.bus.poll_feedback_once.call_count > 0
    assert all(r.attempts == 3 and not r.confirmed for r in report.motors)


def test_shutdown_request_comparison_is_opt_in():
    assert build_parser().parse_args([]).disable_request_feedback is True
    args = build_parser().parse_args(["--no-disable-request-feedback"])
    assert args.disable_request_feedback is False
    validate_args(args)


def test_pre_disable_fault_is_preserved_when_later_status_becomes_disabled(rig):
    motor = rig.motors["wrist_yaw"]
    motor.state.status_code = 8

    def disable():
        motor.call("disable")
        motor.state.status_code = 0

    motor.disable.side_effect = disable
    result = ManagedFollower(rig.robot).disconnect().motors[4]
    assert result.observed_status_codes == [8, 0]
    assert result.cached_status_code == 0
    assert not result.confirmed


def test_report_write_failure_does_not_skip_cleanup(rig, tmp_path):
    result = ManagedFollower(rig.robot, report_path=tmp_path).disconnect()
    rig.bus.close.assert_called_once()
    assert any("write shutdown report" in e for e in result.cleanup_errors)


@pytest.mark.parametrize("option,value", [
    ("--disable-attempts", "0"), ("--disable-attempts", "11"),
    ("--disable-interval-s", "0"), ("--disable-interval-s", "nan"),
    ("--disable-feedback-wait-s", "inf"), ("--disable-feedback-wait-s", "3"),
])
def test_invalid_shutdown_configuration_rejected_before_connect(option, value):
    with pytest.raises(ValueError, match="disable-"):
        validate_args(build_parser().parse_args([option, value]))

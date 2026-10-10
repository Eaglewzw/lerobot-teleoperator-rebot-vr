"""Exercise LeRobot's real CLI/factory with a fake MotorBridge transport."""

import json

import pytest

from lerobot.robots import make_robot_from_config
from lerobot.scripts import lerobot_calibrate
from lerobot_teleoperator_rebot_vr.constants import JOINT_NAMES
from lerobot_teleoperator_rebot_vr.hardware import RebotB601RSFollowerConfig
from lerobot_teleoperator_rebot_vr.hardware.rs import RSFollower, RSFollowerConfig
from lerobot_teleoperator_rebot_vr.hardware.rs_calibration import RSCalibrationStore
from lerobot_teleoperator_rebot_vr.runtime import real
from lerobot_teleoperator_rebot_vr.runtime.cli import build_parser

from test_rs_teleop import Bus


@pytest.fixture
def hardware(monkeypatch):
    import motorbridge
    from lerobot_teleoperator_rebot_vr.hardware import rs_calibration
    buses = []
    def factory(channel):
        bus = Bus(channel)
        buses.append(bus)
        original = bus.add_robstride_motor
        def add(index, host, model):
            motor = original(index, host, model)
            def zero():
                bus.events.append((index, "zero"))
                motor.pos = 0.
            motor.set_zero_position = zero
            return motor
        bus.add_robstride_motor = add
        return bus
    monkeypatch.setattr(motorbridge, "Controller", factory)
    monkeypatch.setattr(rs_calibration.time, "sleep", lambda _: None)
    return buses


def test_standard_calibrate_cli_saves_record_without_enabling(monkeypatch, tmp_path, hardware):
    monkeypatch.setattr("sys.argv", ["lerobot-calibrate",
        "--robot.type=rebot_b601_rs_follower", "--robot.port=can0",
        "--robot.id=rebot_b601_rs_vr", f"--robot.calibration_dir={tmp_path}"])
    prompts = []
    def confirm(prompt):
        prompts.append(prompt)
        return ""
    monkeypatch.setattr("builtins.input", confirm)
    lerobot_calibrate.main()
    assert prompts == ["Press ENTER when ready..."]
    events = hardware[0].events
    assert [e[0] for e in events if e[1] == "disable"] == list(range(1, 8))
    assert [e[0] for e in events if e[1] == "zero"] == list(range(1, 8))
    assert not any(e[1] in ("enable", "mode", "mit") for e in events)
    store = RSCalibrationStore(directory=tmp_path)
    assert store.load()["zeroed_joints"] == list(JOINT_NAMES)
    assert store.load()["gripper_closed_deg"] == 0.
    assert store.load()["gripper_open_deg"] is None
    assert list(tmp_path.glob("*.jsonl"))
    # The motion follower accepts this exact record, without a manual bypass.
    robot = RSFollower(RSFollowerConfig(calibration_dir=tmp_path))
    assert robot.is_calibrated
    try:
        robot.connect()
        assert sum(e[1] == "enable" for e in hardware[-1].events) == 7
    finally:
        robot.close()


@pytest.mark.parametrize("interruption", [EOFError, KeyboardInterrupt])
def test_cancelled_initial_calibration_sends_no_motor_writes(monkeypatch, tmp_path, hardware, interruption):
    robot = make_robot_from_config(RebotB601RSFollowerConfig(calibration_dir=tmp_path))
    robot.connect(calibrate=False)
    def cancel(_):
        raise interruption()
    monkeypatch.setattr("builtins.input", cancel)
    try:
        with pytest.raises(interruption):
            robot.calibrate()
    finally:
        robot.disconnect()
    assert not robot.is_calibrated
    assert not robot.calibration_fpath.exists()
    assert not any(e[1] in ("disable", "enable", "mode", "mit", "zero") for e in hardware[0].events)


def test_reuse_existing_record_does_not_write_motors(monkeypatch, tmp_path, hardware):
    store = RSCalibrationStore(directory=tmp_path)
    store.save(dict.fromkeys(JOINT_NAMES, 0.), JOINT_NAMES)
    robot = make_robot_from_config(RebotB601RSFollowerConfig(calibration_dir=tmp_path))
    monkeypatch.setattr("builtins.input", lambda _: "")
    robot.connect(calibrate=False)
    try:
        robot.calibrate()
    finally:
        robot.disconnect()
    assert robot.is_calibrated
    assert not any(e[1] in ("disable", "enable", "mode", "mit", "zero") for e in hardware[0].events)


def test_partial_recalibration_invalidates_old_record_and_blocks_vr(monkeypatch, tmp_path, hardware):
    store = RSCalibrationStore(directory=tmp_path)
    store.save(dict.fromkeys(JOINT_NAMES, 0.), JOINT_NAMES)
    robot = make_robot_from_config(RebotB601RSFollowerConfig(calibration_dir=tmp_path))
    robot.connect(calibrate=False)
    answers = iter(["c", ""])
    monkeypatch.setattr("builtins.input", lambda _: next(answers))
    def fail():
        raise RuntimeError("lost zero ACK")
    monkeypatch.setattr(robot.motors[JOINT_NAMES[2]], "set_zero_position", fail)
    try:
        with pytest.raises(RuntimeError, match="lost zero ACK"):
            robot.calibrate()
    finally:
        robot.disconnect()
    assert not robot.is_calibrated and not robot.calibration
    assert json.loads(store.path.read_text())["status"] == "incomplete"
    monkeypatch.setattr(real, "B601Kinematics", lambda *a: pytest.fail("model opened"))
    args = build_parser().parse_args(["--robot-model", "b601_rs", "--calibration-dir", str(tmp_path)])
    with pytest.raises(ValueError, match="zeros are unconfirmed"):
        real._run_arm(args)


@pytest.mark.parametrize("change", [
    {"robot_id": "another_arm"}, {"robot_type": "rebot_b601_follower"},
    {"model_sha256": "different"}, {"schema_version": 2},
    {"zeroed_joints": ["shoulder_pan"]}, {"positions_deg": {}},
    {"positions_deg": list(JOINT_NAMES)},
    {"positions_deg": dict.fromkeys(JOINT_NAMES, float("nan"))},
])
def test_mismatched_or_corrupt_records_do_not_confirm_zero(tmp_path, change):
    store = RSCalibrationStore(directory=tmp_path)
    store.save(dict.fromkeys(JOINT_NAMES, 0.), JOINT_NAMES)
    data = json.loads(store.path.read_text())
    store.path.write_text(json.dumps({**data, **change}))
    assert not store.load()


def test_calibration_id_and_directory_match_vr_defaults(tmp_path):
    args = build_parser().parse_args(["--robot-model", "b601_rs"])
    config = RebotB601RSFollowerConfig()
    assert args.robot_id == config.id == "rebot_b601_rs_vr"
    assert args.calibration_dir == config.calibration_dir is None
    store = RSCalibrationStore(config.id, tmp_path)
    store.save(dict.fromkeys(JOINT_NAMES, 0.), JOINT_NAMES)
    assert not RSCalibrationStore("different_arm", tmp_path).load()


def test_legacy_zero_cli_writes_the_same_record(monkeypatch, tmp_path, hardware):
    from lerobot_teleoperator_rebot_vr.tools import rs_zero
    monkeypatch.setattr("sys.argv", ["rs_zero", "--execute", "--robot-id", "legacy",
        "--calibration-dir", str(tmp_path), "--output", str(tmp_path / "legacy.jsonl")])
    monkeypatch.setattr("builtins.input", lambda _: "ZERO")
    rs_zero.main()
    assert RSCalibrationStore("legacy", tmp_path).load()["zeroed_joints"] == list(JOINT_NAMES[:6])


def test_record_invalidation_failure_prevents_zero_writes(monkeypatch, tmp_path, hardware):
    robot = make_robot_from_config(RebotB601RSFollowerConfig(calibration_dir=tmp_path))
    robot.connect(calibrate=False)
    monkeypatch.setattr("builtins.input", lambda _: "")
    def fail(*args):
        raise OSError("cannot persist invalidation")
    monkeypatch.setattr(robot.calibration_store, "_write", fail)
    try:
        with pytest.raises(OSError, match="persist invalidation"):
            robot.calibrate()
    finally:
        robot.disconnect()
    assert not any(e[1] == "zero" for e in hardware[0].events)


def test_calibration_rejects_unsupported_motor_mapping():
    with pytest.raises(ValueError, match="fixed motor IDs"):
        RebotB601RSFollowerConfig(motor_can_ids={"shoulder_pan": (99, 253)})

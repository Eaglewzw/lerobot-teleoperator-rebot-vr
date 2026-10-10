"""Two actual RS runtimes with independent fake CAN buses; no physical IO."""

from dataclasses import replace
import csv
import math
from pathlib import Path
import signal
import threading
import time

import pytest
import yaml

from lerobot_teleoperator_rebot_vr.constants import JOINT_NAMES
from lerobot_teleoperator_rebot_vr.hardware.rs import RSFollower
from lerobot_teleoperator_rebot_vr.hardware.rs_calibration import RSCalibrationStore
from lerobot_teleoperator_rebot_vr.runtime.dual import DualArmSession
from lerobot_teleoperator_rebot_vr.runtime.dual_config import load_dual_config
from lerobot_teleoperator_rebot_vr.vr.xr_v1 import BimanualTrackingSource
from test_dual_teleop import packet
from test_rs_teleop import Bus

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def rs_config(tmp_path):
    config = load_dual_config(ROOT / "config/dual_rs_mit_split.yaml")
    for side, args in config.arms.items():
        args.calibration_dir = tmp_path / "calibration"
        RSCalibrationStore(args.robot_id, args.calibration_dir).save(
            dict.fromkeys(JOINT_NAMES, 0.), JOINT_NAMES)
        args.csv_log = tmp_path / f"{side}.csv"
        args.move_to_initial = False
        args.disable_attempts = 1
        args.disable_interval_s = args.disable_feedback_wait_s = .001
        args.feedback_fault_max_consecutive = 2
        args.feedback_fault_settle_time = .02
    return replace(config, duration=.20)


def test_rs_assembly_preserves_calibration_identity_and_shared_gripper_settings():
    config = load_dual_config(ROOT / "config/dual_rs_mit_split.yaml")
    left, right = config.arms["left"], config.arms["right"]
    assert (left.robot_port, right.robot_port) == ("can0", "can1")
    assert (left.robot_id, right.robot_id) == ("rebot_b601_rs_left", "rebot_b601_rs_right")
    assert left.gripper_enabled and right.gripper_enabled
    assert {k: v for k, v in vars(left).items() if k.startswith("gripper_")} == {
        k: v for k, v in vars(right).items() if k.startswith("gripper_")}
    assert (right.gripper_open_deg, right.gripper_closed_deg) == (310.12, 0.)
    assert left.exit_zero_stall_timeout is None
    assert right.exit_zero_stall_timeout == 10.
    assert left.initial_stall_timeout == right.initial_stall_timeout == 5.
    for side, args in config.arms.items():
        assert args.hand == side and args.robot_model == "b601_rs"
        assert args.move_to_initial and args.return_to_zero_on_exit
        assert args.initial_q == [0., .8, .8, 0., 0., 0.]
        assert args.exit_zero_tolerance_deg == 2.
        assert args.disable_torque_on_disconnect
        assert args.max_joint_speed_rad_s == args.wrist_speed_rad_s == 1.5
        assert args.max_joint_acceleration_rad_s2 == args.wrist_acceleration_rad_s2 == 3.
    assert left.csv_log.parent == right.csv_log.parent
    assert left.csv_log != right.csv_log


@pytest.mark.parametrize("bad", ["same_port", "same_id", "mixed", "unknown_gripper"])
def test_invalid_rs_assembly_rejected_before_hardware(tmp_path, bad):
    data = yaml.safe_load((ROOT / "config/dual_rs_mit_split.yaml").read_text())
    for arm in data["arms"].values():
        arm["control_config"] = str(ROOT / "config/rs_mit_split.yaml")
    left, right = data["arms"]["left"], data["arms"]["right"]
    if bad == "same_port":
        right["robot_port"] = left["robot_port"]
    elif bad == "same_id":
        right["robot_id"] = left["robot_id"]
    elif bad == "mixed":
        right["control_config"] = str(ROOT / "config/mit_split.yaml")
        right["overrides"] = {}
    else:
        right["overrides"]["robot"]["gripper_enabled"] = True
        right["overrides"]["gripper"] = {
            "gripper_open_deg": None, "gripper_closed_deg": None}
    path = tmp_path / "dual.yaml"
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError):
        load_dual_config(path)


@pytest.mark.parametrize("side", ["left", "right"])
def test_missing_rs_calibration_stops_both_before_source_or_workers(rs_config, side):
    args = rs_config.arms[side]
    RSCalibrationStore(args.robot_id, args.calibration_dir).invalidate()
    args.no_calibrate = True  # Must not bypass zero confirmation.
    source = BimanualTrackingSource()
    source.start = lambda: pytest.fail("VR source opened before calibration check")
    session = DualArmSession(rs_config, source)
    with pytest.raises(ValueError, match=f"{side} RS zeros are unconfirmed"):
        session.run(lambda *a, **k: pytest.fail("CAN worker started"))


@pytest.mark.parametrize("outcome", ["duration", "ctrlc", "second_ctrlc", "zero_error", "feedback", "connect", "startup"])
def test_rs_dual_real_runtimes_isolate_buses_and_cleanup(rs_config, monkeypatch, outcome):
    from lerobot_teleoperator_rebot_vr.hardware import rs
    from lerobot_teleoperator_rebot_vr.runtime import real

    # Enable both measured fake grippers to test independent Trigger velocity paths.
    for args in rs_config.arms.values():
        args.gripper_enabled = True
        args.gripper_open_deg, args.gripper_closed_deg = 310.12, 0.
        args.move_to_initial = outcome == "startup"
    buses, robots, initial_targets = {}, {}, {}
    zero_calls = set()
    zero_barrier = threading.Barrier(2)

    class DualBus(Bus):
        def __init__(self, channel):
            super().__init__("can0")
            assert channel in ("can0", "can1")
            buses[channel] = self

        def add_robstride_motor(self, index, host, model):
            motor = super().add_robstride_motor(index, host, model)
            if index == 7:
                motor.pos = math.radians(155.)
            return motor

    class Follower(RSFollower):
        def __init__(self, config):
            super().__init__(config, controller_factory=DualBus)
            self.side = "left" if config.port == "can0" else "right"
            self.reads = 0
            robots[self.side] = self

        def connect(self, **kwargs):
            if outcome == "connect" and self.side == "right":
                self.open()
                raise RuntimeError("injected RS connect failure")
            super().connect(**kwargs)

        def get_observation(self):
            self.reads += 1
            if outcome == "feedback" and self.side == "right" and self.reads > 7:
                return {}
            return super().get_observation()

    class Source(BimanualTrackingSource):
        def start(self):
            self.done = threading.Event()
            def publish():
                index, signalled = 1, False
                while not self.done.is_set():
                    # Release/rearm first, then exercise actual RS IK in both loops.
                    self.feed_bytes(packet(index, grip=0. if index < 15 else 1.))
                    with session._lock:
                        if (outcome in {"ctrlc", "second_ctrlc", "zero_error"}
                                and not signalled and session._ready == {"left", "right"}
                                and all(session._feedback[s][0] for s in session._ready)
                                and all(r.reads > 5 for r in robots.values())):
                            session.handle_signal(signal.SIGINT)
                            signalled = True
                    index += 1
                    self.done.wait(.005)
            self.thread = threading.Thread(target=publish)
            self.thread.start()

        def stop(self):
            self.done.set()
            self.thread.join(timeout=2)
            assert not self.thread.is_alive()
            super().stop()

    def initial(robot, *, target_rad, arm_id, **kwargs):
        initial_targets[arm_id] = target_rad.copy()
        return True

    def zero(robot, *, arm_id, should_stop, **kwargs):
        zero_calls.add(arm_id)
        zero_barrier.wait(timeout=3)
        if outcome in {"second_ctrlc", "zero_error"}:
            if arm_id == "left":
                if outcome == "zero_error":
                    raise RuntimeError("injected RS zero failure")
                session.handle_signal(signal.SIGINT)
            assert session._abort_return.wait(2)
            assert should_stop()
            return False
        assert not should_stop()
        return True

    monkeypatch.setattr(rs, "RSFollower", Follower)
    monkeypatch.setattr(real, "make_vr_controller", lambda *a: pytest.fail("second VR source"))
    monkeypatch.setattr(real, "_move_to_initial_pose", initial)
    monkeypatch.setattr(real, "_move_to_zero_pose", zero)
    session = DualArmSession(rs_config, Source())
    if outcome in {"connect", "zero_error"}:
        with pytest.raises(RuntimeError):
            session.run()
    else:
        session.run()
    assert robots.keys() == {"left", "right"}
    if outcome == "feedback":
        assert session._latched_fault is not None
    assert all(r.bus is None and r._reader is None for r in robots.values())
    assert zero_calls == ({"left", "right"} if outcome in {"ctrlc", "second_ctrlc", "zero_error"} else set())
    if outcome == "startup":
        assert initial_targets.keys() == {"left", "right"}
        for target in initial_targets.values():
            assert target == pytest.approx([0., .8, .8, 0., 0., 0.])  # no DM sign flip
    for side, channel in (("left", "can0"), ("right", "can1")):
        if channel not in buses:  # Peer may stop before opening during partial startup failure.
            continue
        events = buses[channel].events
        assert sum(e[1] == "disable" for e in events) == (7 if outcome == "ctrlc" else 0)
        if outcome in {"duration", "startup", "ctrlc"}:
            velocities = [e[2][1] for e in events if e[0] == 7 and e[1] == "mit"]
            assert (max(velocities) > 0.) if side == "left" else (min(velocities) < 0.)
        with rs_config.arms[side].csv_log.open() as stream:
            rows = list(csv.DictReader(stream))
        assert all(row["arm_id"] == side for row in rows if row["phase"] == "teleop")
        if outcome in {"duration", "startup"}:
            assert any(row["teleop_state"] == "active" for row in rows)

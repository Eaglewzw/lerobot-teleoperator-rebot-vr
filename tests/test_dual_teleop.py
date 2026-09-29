from __future__ import annotations

from dataclasses import replace
import json
import signal
from pathlib import Path
import threading
import time
from types import SimpleNamespace

import numpy as np
import pytest
import yaml

from lerobot_teleoperator_rebot_vr.control.controller import FullBodyQPIKController
from lerobot_teleoperator_rebot_vr.control.types import ARM_JOINT_NAMES, CartesianControlConfig
from lerobot_teleoperator_rebot_vr.runtime.dual import DualArmSession
from lerobot_teleoperator_rebot_vr.runtime.dual_config import DualConfig, load_dual_config
from lerobot_teleoperator_rebot_vr.vr.pose_mapping import RelativePoseMapper, TeleopState
from lerobot_teleoperator_rebot_vr.vr.xr_v1 import BimanualTrackingSource, CMD_FUNCTION, PacketParser


ROOT = Path(__file__).resolve().parents[1]


def packet(timestamp=1, *, left=True, right=True, grip=0.0, buttons=None):
    hands = {}
    buttons = buttons or {}
    for side, include, x in (("left", left, -1), ("right", right, 1)):
        if include:
            hands[side] = {"pose": [x, 0, 0, 0, 0, 0, 1], "grip": grip,
                           "trigger": 0.2 if side == "left" else 0.8,
                           "primaryButton": buttons.get(side, (False, False))[0],
                           "secondaryButton": buttons.get(side, (False, False))[1]}
    return PacketParser.pack(CMD_FUNCTION, json.dumps({"functionName": "Tracking",
        "value": {"timeStampNs": timestamp, "Controller": hands}}))


def test_pair_is_atomic_and_missing_hand_is_not_reused():
    source = BimanualTrackingSource()
    payload = packet()
    source.feed_bytes(payload[:10], received_monotonic_ns=100)
    assert source.latest_pair().left is None
    source.feed_bytes(payload[10:], received_monotonic_ns=200)
    old = source.latest_pair()
    assert old.left.side == "left" and old.right.side == "right"
    assert old.left.received_monotonic_ns == old.right.received_monotonic_ns == 200
    assert old.left.published_monotonic_ns == old.right.published_monotonic_ns
    assert old.left.trigger == 0.2 and old.right.trigger == 0.8
    source.feed_bytes(packet(2, left=False), received_monotonic_ns=300)
    assert source.latest_pair().left is None
    assert source.latest_pair().right.tracking_timestamp_ns == 2
    assert old.left is not None  # Previously handed-out snapshots remain immutable.
    source.feed_bytes(packet(3, left=False, right=False), received_monotonic_ns=400)
    assert source.latest_pair().right is None


def test_pair_reconnect_rollback_and_stop_clear_both_hands():
    source = BimanualTrackingSource()
    source.feed_bytes(packet(20))
    source.feed_bytes(packet(10))
    pair = source.latest_pair()
    assert pair.left.stream_epoch == pair.right.stream_epoch == 1
    source._on_disconnect()
    assert source.latest_pair().left is source.latest_pair().right is None
    source._on_connect(("localhost", 1234))
    source.feed_bytes(packet(30))
    assert source.latest_pair().left.stream_epoch == 2
    source.stop()
    assert source.latest_pair().left is source.latest_pair().right is None


def test_invalid_left_pose_does_not_discard_valid_right():
    source = BimanualTrackingSource()
    data = json.loads(PacketParser.unpack(packet())["body_str"])
    data["value"]["Controller"]["left"]["pose"] = [float("nan")] * 7
    source.feed_bytes(PacketParser.pack(CMD_FUNCTION, json.dumps(data)))
    assert source.latest_pair().left is None
    assert source.latest_pair().right is not None


@pytest.fixture
def config_file(tmp_path):
    data = yaml.safe_load((ROOT / "config/dual_mit_split.yaml").read_text())
    for side in ("left", "right"):
        data["arms"][side]["control_config"] = str(ROOT / "config/mit_split.yaml")
        data["arms"][side]["robot_port"] = f"/dev/fake_{side}"
    data["runtime"]["log_dir"] = None
    path = tmp_path / "dual.yaml"
    path.write_text(yaml.safe_dump(data))
    return path, data


def test_config_overrides_independent_and_single_arm_profile_unchanged(config_file):
    path, data = config_file
    before = (ROOT / "config/mit_split.yaml").read_bytes()
    data["arms"]["left"]["overrides"]["mit"] = {"mit_kp": [12] * 6}
    path.write_text(yaml.safe_dump(data))
    config = load_dual_config(path)
    assert config.arms["left"].mit_kp == [12] * 6
    assert config.arms["right"].mit_kp != [12] * 6
    assert config.arms["left"].hand == "left"
    assert config.arms["left"].move_to_initial
    assert config.arms["right"].move_to_initial
    assert config.arms["right"].return_to_zero_on_exit
    assert (ROOT / "config/mit_split.yaml").read_bytes() == before


def test_startup_zero_diagnostic_caps_motion_without_changing_gains_or_source_config(config_file, tmp_path):
    from lerobot_teleoperator_rebot_vr.runtime.dual_config import startup_zero_test_config

    path, _ = config_file
    original = load_dual_config(path)
    with pytest.raises(ValueError, match="log_dir"):
        startup_zero_test_config(original)
    for side, args in original.arms.items():
        args.csv_log = tmp_path / f"{side}.csv"
    before = {side: vars(args).copy() for side, args in original.arms.items()}
    result = startup_zero_test_config(original)
    for side, args in result.arms.items():
        assert args is not original.arms[side]
        assert args.max_joint_speed_rad_s <= .25
        assert args.wrist_speed_rad_s <= .25
        assert args.exit_zero_speed_rad_s <= .25
        assert args.max_joint_acceleration_rad_s2 <= .5
        assert args.exit_zero_acceleration_rad_s2 <= .5
        assert args.move_to_initial and args.return_to_zero_on_exit
        for key in ("mit_kp", "mit_kd", "mit_gravity_scale", "initial_move_tolerance_deg", "exit_zero_tolerance_deg"):
            assert getattr(args, key) == before[side][key]
        assert vars(original.arms[side]) == before[side]


@pytest.mark.parametrize("bad_feedback", [False, True])
def test_diagnostic_has_no_vr_and_requests_return_only_with_healthy_feedback(config_file, tmp_path, monkeypatch, bad_feedback):
    from lerobot_teleoperator_rebot_vr.runtime import dual

    path, _ = config_file
    config = load_dual_config(path)
    for side, args in config.arms.items():
        args.csv_log = tmp_path / f"{side}.csv"
    monkeypatch.setattr(dual, "STARTUP_ZERO_TEST_HOLD_S", .03)
    monkeypatch.setattr(dual, "BimanualTrackingSource", lambda *a, **k: pytest.fail("VR opened"))
    session = DualArmSession(config, startup_zero_test=True)
    return_requests = {}

    class Runtime:
        def __init__(self, args, *, session, arm_id):
            self.side = arm_id

        def run(self):
            session.report_feedback(self.side, not bad_feedback)
            session.arm_ready(self.side)
            while not session.should_stop():
                session.report_feedback(self.side, not bad_feedback)
                assert session.sample_for(self.side) is None
                time.sleep(.001)
            return_requests[self.side] = session.return_to_zero_requested()

    if bad_feedback:
        with pytest.raises(RuntimeError, match="not completed"):
            session.run(Runtime)
    else:
        session.run(Runtime)
    assert return_requests == {"left": not bad_feedback, "right": not bad_feedback}


def test_initial_pose_configuration_can_be_overridden_per_arm(config_file):
    path, data = config_file
    initial_q = [0.1, 0.6, 0.7, 0.3, 0.0, 0.0]
    data["arms"]["left"]["overrides"]["startup"] = {
        "move_to_initial": False, "initial_q": initial_q, "return_to_zero_on_exit": False,
    }
    path.write_text(yaml.safe_dump(data))
    config = load_dual_config(path)
    assert not config.arms["left"].move_to_initial
    assert config.arms["left"].initial_q == initial_q
    assert config.arms["right"].move_to_initial
    assert config.arms["right"].initial_q != initial_q
    assert not config.arms["left"].return_to_zero_on_exit
    assert config.arms["right"].return_to_zero_on_exit


@pytest.mark.parametrize("kind", ["same_id", "same_port", "unknown", "bad_gain",
                                  "bad_mode", "bad_bool", "reserved", "bad_hand"])
def test_bad_config_rejected_before_hardware(config_file, kind):
    path, data = config_file
    left = data["arms"]["left"]
    if kind == "same_id":
        left["robot_id"] = data["arms"]["right"]["robot_id"]
    elif kind == "same_port":
        left["robot_port"] = data["arms"]["right"]["robot_port"]
    elif kind == "bad_hand":
        left["hand"] = "right"
    else:
        left["overrides"] = {"unknown": {"typo": 1}, "bad_gain": {"mit_kp": [-1] * 6},
                             "bad_mode": {"ik_mode": "oops"}, "bad_bool": {"no_calibrate": "false"},
                             "reserved": {"robot_port": "/dev/other"}}[kind]
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError):
        load_dual_config(path)


def test_same_device_symlink_rejected(config_file, tmp_path):
    path, data = config_file
    device = tmp_path / "device"
    device.touch()
    alias = tmp_path / "alias"
    alias.symlink_to(device)
    data["arms"]["left"]["robot_port"] = str(device)
    data["arms"]["right"]["robot_port"] = str(alias)
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match="different CAN"):
        load_dual_config(path)


@pytest.fixture
def session():
    clock = [1_000_000_000]
    source = BimanualTrackingSource()
    config = DualConfig("localhost", 63901, 0, 0.5,
                        {s: SimpleNamespace(stale_timeout=0.2) for s in ("left", "right")})
    result = DualArmSession(config, source, clock_ns=lambda: clock[0])
    for side in ("left", "right"):
        result.arm_ready(side)
        result.report_feedback(side, True)
    source.feed_bytes(packet(), received_monotonic_ns=clock[0])
    return result, clock


def test_feedback_fault_blocks_both_and_requires_rearm_even_if_peer_missed_fault(session):
    session, clock = session
    mapper = RelativePoseMapper(side="right")
    def update():
        return mapper.update(session.sample_for("right"), np.zeros(3), np.eye(3), now_ns=clock[0])
    update()  # Release.
    clock[0] += 10_000_000
    session.source.feed_bytes(packet(2, grip=1), received_monotonic_ns=clock[0])
    assert update().state is TeleopState.ACTIVE
    session.report_feedback("left", False)
    assert session.sample_for("left") is None
    assert session.sample_for("right") is None
    session.report_feedback("left", True)  # Peer never ran mapper during outage.
    assert update().state is TeleopState.IDLE
    assert update().require_release
    clock[0] += 10_000_000
    session.source.feed_bytes(packet(3, grip=0), received_monotonic_ns=clock[0])
    update()
    clock[0] += 10_000_000
    session.source.feed_bytes(packet(4, grip=1), received_monotonic_ns=clock[0])
    assert update().state is TeleopState.ACTIVE


def test_missing_stale_hand_and_blocked_control_loop_hold_peer(session):
    session, clock = session
    assert session.sample_for("left") is not None
    session.source.feed_bytes(packet(2, left=False), received_monotonic_ns=clock[0])
    assert session.sample_for("right") is None
    session.source.feed_bytes(packet(3), received_monotonic_ns=clock[0])
    clock[0] += 300_000_000
    assert session.sample_for("right") is None  # Stale VR.
    clock[0] += 300_000_000
    session.source.feed_bytes(packet(4), received_monotonic_ns=clock[0])
    session.report_feedback("right", True)
    assert session.sample_for("right") is None  # Left loop has stopped reporting.
    session.report_feedback("left", True)
    assert session.sample_for("right") is not None


def test_persistent_fault_latches_and_blocks_button_commands(session):
    session, clock = session
    session.source.feed_bytes(packet(2, buttons={"left": (True, True)}),
                              received_monotonic_ns=clock[0])
    assert session.sample_for("left").primary_button
    assert session.sample_for("left").secondary_button
    session.latch_fault("left", "missing motor")
    session.report_feedback("left", True)
    assert session.sample_for("left") is session.sample_for("right") is None


def test_first_ctrl_c_requests_zero_and_arm_completion_does_not_cancel_peer(session):
    session, _ = session
    session.handle_signal(signal.SIGINT)
    assert session.should_stop() and session.return_to_zero_requested()
    session.arm_finished("left")
    assert session.return_to_zero_requested()
    session.handle_signal(signal.SIGINT)
    assert session.zero_return_aborted()
    assert not session.return_to_zero_requested()


@pytest.mark.parametrize("reason", ["feedback", "stale_feedback", "persistent", "error", "sigterm"])
def test_fault_or_termination_never_authorizes_exit_motion(session, reason):
    session, clock = session
    if reason == "feedback":
        session.report_feedback("left", False)
    elif reason == "stale_feedback":
        clock[0] += 1_000_000_000
    elif reason == "persistent":
        session.latch_fault("left", "missing motor")
    elif reason == "error":
        session.request_stop()
    elif reason == "sigterm":
        session.handle_signal(signal.SIGTERM)
    session.handle_signal(signal.SIGINT)
    assert not session.return_to_zero_requested()


def test_new_feedback_fault_during_exit_cancels_both_returns(session):
    session, _ = session
    session.handle_signal(signal.SIGINT)
    assert session.return_to_zero_requested()
    session.report_feedback("left", False)
    assert session.zero_return_aborted()


@pytest.mark.parametrize("side", ["left", "right"])
@pytest.mark.parametrize("buttons", [(True, False), (False, True), (True, True)])
def test_dual_buttons_match_single_arm_home_zero_and_do_not_cross_control(session, side, buttons):
    from test_cartesian_teleop import FakeKinematics, ImmediateIKWorker, _observation
    session, clock = session
    controllers = {s: FullBodyQPIKController(
        FakeKinematics(), hand_side=s, xr_to_base_rotation=np.eye(3),
        ik_worker=ImmediateIKWorker(), config=CartesianControlConfig()) for s in ("left", "right")}
    for s, controller in controllers.items():
        controller.update(session.sample_for(s), _observation(), .01, now_ns=clock[0])
    clock[0] += 10_000_000
    session.source.feed_bytes(packet(2, buttons={side: buttons}), received_monotonic_ns=clock[0])
    for s, controller in controllers.items():
        _, status = controller.update(session.sample_for(s), _observation(), .01, now_ns=clock[0])
        assert status.zero_requested == (s == side and buttons[1])
        assert status.home_requested == (s == side and buttons[0] and not buttons[1])
        if s == side:
            target = controller.zero_q_rad if buttons[1] else controller.home_q_rad
            assert status.target_deg[:6] == pytest.approx(np.rad2deg(target))
            if buttons[1]:
                assert status.gripper_target_deg == controller.config.gripper_closed_deg
        # Holding the button must not repeatedly reset the return trajectory.
        _, held = controller.update(session.sample_for(s), _observation(), .01, now_ns=clock[0])
        assert not held.home_requested and not held.zero_requested


def test_left_and_right_controller_state_and_gripper_are_independent():
    # Exercise the actual controller with the existing deterministic test worker.
    from test_cartesian_teleop import FakeKinematics, ImmediateIKWorker, _observation
    source = BimanualTrackingSource()
    source.feed_bytes(packet(), received_monotonic_ns=1_000_000_000)
    controllers = {side: FullBodyQPIKController(
        FakeKinematics(), hand_side=side, xr_to_base_rotation=np.eye(3),
        ik_worker=ImmediateIKWorker(), config=CartesianControlConfig()) for side in ("left", "right")}
    for side, controller in controllers.items():
        sample = replace(getattr(source.latest_pair(), side), primary_button=False)
        controller.update(sample, _observation(), .01, now_ns=1_000_000_000)
    source.feed_bytes(packet(2, grip=1), received_monotonic_ns=1_010_000_000)
    for side, controller in controllers.items():
        sample = replace(getattr(source.latest_pair(), side), primary_button=False,
                         grip=1 if side == "left" else 0)
        _, status = controller.update(sample, _observation(), .01, now_ns=1_010_000_000)
        assert status.state is (TeleopState.ACTIVE if side == "left" else TeleopState.IDLE)
    assert controllers["left"].gripper is not controllers["right"].gripper
    assert controllers["left"].worker is not controllers["right"].worker


def test_startup_wait_holds_fast_arm_until_peer_connected(config_file):
    config = load_dual_config(config_file[0])
    session = DualArmSession(config)
    held_twice = threading.Event()
    released = threading.Event()
    holds = []
    results = []
    def hold():
        holds.append(1)
        if len(holds) >= 2:
            held_twice.set()
    def left():
        results.append(session.wait_for_startup('left', hold))
        released.set()
    thread = threading.Thread(target=left)
    thread.start()
    try:
        assert held_twice.wait(1.)
        assert not released.is_set()
        assert session.wait_for_startup('right', lambda: None)
        assert released.wait(1.)
        assert results == [True]
    finally:
        session.request_stop()
        thread.join(timeout=1.)
        assert not thread.is_alive()


@pytest.mark.parametrize('signal_number', [signal.SIGINT, signal.SIGTERM, None])
def test_startup_wait_is_cancelled_when_peer_fails_or_user_stops(config_file, signal_number):
    session = DualArmSession(load_dual_config(config_file[0]))
    held = threading.Event()
    results = []
    thread = threading.Thread(target=lambda: results.append(
        session.wait_for_startup('left', held.set)))
    thread.start()
    try:
        assert held.wait(1.)
        if signal_number is None:
            session.request_stop()
        else:
            session.handle_signal(signal_number)
        thread.join(timeout=1.)
        assert not thread.is_alive()
        assert results == [False]
        assert not session.wait_for_startup('right', lambda: pytest.fail('hold after stop'))
    finally:
        session.request_stop()
        thread.join(timeout=1.)


def test_startup_wait_timeout_stops_both(config_file):
    config = load_dual_config(config_file[0])
    config.arms['left'].initial_move_timeout = .02
    session = DualArmSession(config)
    with pytest.raises(RuntimeError, match='waiting for both arms'):
        session.wait_for_startup('left', lambda: None)
    assert session.should_stop()
    assert not session.wait_for_startup('right', lambda: pytest.fail('hold after timeout'))


@pytest.mark.parametrize("failure", ["startup", "runtime", "none"])
def test_session_cleans_both_workers_and_one_shared_source(config_file, failure):
    path, _ = config_file
    config = replace(load_dual_config(path), duration=0.03)
    source = BimanualTrackingSource()
    events = []
    source.start = lambda: events.append("source_start")
    source.stop = lambda: events.append("source_stop")
    session = DualArmSession(config, source)
    both_started = threading.Barrier(2)
    class Runtime:
        def __init__(self, args, *, session, arm_id):
            self.session, self.side = session, arm_id
        def run(self):
            try:
                events.append(self.side + "_start")
                both_started.wait(timeout=2)
                if failure == "startup" and self.side == "left":
                    raise RuntimeError("startup failure")
                self.session.arm_ready(self.side)
                if failure == "runtime" and self.side == "left":
                    raise RuntimeError("runtime failure")
                while not self.session.should_stop():
                    self.session.report_feedback(self.side, True)
                    time.sleep(.001)
            finally:
                events.append(self.side + "_cleanup")
    if failure == "none":
        session.run(Runtime)
    else:
        with pytest.raises(RuntimeError, match=failure + " failure"):
            session.run(Runtime)
    assert events.count("source_start") == events.count("source_stop") == 1
    assert "left_cleanup" in events and "right_cleanup" in events
    assert events[-1] == "source_stop"


@pytest.mark.parametrize("mode,failure", [("pos_vel", None), ("mit", None),
                                         ("pos_vel", "connect"), ("mit", "feedback"),
                                         ("pos_vel", "send"), ("pos_vel", "initial"),
                                         ("mit", "initial"), ("mit", "initial_error"),
                                         ("pos_vel", "initial_stop"),
                                         ("pos_vel", "ctrlc"), ("mit", "ctrlc"),
                                         ("mit", "ctrlc_second"), ("mit", "zero_error"),
                                         ("pos_vel", "zero_disabled"), ("mit", "sigterm"),
                                         ("mit", "initial_ctrlc")])
def test_real_dual_loops_with_fake_can(config_file, monkeypatch, mode, failure):
    """Run actual IK, MIT, controller, lifecycle and shutdown without hardware."""
    from lerobot.robots import rebot_b601_follower as follower_module
    from lerobot_teleoperator_rebot_vr.runtime import real
    from test_mit_control import _FakeMotor

    path, data = config_file
    data["runtime"]["duration"] = 0.18
    test_initial = failure in ("initial", "initial_error", "initial_stop", "initial_ctrlc")
    test_zero = failure in ("ctrlc", "ctrlc_second", "zero_error", "zero_disabled", "initial_ctrlc")
    for side in ("left", "right"):
        data["arms"][side]["control_config"] = str(ROOT / f"config/{mode}_split.yaml")
        data["arms"][side]["overrides"].update({
            "disable_attempts": 1, "disable_interval_s": .001, "disable_feedback_wait_s": .001,
            "feedback_fault_max_consecutive": 2,
            "move_to_initial": test_initial,
        })
    if test_initial:
        data["arms"]["left"]["overrides"]["initial_q"] = [0.1, 0.6, 0.7, 0.1, 0.1, 0.1]
    if failure == "zero_disabled":
        data["arms"]["left"]["overrides"]["return_to_zero_on_exit"] = False
    path.write_text(yaml.safe_dump(data))
    config = load_dual_config(path)
    robots = {}
    barrier = threading.Barrier(2)
    initial_barrier = threading.Barrier(2)
    initial_targets = {}
    zero_calls = set()
    zero_barrier = threading.Barrier(2)
    class Robot:
        def __init__(self, robot_config):
            self.config = robot_config
            self.side = "left" if robot_config.id == "rebot_left" else "right"
            self.bus = None
            self.motors = {}
            self.motor_handles = {}
            self.motor_names = [*ARM_JOINT_NAMES, "gripper"]
            self.cameras = {}
            self.is_connected = False
            self.is_calibrated = True
            self.closed = False
            self.reads = 0
            self.actions = []
            self.disable_count = 0
            robots[self.side] = self
        def connect(self, **kwargs):
            self.bus = SimpleNamespace(poll_feedback_once=lambda: None,
                                       close_bus=self.close, close=self.close)
            for name, (send_id, recv_id) in self.config.motor_can_ids.items():
                motor = _FakeMotor()
                state = SimpleNamespace(can_id=send_id, arbitration_id=recv_id, status_code=0)
                motor.get_state = lambda state=state: state
                motor.request_feedback = lambda: None
                motor.disable = self.disable
                motor.close = lambda: None
                self.motors[name] = motor
                self.motor_handles[name] = motor
            barrier.wait(timeout=3)
            if failure == "connect" and self.side == "left":
                raise RuntimeError("injected partial connect failure")
            self.is_connected = True
        def close(self):
            self.closed = True
        def disable(self):
            self.disable_count += 1
        def get_observation(self):
            self.reads += 1
            if failure == "feedback" and self.side == "left" and self.reads > 3:
                return {}
            return {**{f"{name}.pos": q for name, q in zip(ARM_JOINT_NAMES,
                     [0, -60, -70, 0, 0, 0])}, "gripper.pos": -90.0}
        def send_action(self, action):
            if failure == "send" and self.side == "left":
                raise RuntimeError("injected send failure")
            self.actions.append(action.copy())
            return action

    class Source(BimanualTrackingSource):
        def start(self):
            self.done = threading.Event()
            def publish():
                index = 1
                signalled = False
                while not self.done.is_set():
                    self.feed_bytes(packet(index, grip=0 if index < 15 else 1))
                    if failure in ("ctrlc", "ctrlc_second", "zero_error", "zero_disabled", "sigterm"):
                        with session._lock:
                            if (not signalled and session._ready == {"left", "right"}
                                    and all(session._feedback[s][0] for s in session._ready)
                                    and all(r.reads >= 3 for r in robots.values())):
                                session.handle_signal(signal.SIGTERM if failure == "sigterm" else signal.SIGINT)
                                signalled = True
                    index += 1
                    self.done.wait(.005)
            self.producer = threading.Thread(target=publish)
            self.producer.start()
        def stop(self):
            self.done.set()
            self.producer.join(timeout=2)
            super().stop()

    monkeypatch.setattr(follower_module, "RebotB601Follower", Robot)
    def initial(robot, *, target_rad, should_stop, **kwargs):
        assert test_initial
        side = "left" if robot.config.id == "rebot_left" else "right"
        assert side not in session._ready  # Never expose VR before initialization completes.
        initial_targets[side] = target_rad.copy()
        initial_barrier.wait(timeout=3)
        if failure == "initial_error":
            if side == "left":
                raise RuntimeError("injected initial motion failure")
            assert session._stop.wait(2)
            assert should_stop()  # Peer failure reaches the in-progress startup loop.
            return False
        if failure == "initial_stop":
            session.request_stop()
            assert should_stop()
            return False
        if failure == "initial_ctrlc":
            if side == "left":
                session.handle_signal(signal.SIGINT)
            assert session._stop.wait(2)
            assert should_stop()
            return False
        assert not should_stop()
        return True

    # There must be only one VR server and startup remains per-arm configurable.
    monkeypatch.setattr(real, "make_vr_controller", lambda *_: pytest.fail("second VR server"))
    monkeypatch.setattr(real, "_move_to_initial_pose", initial)
    def zero(robot, *, should_stop, **kwargs):
        assert test_zero
        side = "left" if robot.config.id == "rebot_left" else "right"
        assert not robots[side].closed
        zero_calls.add(side)
        if failure != "zero_disabled":
            zero_barrier.wait(timeout=3)
        if failure in ("ctrlc_second", "zero_error"):
            if side == "left":
                if failure == "zero_error":
                    raise RuntimeError("injected zero failure")
                session.handle_signal(signal.SIGINT)
            deadline = time.monotonic() + 2
            while not should_stop() and time.monotonic() < deadline:
                time.sleep(.001)
            assert should_stop()
            return False
        assert not should_stop()
        return True

    monkeypatch.setattr(real, "_move_to_zero_pose", zero)
    session = DualArmSession(config, Source())
    if failure in ("connect", "send", "initial_error", "zero_error"):
        with pytest.raises(RuntimeError, match="injected"):
            session.run()
    else:
        session.run()
    assert set(robots) == {"left", "right"}
    assert all(robot.closed for robot in robots.values())
    assert robots["left"].motors is not robots["right"].motors
    assert zero_calls == ({"right"} if failure == "zero_disabled" else
                          {"left", "right"} if test_zero else set())
    if test_initial:
        assert set(initial_targets) == {"left", "right"}
        for side, target in initial_targets.items():
            expected = np.array(config.arms[side].initial_q, dtype=float)
            expected[1:3] *= -1  # Same RS-to-DM convention as single-arm startup.
            assert target == pytest.approx(expected)
    else:
        assert initial_targets == {}
    if failure is None or failure == "initial":
        assert all(robot.reads > 1 for robot in robots.values())
        if mode == "mit":
            assert all(robot.motor_handles["wrist_roll"].mit_calls for robot in robots.values())
        else:
            assert all(robot.actions for robot in robots.values())
    if failure == "feedback":
        assert session._latched_fault is not None
        assert robots["left"].disable_count == 0  # Existing fault retention policy.
        assert robots["right"].disable_count > 0


def test_dual_missing_calibration_does_not_open_can_or_prompt(config_file, monkeypatch):
    from lerobot.robots import rebot_b601_follower as follower_module
    from lerobot_teleoperator_rebot_vr.runtime.real import ArmRuntime
    path, _ = config_file
    config = load_dual_config(path)
    config.arms["left"].motor_control_mode = "pos_vel"
    session = DualArmSession(config, BimanualTrackingSource())
    class UncalibratedRobot:
        def __init__(self, config):
            self.config = config
            self.is_calibrated = False
            self.bus, self.motors, self.cameras = None, {}, {}
        def connect(self, **kwargs):
            pytest.fail("must not enter interactive calibration or open CAN")
    monkeypatch.setattr(follower_module, "RebotB601Follower", UncalibratedRobot)
    with pytest.raises(RuntimeError, match="no calibration"):
        ArmRuntime(config.arms["left"], session=session, arm_id="left").run()
    assert session.should_stop()

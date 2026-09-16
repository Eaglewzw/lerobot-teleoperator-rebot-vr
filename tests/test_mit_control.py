from __future__ import annotations

from types import SimpleNamespace

import numpy as np
import pytest

from lerobot_teleoperator_rebot_vr.control.dynamics import B601GravityCompensator
from lerobot_teleoperator_rebot_vr.control.mit import MITCommandDispatcher
from lerobot_teleoperator_rebot_vr.control.types import ARM_JOINT_NAMES, GRIPPER_NAME
from lerobot_teleoperator_rebot_vr.ik.kinematics import B601Kinematics
from lerobot_teleoperator_rebot_vr.runtime.cli import build_parser, validate_args


class _FakeMotor:
    def __init__(self) -> None:
        self.mit_calls: list[tuple[float, float, float, float, float]] = []
        self.force_pos_calls: list[tuple[float, float, float]] = []

    def send_mit(
        self, position: float, velocity: float, kp: float, kd: float, torque: float
    ) -> None:
        self.mit_calls.append((position, velocity, kp, kd, torque))

    def send_force_pos(self, position: float, velocity: float, ratio: float) -> None:
        self.force_pos_calls.append((position, velocity, ratio))


class _FakeRobot:
    def __init__(self, max_relative_target=None, joint_limits=None) -> None:
        names = [*ARM_JOINT_NAMES, GRIPPER_NAME]
        self.motor_names = names
        self.motors = {name: _FakeMotor() for name in names}
        self.config = SimpleNamespace(
            joint_limits=(
                {name: (-270.0, 270.0) for name in names}
                if joint_limits is None
                else joint_limits
            ),
            max_relative_target=max_relative_target,
            gripper_control_mode="force_pos",
            gripper_mit_kp=8.0,
            gripper_mit_kd=0.3,
            pos_vel_velocity=[100.0] * 6 + [900.0],
            gripper_torque_ratio=0.2,
        )
        self.observation = {f"{name}.pos": 0.0 for name in names}

    def get_observation(self):
        return dict(self.observation)


def _dispatcher(
    robot: _FakeRobot,
    *,
    acceleration_limit_rad_s2: np.ndarray | None = None,
    position_lookahead_s: np.ndarray | None = None,
    velocity_aligned_axes: np.ndarray | None = None,
    joint_limit_margin_rad: float = 0.0,
) -> MITCommandDispatcher:
    return MITCommandDispatcher(
        robot,
        kp=np.array([45.0, 44.0, 43.0, 8.0, 7.0, 6.0]),
        kd=np.array([3.0, 2.5, 2.0, 1.0, 0.8, 0.6]),
        torque_limit_nm=np.array([12.0, 12.0, 12.0, 3.0, 3.0, 2.0]),
        arm_velocity_limit_rad_s=np.array([1.0, 1.0, 1.0, 2.0, 2.0, 2.0]),
        arm_acceleration_limit_rad_s2=acceleration_limit_rad_s2,
        position_lookahead_s=position_lookahead_s,
        velocity_aligned_axes=velocity_aligned_axes,
        joint_limit_margin_rad=joint_limit_margin_rad,
        gravity_ramp_s=0.0,
    )


def test_mit_trajectory_velocity_preserves_position_and_stops_axes(monkeypatch):
    from lerobot_teleoperator_rebot_vr.control import mit

    now = [10.0]
    monkeypatch.setattr(mit, "time", SimpleNamespace(monotonic=lambda: now[0]))
    robot = _FakeRobot(max_relative_target=10.0)
    dispatcher = _dispatcher(
        robot, acceleration_limit_rad_s2=np.ones(6),
        position_lookahead_s=np.full(6, 0.05),
    )
    dispatcher.get_observation()
    action = {f"{name}.pos": 5.0 for name in [*ARM_JOINT_NAMES, GRIPPER_NAME]}
    dispatcher.set_arm_trajectory_velocity(np.ones(6))
    dispatcher.send_action(action)
    now[0] += 0.02
    dispatcher.send_action(action)
    position, velocity, *_ = robot.motors[ARM_JOINT_NAMES[0]].mit_calls[-1]
    assert position == pytest.approx(np.deg2rad(5.0))  # Not actual + dq * lookahead.
    assert velocity == pytest.approx(0.02)  # Final acceleration limit still applies.
    dispatcher.set_arm_trajectory_velocity(np.zeros(6))
    dispatcher.send_action(action)
    assert all(robot.motors[name].mit_calls[-1][1] == 0.0 for name in ARM_JOINT_NAMES)

    dispatcher.set_arm_velocity(np.ones(6))
    now[0] += 0.02
    dispatcher.send_action(action)
    position, velocity, *_ = robot.motors[ARM_JOINT_NAMES[0]].mit_calls[-1]
    assert position == pytest.approx(velocity * 0.05)  # ACTIVE behavior restored.


def test_mit_trajectory_feedforward_brakes_at_hard_limit(monkeypatch):
    from lerobot_teleoperator_rebot_vr.control import mit

    now = [10.0]
    monkeypatch.setattr(mit, "time", SimpleNamespace(monotonic=lambda: now[0]))
    robot = _FakeRobot(joint_limits={
        name: (-180.0, 0.0) for name in [*ARM_JOINT_NAMES, GRIPPER_NAME]
    })
    robot.observation.update({f"{name}.pos": np.rad2deg(-0.0001) for name in ARM_JOINT_NAMES})
    dispatcher = _dispatcher(robot, acceleration_limit_rad_s2=np.ones(6))
    dispatcher.get_observation()
    action = {f"{name}.pos": 0.0 for name in [*ARM_JOINT_NAMES, GRIPPER_NAME]}
    dispatcher.set_arm_trajectory_velocity(np.ones(6))
    dispatcher.send_action(action)
    now[0] += 0.05
    dispatcher.send_action(action)
    assert robot.motors[ARM_JOINT_NAMES[0]].mit_calls[-1][1] == pytest.approx(np.sqrt(0.0002))
    robot.observation.update({f"{name}.pos": 0.0 for name in ARM_JOINT_NAMES})
    dispatcher.get_observation()
    now[0] += 0.05
    dispatcher.send_action(action)
    assert all(robot.motors[name].mit_calls[-1][1] == 0.0 for name in ARM_JOINT_NAMES)


@pytest.mark.parametrize("phase", ["initial", "zero"])
@pytest.mark.parametrize("outcome", ["reached", "cancelled", "stall", "timeout", "bad_feedback"])
def test_mit_pose_trajectory_sends_velocity_and_clears_it_on_exit(monkeypatch, outcome, phase):
    from lerobot_teleoperator_rebot_vr.control import mit
    from lerobot_teleoperator_rebot_vr.runtime import safety

    now = [10.0]
    clock = SimpleNamespace(
        monotonic=lambda: now[0], sleep=lambda dt: now.__setitem__(0, now[0] + dt)
    )
    monkeypatch.setattr(mit, "time", clock)
    monkeypatch.setattr(safety, "time", clock)
    robot = _FakeRobot(max_relative_target=10.0)
    if phase == "zero":
        robot.observation.update({f"{name}.pos": np.rad2deg(0.2) for name in ARM_JOINT_NAMES})
    dispatcher = _dispatcher(
        robot, acceleration_limit_rad_s2=np.ones(6),
        position_lookahead_s=np.full(6, 0.05),
    )

    def feedback():
        calls = robot.motors[ARM_JOINT_NAMES[0]].mit_calls
        if outcome == "reached" and calls:
            for name in ARM_JOINT_NAMES:
                robot.observation[f"{name}.pos"] = np.rad2deg(robot.motors[name].mit_calls[-1][0])
        if outcome == "bad_feedback" and len(calls) >= 5:
            robot.observation["shoulder_pan.pos"] = float("nan")
        return dict(robot.observation)

    robot.get_observation = feedback
    args = SimpleNamespace(
        max_joint_speed_rad_s=0.4, max_joint_acceleration_rad_s2=1.0,
        initial_move_tolerance_deg=0.05, max_relative_target_deg=10.0,
        initial_move_timeout=0.1 if outcome == "timeout" else 5.0,
        initial_stall_timeout=0.1 if outcome == "stall" else 3.0,
        status_rate=1.0, fps=100.0,
        exit_zero_speed_rad_s=0.4, exit_zero_acceleration_rad_s2=1.0,
        wrist_speed_rad_s=None, wrist_acceleration_rad_s2=None, wrist_relative_target_deg=None,
    )

    def run():
        move = safety.move_to_initial_pose if phase == "initial" else safety.move_to_zero_pose
        target = {"target_rad": np.full(6, 0.2)} if phase == "initial" else {}
        return move(
            dispatcher, **target,
            lower_limit_rad=np.full(6, -3.0), upper_limit_rad=np.full(6, 3.0),
            args=args, should_stop=lambda: outcome == "cancelled" and now[0] >= 10.1,
        )

    if outcome in ("stall", "timeout", "bad_feedback"):
        with pytest.raises(RuntimeError, match={
            "stall": "not following", "timeout": "timed out", "bad_feedback": "invalid"
        }[outcome]):
            run()
    else:
        assert run() == (outcome == "reached")
    calls = robot.motors[ARM_JOINT_NAMES[0]].mit_calls
    assert any(abs(call[1]) > 0.0 for call in calls)
    assert max(abs(call[1]) for call in calls) <= 0.4
    assert dispatcher.desired_velocity_rad_s == pytest.approx(np.zeros(6))
    assert dispatcher.target_velocity_rad_s == pytest.approx(np.zeros(6))
    if outcome == "reached":
        assert calls[-1][1] == 0.0


def test_packaged_dynamics_model_has_six_joints_and_finite_gravity() -> None:
    gravity = B601GravityCompensator()
    torque = gravity.gravity_torque(np.array([0.0, -0.8, -0.8, 0.0, 0.0, 0.0]))

    assert gravity.model.nq == 6
    assert gravity.model.nv == 6
    assert torque == pytest.approx(
        [0.0, 1.191932, -6.971919, -1.888309, 0.000005, 0.000321],
        abs=2e-6,
    )


def test_dynamics_model_matches_production_tcp_kinematics() -> None:
    pin = pytest.importorskip("pinocchio")
    kinematics = B601Kinematics()
    gravity = B601GravityCompensator()
    frame_id = gravity.model.getFrameId("gripper_end")
    data = gravity.model.createData()
    try:
        for q_rad in (
            np.zeros(6),
            np.array([0.3, -0.8, -1.1, 0.2, -0.4, 0.5]),
            np.array([-1.2, -1.6, -0.3, -0.7, 0.6, -1.0]),
        ):
            expected_position, expected_rotation = kinematics.forward_kinematics(
                q_rad
            )
            pin.framesForwardKinematics(gravity.model, data, q_rad)
            actual = data.oMf[frame_id]
            assert actual.translation == pytest.approx(expected_position, abs=1e-12)
            assert actual.rotation == pytest.approx(expected_rotation, abs=1e-12)
    finally:
        kinematics.close()


def test_mit_dispatcher_sends_six_axis_velocity_and_gravity_but_force_pos_gripper() -> None:
    robot = _FakeRobot()
    dispatcher = _dispatcher(robot)
    observation = dispatcher.get_observation()
    dispatcher.set_observation(observation)
    dispatcher.set_arm_velocity(np.array([4.0, -4.0, 0.5, 5.0, -5.0, 1.0]))
    action = {f"{name}.pos": 0.0 for name in (*ARM_JOINT_NAMES, GRIPPER_NAME)}

    sent = dispatcher.send_action(action)

    expected_velocity = [1.0, -1.0, 0.5, 2.0, -2.0, 1.0]
    expected_gravity = dispatcher.gravity.gravity_torque(np.zeros(6))
    expected_torque = np.clip(
        expected_gravity,
        -dispatcher.torque_limit_nm,
        dispatcher.torque_limit_nm,
    )
    for index, name in enumerate(ARM_JOINT_NAMES):
        assert len(robot.motors[name].mit_calls) == 1
        position, velocity, kp, kd, torque = robot.motors[name].mit_calls[0]
        assert position == pytest.approx(0.0)
        assert velocity == pytest.approx(expected_velocity[index])
        assert kp == pytest.approx(dispatcher.kp[index])
        assert kd == pytest.approx(dispatcher.kd[index])
        assert torque == pytest.approx(expected_torque[index])
    assert robot.motors[GRIPPER_NAME].mit_calls == []
    assert len(robot.motors[GRIPPER_NAME].force_pos_calls) == 1
    assert sent == action


def test_mit_dispatcher_preserves_relative_target_limit() -> None:
    robot = _FakeRobot(max_relative_target=5.0)
    dispatcher = _dispatcher(robot)
    dispatcher.get_observation()
    action = {f"{name}.pos": 20.0 for name in (*ARM_JOINT_NAMES, GRIPPER_NAME)}

    sent = dispatcher.send_action(action)

    assert all(value == pytest.approx(5.0) for value in sent.values())
    for name in ARM_JOINT_NAMES:
        assert robot.motors[name].mit_calls[0][0] == pytest.approx(np.deg2rad(5.0))


def test_mit_velocity_target_uses_qp_velocity_directly() -> None:
    dispatcher = _dispatcher(_FakeRobot())
    qp_velocity = np.array([0.4, -0.3, 0.2, 0.8, -0.6, 0.4])

    dispatcher.set_arm_velocity(qp_velocity)

    assert dispatcher.target_velocity_rad_s == pytest.approx(qp_velocity)
    assert dispatcher.desired_velocity_rad_s == pytest.approx(qp_velocity)


def test_mit_final_velocity_obeys_acceleration_limit(monkeypatch) -> None:
    now_s = [10.0]
    monkeypatch.setattr(
        "lerobot_teleoperator_rebot_vr.control.mit.time.monotonic",
        lambda: now_s[0],
    )
    acceleration = np.array([2.0, 2.0, 2.0, 4.0, 4.0, 4.0])
    lookahead = np.array([0.05, 0.05, 0.05, 0.025, 0.025, 0.025])
    robot = _FakeRobot()
    dispatcher = _dispatcher(
        robot,
        acceleration_limit_rad_s2=acceleration,
        position_lookahead_s=lookahead,
    )
    dispatcher.get_observation()
    action = {f"{name}.pos": 0.0 for name in (*ARM_JOINT_NAMES, GRIPPER_NAME)}
    dispatcher.send_action(action)

    dispatcher.set_arm_velocity(np.ones(6))
    now_s[0] += 0.02
    dispatcher.send_action(action)
    assert dispatcher.desired_velocity_rad_s == pytest.approx(
        [0.04, 0.04, 0.04, 0.08, 0.08, 0.08]
    )
    for name in ARM_JOINT_NAMES:
        assert robot.motors[name].mit_calls[-1][0] == pytest.approx(0.002)

    dispatcher.stop_arm_velocity(immediate=False)
    now_s[0] += 0.01
    dispatcher.send_action(action)
    assert dispatcher.desired_velocity_rad_s == pytest.approx(
        [0.02, 0.02, 0.02, 0.04, 0.04, 0.04]
    )
    for name in ARM_JOINT_NAMES:
        assert robot.motors[name].mit_calls[-1][0] == pytest.approx(0.001)


def test_mit_final_velocity_reserves_braking_distance_without_margin_jump(
    monkeypatch,
) -> None:
    now_s = [10.0]
    monkeypatch.setattr(
        "lerobot_teleoperator_rebot_vr.control.mit.time.monotonic",
        lambda: now_s[0],
    )
    names = [*ARM_JOINT_NAMES, GRIPPER_NAME]
    limits = {name: (-270.0, 270.0) for name in names}
    limits["wrist_flex"] = (-80.0, 90.0)
    robot = _FakeRobot(joint_limits=limits)
    robot.observation["wrist_flex.pos"] = -75.2
    acceleration = np.full(6, 8.0)
    dispatcher = _dispatcher(
        robot,
        acceleration_limit_rad_s2=acceleration,
        position_lookahead_s=np.full(6, 0.015),
        joint_limit_margin_rad=np.deg2rad(2.0),
    )
    dispatcher.get_observation()
    action = {
        f"{name}.pos": float(robot.observation[f"{name}.pos"])
        for name in names
    }
    dispatcher.send_action(action)
    velocity = np.zeros(6)
    velocity[3] = -3.0
    dispatcher.set_arm_velocity(velocity)
    for _ in range(10):
        now_s[0] += 0.05
        dispatcher.send_action(action)

    braking_speed = np.sqrt(2.0 * 8.0 * np.deg2rad(2.8))
    position, sent_velocity, *_ = robot.motors["wrist_flex"].mit_calls[-1]
    assert sent_velocity == pytest.approx(-braking_speed)
    assert position == pytest.approx(
        np.deg2rad(-75.2) - braking_speed * 0.015
    )

    robot.observation["wrist_flex.pos"] = -79.0
    dispatcher.set_observation(robot.observation)
    now_s[0] += 0.05
    dispatcher.send_action(action)
    position, sent_velocity, *_ = robot.motors["wrist_flex"].mit_calls[-1]
    assert sent_velocity == pytest.approx(0.0)
    assert position == pytest.approx(np.deg2rad(-79.0))


def test_mit_immediate_stop_restores_explicit_position_command() -> None:
    robot = _FakeRobot()
    dispatcher = _dispatcher(
        robot,
        position_lookahead_s=np.full(6, 0.05, dtype=np.float64),
    )
    dispatcher.get_observation()
    dispatcher.set_arm_velocity(np.ones(6))
    dispatcher.stop_arm_velocity(immediate=True)
    action = {f"{name}.pos": 10.0 for name in (*ARM_JOINT_NAMES, GRIPPER_NAME)}

    dispatcher.send_action(action)

    for name in ARM_JOINT_NAMES:
        assert robot.motors[name].mit_calls[-1][0] == pytest.approx(np.deg2rad(10.0))


def test_mit_position_mode_alignment_preserves_explicit_wrist_hold() -> None:
    robot = _FakeRobot()
    dispatcher = _dispatcher(
        robot,
        position_lookahead_s=np.full(6, 0.05, dtype=np.float64),
        velocity_aligned_axes=np.array([True, True, True, False, False, False]),
    )
    dispatcher.get_observation()
    dispatcher.set_arm_velocity(np.array([0.5, 0.0, 0.0, 0.0, 0.0, 0.0]))
    action = {f"{name}.pos": 10.0 for name in (*ARM_JOINT_NAMES, GRIPPER_NAME)}

    dispatcher.send_action(action)

    assert robot.motors["shoulder_pan"].mit_calls[-1][0] == pytest.approx(0.025)
    for name in ARM_JOINT_NAMES[3:]:
        assert robot.motors[name].mit_calls[-1][0] == pytest.approx(np.deg2rad(10.0))


def test_mit_velocity_survives_short_result_gap_then_expires(monkeypatch) -> None:
    now_s = [20.0]
    monkeypatch.setattr(
        "lerobot_teleoperator_rebot_vr.control.mit.time.monotonic",
        lambda: now_s[0],
    )
    dispatcher = _dispatcher(_FakeRobot())
    velocity = np.array([0.4, -0.3, 0.2, 0.8, -0.6, 0.4])
    dispatcher.set_arm_velocity(velocity)

    now_s[0] += 0.02
    assert not dispatcher.stop_stale_arm_velocity(0.03)
    assert dispatcher.target_velocity_rad_s == pytest.approx(velocity)

    now_s[0] += 0.02
    assert dispatcher.stop_stale_arm_velocity(0.03)
    assert dispatcher.target_velocity_rad_s == pytest.approx(np.zeros(6))


def test_mit_cli_defaults_to_pos_vel_and_validates_protocol_ranges() -> None:
    parser = build_parser()
    defaults = parser.parse_args([])
    mit_defaults = parser.parse_args(["--motor-control-mode", "mit"])

    assert defaults.motor_control_mode == "pos_vel"
    assert mit_defaults.mit_kp == pytest.approx([25.0, 30.0, 30.0, 10.0, 10.0, 10.0])
    assert mit_defaults.mit_kd == pytest.approx([5.0, 5.0, 4.0, 0.5, 0.5, 0.5])
    assert mit_defaults.mit_torque_limit_nm == pytest.approx(
        [27.0, 27.0, 27.0, 7.0, 7.0, 7.0]
    )
    assert mit_defaults.mit_gravity_scale == pytest.approx(1.0)
    assert mit_defaults.mit_gravity_ramp_s == pytest.approx(1.5)
    validate_args(defaults)
    validate_args(mit_defaults)

    invalid_kd = parser.parse_args(["--mit-kd", "6", "1", "1", "1", "1", "1"])
    with pytest.raises(ValueError, match="MIT Kd"):
        validate_args(invalid_kd)

    invalid_torque = parser.parse_args(
        ["--mit-torque-limit-nm", "28", "12", "12", "3", "3", "2"]
    )
    with pytest.raises(ValueError, match="MIT torque limits"):
        validate_args(invalid_torque)

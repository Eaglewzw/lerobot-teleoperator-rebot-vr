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
    def __init__(self, max_relative_target=None) -> None:
        names = [*ARM_JOINT_NAMES, GRIPPER_NAME]
        self.motor_names = names
        self.motors = {name: _FakeMotor() for name in names}
        self.config = SimpleNamespace(
            joint_limits={name: (-270.0, 270.0) for name in names},
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


def _dispatcher(robot: _FakeRobot) -> MITCommandDispatcher:
    return MITCommandDispatcher(
        robot,
        kp=np.array([45.0, 44.0, 43.0, 8.0, 7.0, 6.0]),
        kd=np.array([3.0, 2.5, 2.0, 1.0, 0.8, 0.6]),
        torque_limit_nm=np.array([12.0, 12.0, 12.0, 3.0, 3.0, 2.0]),
        arm_velocity_limit_rad_s=np.array([1.0, 1.0, 1.0, 2.0, 2.0, 2.0]),
        gravity_ramp_s=0.0,
    )


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


def test_mit_velocity_recovers_qp_velocity_then_decays_with_position_error() -> None:
    dispatcher = _dispatcher(_FakeRobot())
    lookahead = np.array([0.05, 0.05, 0.05, 0.025, 0.025, 0.025])
    qp_velocity = np.array([0.4, -0.3, 0.2, 0.8, -0.6, 0.4])
    actual_deg = np.zeros(6)
    command_deg = np.rad2deg(qp_velocity * lookahead)

    dispatcher.set_arm_velocity_from_position_error(
        command_deg, actual_deg, lookahead
    )
    assert dispatcher.desired_velocity_rad_s == pytest.approx(qp_velocity)

    halfway_deg = command_deg * 0.5
    dispatcher.set_arm_velocity_from_position_error(
        command_deg, halfway_deg, lookahead
    )
    assert dispatcher.desired_velocity_rad_s == pytest.approx(qp_velocity * 0.5)

    dispatcher.set_arm_velocity_from_position_error(
        command_deg, command_deg, lookahead
    )
    assert dispatcher.desired_velocity_rad_s == pytest.approx(np.zeros(6))


def test_mit_cli_defaults_to_pos_vel_and_validates_protocol_ranges() -> None:
    parser = build_parser()
    defaults = parser.parse_args([])
    mit_defaults = parser.parse_args(["--motor-control-mode", "mit"])

    assert defaults.motor_control_mode == "pos_vel"
    assert mit_defaults.mit_kp == pytest.approx([36.0, 36.0, 36.0, 10.0, 10.0, 10.0])
    assert mit_defaults.mit_kd == pytest.approx([4.0, 4.0, 4.0, 1.0, 1.0, 1.0])
    assert mit_defaults.mit_torque_limit_nm == pytest.approx(
        [27.0, 27.0, 27.0, 7.0, 7.0, 7.0]
    )
    assert mit_defaults.mit_gravity_scale == pytest.approx(1.0)
    assert mit_defaults.mit_gravity_ramp_s == pytest.approx(0.0)
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

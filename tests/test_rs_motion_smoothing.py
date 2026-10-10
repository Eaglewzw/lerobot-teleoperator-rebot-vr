import numpy as np
import pytest

from lerobot_teleoperator_rebot_vr.control.gripper import GripperController
from lerobot_teleoperator_rebot_vr.control.joint_command import (
    bound_position_command_to_feedback, shape_joint_position_command,
)
from lerobot_teleoperator_rebot_vr.control.zero_trim import ZeroPoseTrim


@pytest.mark.parametrize("direction", [-1., 1.])
def test_moving_gripper_feedback_limit_does_not_repeatedly_restart_acceleration(direction):
    actual = 155.
    gripper = GripperController(goal_deg=310.12 if direction > 0 else 0.,
        command_deg=actual + direction * 10.8, velocity_deg_s=direction * 10.)
    for _ in range(100):
        actual += direction * .2
        previous = gripper.command_deg
        gripper.update_command(actual_deg=actual, dt_s=.02, open_deg=310.12, closed_deg=0.,
            max_speed_deg_s=120., max_acceleration_deg_s2=240., feedback_error_deg=10.8,
            shape_fn=shape_joint_position_command, bound_fn=bound_position_command_to_feedback,
            smooth_motion=True)
        assert gripper.command_deg - previous == pytest.approx(direction * .2)
        assert gripper.velocity_deg_s == pytest.approx(direction * 10.)
        assert abs(gripper.command_deg - actual) <= 10.8 + 1e-9


@pytest.mark.parametrize("start,target", [(310.12, 0.), (0., 310.12)])
@pytest.mark.parametrize("speed,acceleration", [(120., 240.), (240., 960.), (300., 1200.)])
def test_fixed_gripper_target_brakes_before_endpoint_without_velocity_snap(start, target, speed, acceleration):
    position, velocity = np.array([start]), np.zeros(1)
    positions, velocities = [start], [0.]
    for _ in range(300):
        position, velocity = shape_joint_position_command(previous_position=position,
            previous_velocity=velocity, target_position=np.array([target]), dt_s=.02,
            max_speed=np.array([speed]), max_acceleration=np.array([acceleration]),
            lower_limit=np.array([0.]), upper_limit=np.array([310.12]), brake_at_target=True)
        positions.append(position[0])
        velocities.append(velocity[0])
    assert positions[-1] == pytest.approx(target, abs=.001)
    assert np.max(np.abs(np.diff(velocities))) <= acceleration * .02 + 1e-8
    assert np.min(positions) >= 0. and np.max(positions) <= 310.12
    assert np.max(np.abs(velocities)) <= speed
    if speed == 240.:
        assert abs(positions[85] - target) < .01  # within 1.7 s with ideal feedback
    if speed == 300.:
        assert abs(positions[70] - target) < .01  # within 1.4 s with ideal feedback


def test_zero_correction_is_frozen_when_joint_breaks_static_friction():
    trim = ZeroPoseTrim([12, 12, 12, 3, 3, 3])
    trim.active = True
    q = np.deg2rad([0., 0., 2., 0., 0., 0.])
    for i in range(51):
        before = trim.update(q, np.zeros(6), i * .02)
    assert before[2] < -.3
    q[2] = np.deg2rad(1.8)  # 10 deg/s motion after breakaway.
    after = trim.update(q, np.zeros(6), 1.02)
    assert after == pytest.approx(before)
    trim.active = False
    after = trim.update(q, np.zeros(6), 1.04)
    assert after[2] == pytest.approx(before[2] + .01)

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from lerobot_teleoperator_rebot_vr.ik.kinematics import FullBodyQPIKSolver


class Model:
    lower_position_limit = np.full(6, -3.)
    upper_position_limit = np.full(6, 3.)

    def __init__(self, rotation=np.eye(3), zero_lever=False):
        self.rotation = rotation
        self.j = np.zeros((6, 6))
        self.j[:3, :3] = [[-.03, -.2, .1], [.08, 0, 0], [0, .1, -.1]]
        self.j[3:,0] = [0,0,1]
        if zero_lever:
            self.j[:, 0] = 0
        self.j[:3] = rotation @ self.j[:3]
        self.j[3:] = rotation @ self.j[3:]

    def tcp_pose_error(self, q, target, rotation):
        return np.r_[target, np.zeros(3)]

    def tcp_jacobian(self, q):
        return self.j


def solve(model, weight=1., target=(.03, 0, 0), previous=(0, .4, 0, 0, 0, 0)):
    solver = FullBodyQPIKSolver(model, position_gain=3,
        position_contour_weight=weight, position_contour_mode='shoulder_lateral',
        damping_min=.001, damping_max=.001, max_solve_time_ms=100)
    return solver.solve(target_position=model.rotation @ np.array(target), target_rotation=np.eye(3),
        q_actual=np.zeros(6), dq_previous=np.array(previous), dt=.02, q_nominal=np.zeros(6),
        max_joint_speed=np.full(6, 2.), max_joint_acceleration=np.full(6, 6.))


def test_lateral_error_reduced_without_relaxing_acceleration():
    m=Model(); a=solve(m); b=solve(m, 9.)
    assert a.success and b.success
    tangent=np.array([0,1,0])
    task=np.array([.09,0,0])
    assert abs(tangent @ (m.j[:3] @ b.joint_velocity_rad_s-task)) < abs(tangent @ (m.j[:3] @ a.joint_velocity_rad_s-task))
    assert np.all(abs(b.joint_velocity_rad_s-np.array([0,.4,0,0,0,0]))<=.12+1e-8)


def test_cost_cannot_override_binding_acceleration_limits():
    previous=(0,1,0,0,0,0)
    a=solve(Model(),previous=previous);b=solve(Model(),9.,previous=previous)
    assert a.success and b.success
    np.testing.assert_allclose(a.joint_velocity_rad_s,b.joint_velocity_rad_s,atol=1e-7)
    assert np.all(abs(b.joint_velocity_rad_s-np.array(previous))<=.12+1e-8)


@pytest.mark.parametrize('weight', [9., 25.])
def test_lateral_requests_are_not_locked_and_frame_rotation_is_equivariant(weight):
    a=solve(Model(),weight,target=(0,.01,0),previous=(0,0,0,0,0,0))
    rotated=Model(Rotation.from_rotvec([.4,-.2,.7]).as_matrix())
    b=solve(rotated,weight,target=(0,.01,0),previous=(0,0,0,0,0,0))
    assert a.success and b.success
    assert a.joint_velocity_rad_s[0]>.01
    np.testing.assert_allclose(a.joint_velocity_rad_s,b.joint_velocity_rad_s,atol=1e-6)


def test_zero_lever_is_finite_and_neutral():
    a=solve(Model(zero_lever=True));b=solve(Model(zero_lever=True),9.)
    assert a.success and b.success
    np.testing.assert_allclose(a.joint_velocity_rad_s,b.joint_velocity_rad_s,atol=1e-8)


def test_rejects_unknown_mode():
    with pytest.raises(ValueError):
        FullBodyQPIKSolver(Model(),position_contour_mode='unknown')

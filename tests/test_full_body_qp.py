from __future__ import annotations

import numpy as np
import pytest

from lerobot_teleoperator_rebot_vr.kinematics import B601Kinematics, FullBodyQPIKSolver
from lerobot_teleoperator_rebot_vr.async_ik import IKRequest, LatestOnlyQPIKWorker
from lerobot_teleoperator_rebot_vr.cartesian_controller import (
    ARM_JOINT_NAMES,
    CartesianControlConfig,
    FullBodyQPIKController,
)
from lerobot_teleoperator_rebot_vr.config_rebot_vr import DEFAULT_BASE_T_ANCHOR
from lerobot_teleoperator_rebot_vr.processor import VRFrame


@pytest.fixture()
def model():
    k = B601Kinematics()
    yield k
    k.close()


def test_gripper_end_fk_and_jacobian(model):
    q = np.array([0.1, -0.8, -0.7, 0.2, -0.1, 0.3])
    p, r = model.forward_kinematics(q)
    j = model.tcp_jacobian(q)
    assert p.shape == (3,)
    assert r.shape == (3, 3)
    assert j.shape == (6, 6)
    assert np.all(np.isfinite(j))
    eps = 1e-7
    numeric = np.zeros((6, 6))
    for i in range(6):
        qp = q.copy(); qp[i] += eps
        qm = q.copy(); qm[i] -= eps
        pp, rp = model.forward_kinematics(qp)
        pm, rm = model.forward_kinematics(qm)
        numeric[:3, i] = (pp - pm) / (2 * eps)
        from scipy.spatial.transform import Rotation
        numeric[3:, i] = Rotation.from_matrix(rp @ rm.T).as_rotvec() / (2 * eps)
    assert j == pytest.approx(numeric, abs=3e-5)


def test_qp_uses_all_axes_and_respects_limits(model):
    q = np.array([0.1, -0.8, -0.7, 0.2, -0.1, 0.3])
    p, r = model.forward_kinematics(q)
    target = r @ np.eye(3)
    solver = FullBodyQPIKSolver(model, max_solve_time_ms=20.0)
    q_next, ok, _, _, _ = solver.solve(
        target_position=p + np.array([0.002, 0.0, 0.0]),
        target_rotation=target,
        q_actual=q,
        dq_previous=np.zeros(6),
        dt=0.01,
        q_nominal=q,
        max_joint_speed=np.full(6, 0.5),
        max_joint_acceleration=np.full(6, 2.0),
    )
    assert ok
    assert q_next.shape == (6,)
    assert np.all(np.abs(q_next - q) <= 0.005 + 1e-8)
    assert np.all(q_next >= model.lower_position_limit - 1e-8)
    assert np.all(q_next <= model.upper_position_limit + 1e-8)


def test_qp_acceleration_constraint_preserves_previous_velocity() -> None:
    class LinearKinematics:
        lower_position_limit = np.full(6, -10.0)
        upper_position_limit = np.full(6, 10.0)

        @staticmethod
        def tcp_pose_error(q, target_position, target_rotation):
            del target_rotation
            q = np.asarray(q, dtype=float)
            return np.concatenate((np.asarray(target_position, dtype=float) - q[:3], -q[3:]))

        @staticmethod
        def tcp_jacobian(q):
            del q
            return np.eye(6)

    solver = FullBodyQPIKSolver(LinearKinematics(), max_solve_time_ms=20.0)
    q = np.zeros(6)
    speed = np.array([5.5] * 3 + [12.0] * 3)
    acceleration = np.array([20.0] * 3 + [60.0] * 3)
    dt = 0.01
    q_next, ok, _, _, _ = solver.solve(
        target_position=np.ones(3),
        target_rotation=np.eye(3),
        q_actual=q,
        dq_previous=speed,
        dt=dt,
        q_nominal=q,
        max_joint_speed=speed,
        max_joint_acceleration=acceleration,
    )
    assert ok
    # With dq_prev already at the speed limit, the next step must be allowed
    # to remain near that speed. The old implementation limited it to roughly
    # 2*acceleration*dt instead (0.4 and 1.2 rad/s respectively).
    assert np.all(q_next > 0.5 * speed * dt)
    assert np.all(q_next <= speed * dt + 1e-8)


def test_qp_failure_does_not_return_partial_wrist(model):
    q = np.zeros(6)
    p, r = model.forward_kinematics(q)
    solver = FullBodyQPIKSolver(model, max_solve_time_ms=0.001)
    result = solver.solve(target_position=p, target_rotation=r, q_actual=q,
                          dq_previous=np.zeros(6), dt=0.01, q_nominal=q,
                          max_joint_speed=np.ones(6), max_joint_acceleration=np.ones(6))
    assert result[1] is False


def test_qp_target_does_not_integrate_feedback_delay(model):
    q = np.array([0.1, -0.8, -0.7, 0.2, -0.1, 0.3])
    p, r = model.forward_kinematics(q)
    worker = LatestOnlyQPIKWorker(
        FullBodyQPIKSolver(model, max_solve_time_ms=20.0),
        max_joint_speed_rad_s=np.array([5.5] * 3 + [12.0] * 3),
        max_joint_acceleration_rad_s2=np.array([20.0] * 3 + [60.0] * 3),
    )
    seed = q.copy()
    targets = []
    for sequence in range(1, 4):
        request = IKRequest(
            sequence=sequence,
            generation=1,
            sample_id=sequence,
            target_position=p + np.array([0.0, 0.1, 0.0]),
            target_rotation=r,
            q_seed=seed,
            q_actual=q,
            dq_previous=np.zeros(6),
            q_nominal=q,
            dt=0.02,
        )
        result = worker._solve(request)
        assert result.success
        targets.append(result.q_target_rad.copy())
        seed = result.q_target_rad.copy()
    # Feedback is intentionally held at q. Re-solving the same target must
    # return the same one-step command, rather than accumulating old deltas.
    assert targets[1] == pytest.approx(targets[0], abs=1e-8)
    assert targets[2] == pytest.approx(targets[0], abs=1e-8)


def test_stationary_grip_does_not_drift_from_home_posture(model):
    controller = FullBodyQPIKController(
        model,
        xr_to_base_rotation=np.asarray(DEFAULT_BASE_T_ANCHOR)[:3, :3],
        config=CartesianControlConfig(
            position_filter_hz=8.0,
            orientation_filter_hz=6.0,
            max_joint_speed_rad_s=5.5,
            max_joint_acceleration_rad_s2=20.0,
            wrist_speed_rad_s=12.0,
            wrist_acceleration_rad_s2=60.0,
        ),
    )
    q_deg = np.array([0.0, -60.0, -70.0, 0.0, 0.0, 0.0])
    observation = {f"{name}.pos": float(value) for name, value in zip(ARM_JOINT_NAMES, q_deg)}
    observation["gripper.pos"] = -180.0
    now = 1_000_000_000
    controller.start()
    try:
        for index in range(16):
            frame = VRFrame(
                grip_pos=np.zeros(3), grip_quat=np.array([0.0, 0.0, 0.0, 1.0]),
                squeeze=0.0 if index == 0 else 1.0, trigger=0.0,
                is_tracking=True, received_monotonic_ns=now + index * 20_000_000,
                tracking_timestamp_ns=now + index * 20_000_000, stream_epoch=1,
                side="right", primary_button=False, secondary_button=False, status=1,
            )
            _, status = controller.update(frame, observation, 0.02, now_ns=frame.received_monotonic_ns)
            assert status.state.value in ("idle", "active")
        assert status.target_deg[:6] == pytest.approx(q_deg, abs=0.05)
        assert status.command_deg[:6] == pytest.approx(q_deg, abs=0.05)
    finally:
        controller.stop()


def test_grip_capture_resets_stale_command_velocity_and_skips_first_qp(model):
    controller = FullBodyQPIKController(
        model,
        xr_to_base_rotation=np.asarray(DEFAULT_BASE_T_ANCHOR)[:3, :3],
        config=CartesianControlConfig(qp_max_solve_time_ms=20.0),
    )
    q_deg = np.array([1.0, -60.5, -69.7, 0.4, -0.2, 0.6])
    observation = {
        f"{name}.pos": float(value)
        for name, value in zip(ARM_JOINT_NAMES, q_deg)
    }
    observation["gripper.pos"] = -180.0
    now = 2_000_000_000

    released = VRFrame(
        grip_pos=np.zeros(3),
        grip_quat=np.array([0.0, 0.0, 0.0, 1.0]),
        squeeze=0.0,
        trigger=0.0,
        is_tracking=True,
        received_monotonic_ns=now,
        tracking_timestamp_ns=now,
        stream_epoch=1,
        side="right",
        primary_button=False,
        secondary_button=False,
        status=1,
    )
    controller.update(released, observation, 0.02, now_ns=now)
    controller._q_command_rad += 0.1
    controller._dq_command_rad_s.fill(1.0)

    pressed = VRFrame(
        grip_pos=np.zeros(3),
        grip_quat=np.array([0.0, 0.0, 0.0, 1.0]),
        squeeze=1.0,
        trigger=0.0,
        is_tracking=True,
        received_monotonic_ns=now + 20_000_000,
        tracking_timestamp_ns=now + 20_000_000,
        stream_epoch=1,
        side="right",
        primary_button=False,
        secondary_button=False,
        status=1,
    )
    _, status = controller.update(
        pressed, observation, 0.02, now_ns=pressed.received_monotonic_ns
    )
    assert status.target_deg[:6] == pytest.approx(q_deg)
    assert status.command_deg[:6] == pytest.approx(q_deg)
    assert controller._dq_command_rad_s == pytest.approx(np.zeros(6))
    assert controller.worker.submitted == 0

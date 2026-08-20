from __future__ import annotations

import time

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from lerobot_teleoperator_rebot_vr.async_ik import (
    IKRequest,
    IKResult,
    LatestOnlyQPIKWorker,
)
from lerobot_teleoperator_rebot_vr.cartesian_controller import (
    ARM_JOINT_NAMES,
    CartesianControlConfig,
    FullBodyQPIKController,
)
from lerobot_teleoperator_rebot_vr.joint_command import shape_joint_position_command
from lerobot_teleoperator_rebot_vr.pose_mapping import (
    DEFAULT_XR_TO_WORLD,
    RelativePoseMapper,
    TeleopState,
)
from lerobot_teleoperator_rebot_vr.processor import VRFrame
from lerobot_teleoperator_rebot_vr.tracking import ControllerSample


def _sample(
    received_ns: int,
    *,
    position=(0.0, 0.0, 0.0),
    quaternion=(0.0, 0.0, 0.0, 1.0),
    grip=0.0,
    trigger=0.0,
    primary=False,
    secondary=False,
    epoch=1,
    tracking_timestamp_ns: int | None = None,
) -> ControllerSample:
    return ControllerSample(
        received_monotonic_ns=received_ns,
        tracking_timestamp_ns=(
            received_ns if tracking_timestamp_ns is None else tracking_timestamp_ns
        ),
        stream_epoch=epoch,
        side="right",
        position=position,
        quaternion_xyzw=quaternion,
        grip=grip,
        trigger=trigger,
        primary_button=primary,
        secondary_button=secondary,
        status=1,
    )


def _mapper(**kwargs) -> RelativePoseMapper:
    return RelativePoseMapper(
        position_filter_hz=0.0,
        orientation_filter_hz=0.0,
        position_deadband_m=0.0,
        orientation_deadband_rad=0.0,
        stale_timeout_s=1.0,
        **kwargs,
    )


def _activate(
    mapper: RelativePoseMapper,
    base_ns: int,
    ee_position: np.ndarray | None = None,
    ee_rotation: np.ndarray | None = None,
) -> None:
    position = np.zeros(3) if ee_position is None else ee_position
    rotation = np.eye(3) if ee_rotation is None else ee_rotation
    mapper.update(_sample(base_ns), position, rotation, now_ns=base_ns)
    update = mapper.update(
        _sample(base_ns + 10_000_000, grip=1.0),
        position,
        rotation,
        now_ns=base_ns + 10_000_000,
    )
    assert update.state is TeleopState.ACTIVE
    assert update.reference_captured


def test_default_xr_translation_and_rotation_axes_map_to_world() -> None:
    base_ns = 1_000_000_000
    expected_axes = (
        np.array([0.0, -1.0, 0.0]),
        np.array([0.0, 0.0, 1.0]),
        np.array([-1.0, 0.0, 0.0]),
    )
    for axis, expected_world in zip(np.eye(3), expected_axes):
        mapper = _mapper()
        _activate(mapper, base_ns)
        moved = mapper.update(
            _sample(
                base_ns + 20_000_000,
                position=0.1 * axis,
                quaternion=Rotation.from_rotvec(0.2 * axis).as_quat(),
                grip=1.0,
            ),
            np.zeros(3),
            np.eye(3),
            now_ns=base_ns + 20_000_000,
        )
        assert moved.target is not None
        assert moved.target.position == pytest.approx(0.1 * expected_world)
        assert Rotation.from_matrix(moved.target.rotation).as_rotvec() == pytest.approx(
            0.2 * expected_world
        )

    assert DEFAULT_XR_TO_WORLD @ [1, 0, 0] == pytest.approx([0, -1, 0])


def test_grip_release_hysteresis_epoch_reset_and_quaternion_sign_continuity() -> None:
    mapper = _mapper(xr_to_world=np.eye(3))
    now = 2_000_000_000
    held = mapper.update(_sample(now, grip=1.0), np.zeros(3), np.eye(3), now_ns=now)
    assert held.state is TeleopState.IDLE
    assert held.require_release
    mapper.update(
        _sample(now + 1, grip=0.7), np.zeros(3), np.eye(3), now_ns=now + 1
    )
    active = mapper.update(
        _sample(now + 2, grip=0.9), np.zeros(3), np.eye(3), now_ns=now + 2
    )
    assert active.state is TeleopState.ACTIVE
    middle = mapper.update(
        _sample(now + 3, quaternion=(0, 0, 0, -1), grip=0.8),
        np.zeros(3),
        np.eye(3),
        now_ns=now + 3,
    )
    assert middle.state is TeleopState.ACTIVE
    assert middle.target.rotation == pytest.approx(np.eye(3))

    reset = mapper.update(
        _sample(now + 4, grip=1.0, epoch=2),
        np.zeros(3),
        np.eye(3),
        now_ns=now + 4,
    )
    assert reset.state is TeleopState.IDLE
    assert reset.require_release


def test_pose_filter_and_deadband_run_once_per_latest_sample() -> None:
    mapper = RelativePoseMapper(
        xr_to_world=np.eye(3),
        position_filter_hz=1.0,
        orientation_filter_hz=1.0,
        position_deadband_m=5e-4,
        orientation_deadband_rad=np.deg2rad(0.25),
        stale_timeout_s=1.0,
    )
    base = 3_000_000_000
    _activate(mapper, base)
    sample = _sample(
        base + 110_000_000,
        position=(0.1, 0, 0),
        quaternion=Rotation.from_rotvec([0, 0, 0.2]).as_quat(),
        grip=1.0,
    )
    first = mapper.update(sample, np.zeros(3), np.eye(3), now_ns=sample.received_monotonic_ns)
    repeated = mapper.update(
        sample,
        np.zeros(3),
        np.eye(3),
        now_ns=sample.received_monotonic_ns + 20_000_000,
    )
    assert repeated.target.position == pytest.approx(first.target.position)
    assert repeated.target.rotation == pytest.approx(first.target.rotation)


def test_pose_filter_uses_receive_time_when_upstream_timestamp_is_missing() -> None:
    mapper = RelativePoseMapper(
        xr_to_world=np.eye(3),
        position_filter_hz=1.0,
        orientation_filter_hz=1.0,
        position_deadband_m=0.0,
        orientation_deadband_rad=0.0,
        stale_timeout_s=1.0,
    )
    base = 4_000_000_000
    mapper.update(
        _sample(base, tracking_timestamp_ns=0),
        np.zeros(3),
        np.eye(3),
        now_ns=base,
    )
    captured = mapper.update(
        _sample(base + 10_000_000, grip=1.0, tracking_timestamp_ns=0),
        np.zeros(3),
        np.eye(3),
        now_ns=base + 10_000_000,
    )
    moved = mapper.update(
        _sample(
            base + 110_000_000,
            position=(0.1, 0.0, 0.0),
            grip=1.0,
            tracking_timestamp_ns=0,
        ),
        np.zeros(3),
        np.eye(3),
        now_ns=base + 110_000_000,
    )
    assert captured.reference_captured
    alpha = 1.0 - np.exp(-2.0 * np.pi * 1.0 * 0.1)
    assert moved.target.position == pytest.approx([0.1 * alpha, 0.0, 0.0])


class FakeKinematics:
    lower_position_limit = np.array([-2.8, -3.14, -3.14, -1.4, -1.5, -1.5])
    upper_position_limit = np.array([2.8, 0.0, 0.0, 1.4, 1.5, 1.5])

    def forward_kinematics(self, q_rad):
        q = np.asarray(q_rad, dtype=np.float64)
        return q[:3].copy(), Rotation.from_euler("ZYX", q[3:6]).as_matrix()


class ImmediateWorker:
    def __init__(self) -> None:
        self.submitted = 0
        self.solved = 0
        self.rejected = 0
        self.clear_count = 0
        self.requests: list[IKRequest] = []
        self.result: IKResult | None = None

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    def clear(self) -> None:
        self.clear_count += 1
        self.result = None

    def submit(self, request: IKRequest) -> None:
        self.submitted += 1
        self.solved += 1
        self.requests.append(request)
        q_goal = request.q_seed.copy()
        q_goal[:3] = request.target_position
        self.result = IKResult(
            generation=request.generation,
            sequence=request.sequence,
            sample_id=request.sample_id,
            q_target_rad=q_goal,
            success=True,
            position_error_m=0.0,
            solve_time_ms=0.1,
        )

    def latest_result(self) -> IKResult | None:
        return self.result


def _observation(q=(0.0, -60.0, -70.0, 0.0, 0.0, 0.0), gripper=-270.0):
    observation = {
        f"{name}.pos": value for name, value in zip(ARM_JOINT_NAMES, q)
    }
    observation["gripper.pos"] = gripper
    return observation


def _legacy_frame(now_ns: int, **kwargs) -> VRFrame:
    values = {
        "grip_pos": np.zeros(3),
        "grip_quat": np.array([0.0, 0.0, 0.0, 1.0]),
        "squeeze": 0.0,
        "trigger": 0.0,
        "received_monotonic_ns": now_ns,
        "stream_epoch": 1,
    }
    values.update(kwargs)
    return VRFrame(**values)


def test_primary_button_edge_returns_home_and_requires_grip_release() -> None:
    worker = ImmediateWorker()
    controller = FullBodyQPIKController(
        FakeKinematics(),
        xr_to_base_rotation=DEFAULT_XR_TO_WORLD,
        config=CartesianControlConfig(
            position_filter_hz=0.0,
            orientation_filter_hz=0.0,
            stale_timeout_s=1.0,
            max_joint_speed_rad_s=1000.0,
            max_joint_acceleration_rad_s2=1000.0,
        ),
        ik_worker=worker,
    )
    now = time.monotonic_ns()
    controller.update(_legacy_frame(now), _observation(), 0.02, now_ns=now)
    controller.update(
        _legacy_frame(now + 1, squeeze=1.0),
        _observation(),
        0.02,
        now_ns=now + 1,
    )
    action, status = controller.update(
        _legacy_frame(now + 2, squeeze=1.0, primary_button=True),
        _observation(),
        0.05,
        now_ns=now + 2,
    )
    assert status.home_requested
    assert status.state is TeleopState.IDLE
    assert [action[f"{name}.pos"] for name in ARM_JOINT_NAMES] == pytest.approx(
        np.rad2deg(controller.home_q_rad)
    )
    generation = status.generation

    _, held = controller.update(
        _legacy_frame(now + 3, squeeze=1.0, primary_button=True),
        _observation(),
        0.02,
        now_ns=now + 3,
    )
    assert not held.home_requested
    assert held.state is TeleopState.IDLE
    assert held.generation == generation
    controller.update(
        _legacy_frame(now + 4, squeeze=0.0),
        _observation(),
        0.02,
        now_ns=now + 4,
    )
    _, rearmed = controller.update(
        _legacy_frame(now + 5, squeeze=1.0),
        _observation(),
        0.02,
        now_ns=now + 5,
    )
    assert rearmed.state is TeleopState.ACTIVE


def test_secondary_button_edge_returns_zero_smoothly_and_requires_grip_release() -> None:
    worker = ImmediateWorker()
    controller = FullBodyQPIKController(
        FakeKinematics(),
        xr_to_base_rotation=DEFAULT_XR_TO_WORLD,
        config=CartesianControlConfig(
            position_filter_hz=0.0,
            orientation_filter_hz=0.0,
            stale_timeout_s=1.0,
            max_joint_speed_rad_s=0.4,
            max_joint_acceleration_rad_s2=1.0,
        ),
        ik_worker=worker,
    )
    actual_deg = np.array([10.0, -60.0, -70.0, 20.0, -30.0, 40.0])
    observation = _observation(q=actual_deg)
    now = time.monotonic_ns()
    controller.update(_legacy_frame(now), observation, 0.02, now_ns=now)
    _, active = controller.update(
        _legacy_frame(now + 1, squeeze=1.0),
        observation,
        0.02,
        now_ns=now + 1,
    )
    assert active.state is TeleopState.ACTIVE

    action, status = controller.update(
        _legacy_frame(now + 2, squeeze=1.0, secondary_button=True),
        observation,
        0.05,
        now_ns=now + 2,
    )
    command_deg = np.array([action[f"{name}.pos"] for name in ARM_JOINT_NAMES])
    assert status.secondary_button
    assert status.zero_requested
    assert not status.home_requested
    assert status.state is TeleopState.IDLE
    assert status.target_deg[:6] == pytest.approx(np.zeros(6))
    assert np.all(np.abs(command_deg) < np.abs(actual_deg))
    assert command_deg != pytest.approx(np.zeros(6))
    generation = status.generation

    _, held = controller.update(
        _legacy_frame(now + 3, squeeze=1.0, secondary_button=True),
        observation,
        0.02,
        now_ns=now + 3,
    )
    assert not held.zero_requested
    assert held.state is TeleopState.IDLE
    assert held.generation == generation

    controller.update(
        _legacy_frame(now + 4, squeeze=1.0),
        observation,
        0.02,
        now_ns=now + 4,
    )
    _, still_idle = controller.update(
        _legacy_frame(now + 5, squeeze=1.0),
        observation,
        0.02,
        now_ns=now + 5,
    )
    assert still_idle.state is TeleopState.IDLE
    controller.update(
        _legacy_frame(now + 6, squeeze=0.0),
        observation,
        0.02,
        now_ns=now + 6,
    )
    _, rearmed = controller.update(
        _legacy_frame(now + 7, squeeze=1.0),
        observation,
        0.02,
        now_ns=now + 7,
    )
    assert rearmed.state is TeleopState.ACTIVE


def test_command_shaper_formula_and_controller_dt_cap() -> None:
    position, velocity = shape_joint_position_command(
        previous_position=np.array([0.0]),
        previous_velocity=np.array([0.0]),
        target_position=np.array([1.0]),
        dt_s=0.1,
        max_speed=np.array([2.0]),
        max_acceleration=np.array([8.0]),
        lower_limit=np.array([-2.0]),
        upper_limit=np.array([2.0]),
    )
    assert velocity == pytest.approx([0.8])
    assert position == pytest.approx([0.08])

    controller = FullBodyQPIKController(
        FakeKinematics(),
        xr_to_base_rotation=DEFAULT_XR_TO_WORLD,
        config=CartesianControlConfig(
            stale_timeout_s=1.0,
            gripper_max_speed_deg_s=100.0,
            gripper_max_acceleration_deg_s2=100.0,
        ),
        ik_worker=ImmediateWorker(),
    )
    now = time.monotonic_ns()
    controller.update(
        _legacy_frame(now, trigger=0.0),
        _observation(),
        0.1,
        now_ns=now,
    )
    action, _ = controller.update(
        _legacy_frame(now + 1, trigger=1.0),
        _observation(),
        0.1,
        now_ns=now + 1,
    )
    # Controller caps dt to 0.05: dv=5 deg/s and dx=0.25 deg.
    assert action["gripper.pos"] == pytest.approx(-269.75)

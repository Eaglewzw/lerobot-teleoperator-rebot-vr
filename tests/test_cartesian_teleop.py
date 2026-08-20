from __future__ import annotations

import threading
import time
from types import SimpleNamespace

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

import lerobot_teleoperator_rebot_vr.cartesian_controller as cartesian_controller_module
from lerobot_teleoperator_rebot_vr.async_ik import IKRequest, IKResult, LatestOnlyQPIKWorker
from lerobot_teleoperator_rebot_vr.cartesian_controller import (
    ARM_JOINT_NAMES,
    CartesianControlConfig,
    FullBodyQPIKController,
)
from lerobot_teleoperator_rebot_vr.config_rebot_vr import DEFAULT_BASE_T_ANCHOR
from lerobot_teleoperator_rebot_vr.joint_command import (
    bound_position_command_to_feedback,
)
from lerobot_teleoperator_rebot_vr.pose_mapping import (
    PoseTarget,
    RelativePoseMapper,
    TeleopState,
)
from lerobot_teleoperator_rebot_vr.processor import VRFrame
from lerobot_teleoperator_rebot_vr.startup_pose import (
    StartupPoseMover,
    reference_initial_q_to_dm,
)
from lerobot_teleoperator_rebot_vr.teleoperate_real import (
    _follower_pos_vel_velocity,
    _follower_relative_target,
    _feedback_hold_action,
    _parser,
    _send_feedback_hold_action,
    _status_line,
    _validate_args,
    _move_to_initial_pose,
)


XR_TO_BASE = np.asarray(DEFAULT_BASE_T_ANCHOR, dtype=np.float64)[:3, :3]


def _frame(
    sample_id: int,
    *,
    position=(0.0, 0.0, 0.0),
    quaternion=(0.0, 0.0, 0.0, 1.0),
    grip=0.0,
    trigger=0.0,
    tracking=True,
    stream_epoch=1,
) -> VRFrame:
    return VRFrame(
        grip_pos=position,
        grip_quat=quaternion,
        squeeze=grip,
        trigger=trigger,
        is_tracking=tracking,
        received_monotonic_ns=sample_id,
        stream_epoch=stream_epoch,
    )


def _unfiltered_mapper() -> RelativePoseMapper:
    return RelativePoseMapper(
        position_filter_hz=0.0,
        orientation_filter_hz=0.0,
        position_deadband_m=0.0,
        orientation_deadband_rad=0.0,
        stale_timeout_s=1.0,
    )


def test_relative_pose_uses_already_transformed_base_frame_and_filters_each_sample_once() -> None:
    mapper = _unfiltered_mapper()
    now = time.monotonic_ns()
    ee_position = np.array([0.4, -0.1, 0.3])
    ee_rotation = Rotation.from_euler("z", 0.4).as_matrix()

    assert mapper.update(_frame(now), ee_position, ee_rotation, now_ns=now).state is TeleopState.IDLE
    captured = mapper.update(
        _frame(now + 10_000_000, position=(1.0, 2.0, 3.0), grip=1.0),
        ee_position,
        ee_rotation,
        now_ns=now + 10_000_000,
    )
    assert captured.reference_captured
    assert np.allclose(captured.target.position, ee_position)

    delta_base = np.array([0.05, -0.02, 0.03])
    sample = _frame(
        now + 30_000_000,
        position=np.array([1.0, 2.0, 3.0]) + delta_base,
        grip=1.0,
    )
    moved = mapper.update(sample, ee_position, ee_rotation, now_ns=now + 30_000_000)
    repeated = mapper.update(sample, ee_position, ee_rotation, now_ns=now + 40_000_000)
    assert np.allclose(moved.target.position, ee_position + delta_base)
    assert np.allclose(repeated.target.position, moved.target.position)


class FakeKinematics:
    lower_position_limit = np.array([-2.8, -3.14, -3.14, -1.87, -1.57, -3.14])
    upper_position_limit = np.array([2.8, 0.0, 0.0, 1.57, 1.57, 3.14])

    def forward_kinematics(self, q_rad: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        q = np.asarray(q_rad, dtype=np.float64)
        return q[:3].copy(), Rotation.from_euler("ZYX", q[3:6]).as_matrix()


class ImmediateIKWorker:
    def __init__(self) -> None:
        self.submitted = 0
        self.solved = 0
        self.rejected = 0
        self.result: IKResult | None = None
        self.requests: list[IKRequest] = []

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    def clear(self) -> None:
        self.result = None

    def submit(self, request: IKRequest) -> None:
        self.submitted += 1
        self.solved += 1
        self.requests.append(request)
        q_target = request.q_seed.copy()
        q_target[:3] = request.target_position
        joint_velocity = (q_target - request.q_actual) / request.dt
        self.result = IKResult(
            generation=request.generation,
            sequence=request.sequence,
            sample_id=request.sample_id,
            q_target_rad=q_target,
            success=True,
            position_error_m=0.0,
            solve_time_ms=0.1,
            orientation_error_rad=0.02,
            sigma_min=0.12,
            condition_number=8.0,
            damping=1e-3,
            orientation_weight=2.0,
            joint_velocity_rad_s=joint_velocity,
            submitted_monotonic_ns=request.submitted_monotonic_ns,
        )

    def latest_result(self) -> IKResult | None:
        return self.result


class FailingIKWorker(ImmediateIKWorker):
    def submit(self, request: IKRequest) -> None:
        self.submitted += 1
        self.rejected += 1
        self.requests.append(request)
        self.result = IKResult(
            generation=request.generation,
            sequence=request.sequence,
            sample_id=request.sample_id,
            q_target_rad=request.q_seed,
            success=False,
            position_error_m=0.01,
            solve_time_ms=0.1,
            reason="position_not_converged",
        )


def _observation(q_deg=(0.0, -60.0, -70.0, 0.0, 0.0, 0.0), gripper=-270.0):
    result = {f"{name}.pos": value for name, value in zip(ARM_JOINT_NAMES, q_deg)}
    result["gripper.pos"] = gripper
    return result


def test_closed_loop_starts_from_actual_pose_and_atomically_applies_arm_and_wrist_target() -> None:
    worker = ImmediateIKWorker()
    controller = FullBodyQPIKController(
        FakeKinematics(),
        xr_to_base_rotation=XR_TO_BASE,
        config=CartesianControlConfig(
            position_filter_hz=0.0,
            orientation_filter_hz=0.0,
            position_deadband_m=0.0,
            orientation_deadband_rad=0.0,
            stale_timeout_s=1.0,
            max_joint_speed_rad_s=1000.0,
            max_joint_acceleration_rad_s2=1000.0,
            arm_command_lookahead_s=0.02,
            wrist_command_lookahead_s=0.02,
        ),
        ik_worker=worker,
    )
    now = time.monotonic_ns()
    observation = _observation()

    initial, status = controller.update(_frame(now), observation, 0.02, now_ns=now)
    assert status.state is TeleopState.IDLE
    assert [initial[f"{name}.pos"] for name in ARM_JOINT_NAMES] == pytest.approx(
        [0.0, -60.0, -70.0, 0.0, 0.0, 0.0]
    )

    controller.update(
        _frame(now + 20_000_000, grip=1.0),
        observation,
        0.02,
        now_ns=now + 20_000_000,
    )
    xr_rotation = Rotation.from_rotvec([0.2, 0.0, 0.0]).as_matrix()
    rotation_base = XR_TO_BASE @ xr_rotation @ XR_TO_BASE.T
    moved, status = controller.update(
        _frame(
            now + 40_000_000,
            position=(0.1, 0.0, 0.0),
            quaternion=Rotation.from_matrix(rotation_base).as_quat(),
            grip=1.0,
        ),
        observation,
        0.02,
        now_ns=now + 40_000_000,
    )
    assert status.ik_success is True
    expected_rad = np.deg2rad(np.array([0.0, -60.0, -70.0, 0.0, 0.0, 0.0]))
    expected_rad[0] += 0.1
    expected_rad[3:6] = 0.0
    assert [moved[f"{name}.pos"] for name in ARM_JOINT_NAMES] == pytest.approx(
        np.rad2deg(expected_rad)
    )
    request = worker.requests[-1]
    assert request.sample_id == now + 40_000_000
    assert request.target_linear_velocity_m_s == pytest.approx([5.0, 0.0, 0.0])
    assert request.target_angular_velocity_rad_s == pytest.approx(
        Rotation.from_matrix(rotation_base).as_rotvec() / 0.02
    )
    assert status.sigma_min == pytest.approx(0.12)
    assert status.condition_number == pytest.approx(8.0)
    assert status.current_damping == pytest.approx(1e-3)
    assert status.current_orientation_weight == pytest.approx(2.0)
    assert status.qp_solve_time_ms == pytest.approx(0.1)
    assert status.qp_result_age_ms == pytest.approx(0.0)
    assert status.tcp_position_error_m == pytest.approx(0.1)
    assert status.qp_joint_velocity_rad_s == pytest.approx(
        worker.result.joint_velocity_rad_s
    )


def test_completed_qp_is_consumed_before_next_submission_and_keeps_its_own_velocity() -> None:
    worker = ImmediateIKWorker()
    controller = FullBodyQPIKController(
        FakeKinematics(),
        xr_to_base_rotation=XR_TO_BASE,
        config=CartesianControlConfig(
            position_filter_hz=0.0,
            orientation_filter_hz=0.0,
            position_deadband_m=0.0,
            orientation_deadband_rad=0.0,
            stale_timeout_s=1.0,
            max_joint_speed_rad_s=1000.0,
            max_joint_acceleration_rad_s2=1000.0,
        ),
        ik_worker=worker,
    )
    now = time.monotonic_ns()
    observation = _observation()
    controller.update(_frame(now), observation, 0.02, now_ns=now)
    controller.update(
        _frame(now + 20_000_000, grip=1.0),
        observation,
        0.02,
        now_ns=now + 20_000_000,
    )
    controller.update(
        _frame(now + 40_000_000, position=(0.1, 0.0, 0.0), grip=1.0),
        observation,
        0.02,
        now_ns=now + 40_000_000,
    )
    first_request = worker.requests[-1]
    assert worker.result is not None
    expected_qp_velocity = (
        worker.result.q_target_rad - first_request.q_actual
    ) / first_request.dt

    # The outer shaper may clamp/reset its own velocity between QP solves.
    outer_velocity = np.full(6, -123.0)
    controller._dq_command_rad_s[:] = outer_velocity
    controller.update(
        _frame(now + 60_000_000, position=(0.2, 0.0, 0.0), grip=1.0),
        observation,
        0.02,
        now_ns=now + 60_000_000,
    )

    assert worker.submitted == 2
    assert worker.requests[-1].dq_previous == pytest.approx(expected_qp_velocity)
    assert worker.requests[-1].dq_previous != pytest.approx(outer_velocity)


def test_active_qp_command_uses_feedback_bounded_lookahead_without_arm_reshaping(
    monkeypatch,
) -> None:
    arm_shape_calls = 0
    original_shape = cartesian_controller_module.shape_joint_position_command

    def recording_shape(**kwargs):
        nonlocal arm_shape_calls
        if np.asarray(kwargs["previous_position"]).shape == (6,):
            arm_shape_calls += 1
        return original_shape(**kwargs)

    monkeypatch.setattr(
        cartesian_controller_module,
        "shape_joint_position_command",
        recording_shape,
    )
    worker = ImmediateIKWorker()
    controller = FullBodyQPIKController(
        FakeKinematics(),
        xr_to_base_rotation=XR_TO_BASE,
        config=CartesianControlConfig(
            position_filter_hz=0.0,
            orientation_filter_hz=0.0,
            position_deadband_m=0.0,
            orientation_deadband_rad=0.0,
            stale_timeout_s=1.0,
            max_joint_speed_rad_s=1000.0,
            max_joint_acceleration_rad_s2=1000.0,
            arm_command_lookahead_s=0.05,
            wrist_command_lookahead_s=0.025,
            max_command_feedback_error_deg=1.8,
        ),
        ik_worker=worker,
    )
    now = time.monotonic_ns()
    observation = _observation()
    controller.update(_frame(now), observation, 0.02, now_ns=now)
    controller.update(
        _frame(now + 20_000_000, grip=1.0),
        observation,
        0.02,
        now_ns=now + 20_000_000,
    )
    arm_shape_calls = 0

    action, status = controller.update(
        _frame(now + 40_000_000, position=(0.1, 0.0, 0.0), grip=1.0),
        observation,
        0.02,
        now_ns=now + 40_000_000,
    )

    assert arm_shape_calls == 0
    assert status.target_deg[0] == pytest.approx(np.rad2deg(0.25))
    assert status.command_deg[0] == pytest.approx(1.8)
    assert action["shoulder_pan.pos"] == pytest.approx(1.8)
    assert status.target_linear_velocity_m_s == pytest.approx([5.0, 0.0, 0.0])
    rendered = _status_line(status)
    assert "result_age_ms=0.000" in rendered
    assert "target_twist linear_m_s/angular_rad_s=" in rendered


def test_wrist_target_updates_when_position_ik_fails() -> None:
    worker = FailingIKWorker()
    controller = FullBodyQPIKController(
        FakeKinematics(),
        xr_to_base_rotation=XR_TO_BASE,
        config=CartesianControlConfig(
            position_filter_hz=0.0,
            orientation_filter_hz=0.0,
            position_deadband_m=0.0,
            orientation_deadband_rad=0.0,
            stale_timeout_s=1.0,
            max_joint_speed_rad_s=1000.0,
            max_joint_acceleration_rad_s2=1000.0,
        ),
        ik_worker=worker,
    )
    now = time.monotonic_ns()
    observation = _observation()
    controller.update(_frame(now), observation, 0.02, now_ns=now)
    controller.update(
        _frame(now + 10_000_000, grip=1.0),
        observation,
        0.02,
        now_ns=now + 10_000_000,
    )
    target_rotation = Rotation.from_euler("Z", 0.3).as_matrix()
    action, status = controller.update(
        _frame(
            now + 20_000_000,
            position=(0.1, 0.0, 0.0),
            quaternion=Rotation.from_matrix(target_rotation).as_quat(),
            grip=1.0,
        ),
        observation,
        0.02,
        now_ns=now + 20_000_000,
    )

    assert status.ik_success is False
    assert controller._q_goal_rad[:3] == pytest.approx(
        np.deg2rad([0.0, -60.0, -70.0])
    )
    assert status.orientation_error_deg == pytest.approx(np.rad2deg(0.3))
    rendered_status = _status_line(status)
    assert "orientation_error_deg=17.189" in rendered_status


def test_fresh_tracking_immediately_maps_trigger_to_gripper() -> None:
    controller = FullBodyQPIKController(
        FakeKinematics(),
        xr_to_base_rotation=XR_TO_BASE,
        config=CartesianControlConfig(
            stale_timeout_s=1.0,
            gripper_open_deg=-140.0,
            gripper_max_speed_deg_s=10_000.0,
            gripper_max_acceleration_deg_s2=1_000_000_000.0,
        ),
        ik_worker=ImmediateIKWorker(),
    )
    now = time.monotonic_ns()
    observation = _observation(gripper=-70.0)

    held, held_status = controller.update(
        _frame(now, trigger=0.0), observation, 0.1, now_ns=now
    )
    middle, middle_status = controller.update(
        _frame(now + 10_000_000, trigger=0.5),
        observation,
        0.1,
        now_ns=now + 10_000_000,
    )
    closed, closed_status = controller.update(
        _frame(now + 20_000_000, trigger=1.0),
        observation,
        0.1,
        now_ns=now + 20_000_000,
    )
    opened, open_status = controller.update(
        _frame(now + 30_000_000, trigger=0.0),
        observation,
        0.1,
        now_ns=now + 30_000_000,
    )

    assert held["gripper.pos"] == pytest.approx(-140.0)
    assert held_status.gripper_target_deg == pytest.approx(-140.0)
    assert held_status.gripper_trigger_active
    assert middle["gripper.pos"] == pytest.approx(-70.0)
    assert middle_status.gripper_target_deg == pytest.approx(-70.0)
    assert middle_status.gripper_trigger_active
    assert closed["gripper.pos"] == pytest.approx(0.0)
    assert closed_status.trigger == pytest.approx(1.0)
    assert closed_status.gripper_actual_deg == pytest.approx(-70.0)
    assert closed_status.gripper_target_deg == pytest.approx(0.0)
    assert closed_status.gripper_command_deg == pytest.approx(0.0)
    assert closed_status.gripper_trigger_active
    assert opened["gripper.pos"] == pytest.approx(-140.0)
    assert open_status.gripper_target_deg == pytest.approx(-140.0)


@pytest.mark.parametrize("open_deg", (-80.0, -100.0, -140.0))
def test_cli_gripper_open_endpoint_reaches_final_action(open_deg: float) -> None:
    args = _parser().parse_args(["--gripper-open-deg", str(open_deg)])
    controller = FullBodyQPIKController(
        FakeKinematics(),
        xr_to_base_rotation=XR_TO_BASE,
        config=CartesianControlConfig(
            stale_timeout_s=1.0,
            gripper_open_deg=args.gripper_open_deg,
            gripper_closed_deg=args.gripper_closed_deg,
            gripper_max_speed_deg_s=10_000.0,
            gripper_max_acceleration_deg_s2=1_000_000_000.0,
        ),
        ik_worker=ImmediateIKWorker(),
    )
    now = time.monotonic_ns()
    observation = _observation(gripper=-40.0)
    controller.update(
        _frame(now, trigger=0.0), observation, 0.05, now_ns=now
    )
    action, status = controller.update(
        _frame(now + 10_000_000, trigger=0.0),
        observation,
        0.05,
        now_ns=now + 10_000_000,
    )

    assert status.gripper_trigger_active
    assert status.gripper_target_deg == pytest.approx(open_deg)
    assert status.gripper_command_deg == pytest.approx(open_deg)
    assert action["gripper.pos"] == pytest.approx(open_deg)


def test_closed_loop_keeps_gripper_command_inside_follower_tracking_window() -> None:
    controller = FullBodyQPIKController(
        FakeKinematics(),
        xr_to_base_rotation=XR_TO_BASE,
        config=CartesianControlConfig(
            stale_timeout_s=1.0,
            gripper_open_deg=-140.0,
            gripper_max_speed_deg_s=10_000.0,
            gripper_max_acceleration_deg_s2=100_000.0,
            max_command_feedback_error_deg=1.8,
        ),
        ik_worker=ImmediateIKWorker(),
    )
    now = time.monotonic_ns()
    controller.update(
        _frame(now, trigger=0.0),
        _observation(gripper=-140.0),
        0.1,
        now_ns=now,
    )
    action, status = controller.update(
        _frame(now + 10_000_000, trigger=1.0),
        _observation(gripper=-140.0),
        0.1,
        now_ns=now + 10_000_000,
    )
    assert action["gripper.pos"] == pytest.approx(-138.2)
    assert status.gripper_target_deg == pytest.approx(0.0)
    assert status.gripper_command_deg == pytest.approx(-138.2)


def test_single_invalid_feedback_enters_hold_and_recovers_without_command_jump() -> None:
    worker = ImmediateIKWorker()
    controller = FullBodyQPIKController(
        FakeKinematics(),
        xr_to_base_rotation=XR_TO_BASE,
        config=CartesianControlConfig(
            stale_timeout_s=1.0,
            feedback_fault_max_consecutive=3,
        ),
        ik_worker=worker,
    )
    now = time.monotonic_ns()
    observation = _observation()
    valid_action, valid_status = controller.update(
        _frame(now), observation, 0.02, now_ns=now
    )
    assert valid_action is not None
    assert valid_status.feedback_valid

    invalid = dict(observation)
    invalid["shoulder_pan.pos"] = float("nan")
    held_action, held_status = controller.update(
        _frame(now + 1, grip=1.0), invalid, 0.02, now_ns=now + 1
    )

    assert held_action == pytest.approx(valid_action)
    assert held_status.state is TeleopState.HOLD
    assert not held_status.feedback_valid
    assert held_status.feedback_fault_count == 1
    assert held_status.feedback_fault_reason == "non_finite:shoulder_pan"
    assert not held_status.feedback_abort_requested

    recovered_action, recovered_status = controller.update(
        _frame(now + 2, grip=1.0), observation, 0.02, now_ns=now + 2
    )
    assert recovered_action is not None
    assert recovered_status.state is TeleopState.IDLE
    assert recovered_status.feedback_valid
    assert recovered_status.feedback_fault_count == 0
    assert recovered_action == pytest.approx(valid_action)


def test_consecutive_out_of_limit_feedback_requests_controlled_abort() -> None:
    controller = FullBodyQPIKController(
        FakeKinematics(),
        xr_to_base_rotation=XR_TO_BASE,
        config=CartesianControlConfig(
            stale_timeout_s=1.0,
            feedback_fault_max_consecutive=2,
        ),
        ik_worker=ImmediateIKWorker(),
    )
    now = time.monotonic_ns()
    valid_action, _ = controller.update(
        _frame(now), _observation(), 0.02, now_ns=now
    )
    invalid = _observation(q_deg=(200.0, -60.0, -70.0, 0.0, 0.0, 0.0))

    first_action, first = controller.update(
        _frame(now + 1), invalid, 0.02, now_ns=now + 1
    )
    second_action, second = controller.update(
        _frame(now + 2), invalid, 0.02, now_ns=now + 2
    )

    assert first_action == pytest.approx(valid_action)
    assert second_action == pytest.approx(valid_action)
    assert first.state is TeleopState.HOLD
    assert not first.feedback_abort_requested
    assert second.feedback_abort_requested
    assert second.feedback_fault_count == 2
    assert second.feedback_fault_reason.startswith("outside_limits:shoulder_pan=")
    rendered = _status_line(second, second_action)
    assert "state=hold" in rendered
    assert "feedback=HOLD 2 reason=outside_limits:" in rendered


def test_invalid_feedback_before_first_valid_sample_does_not_invent_a_command() -> None:
    controller = FullBodyQPIKController(
        FakeKinematics(),
        xr_to_base_rotation=XR_TO_BASE,
        config=CartesianControlConfig(feedback_fault_max_consecutive=2),
        ik_worker=ImmediateIKWorker(),
    )
    now = time.monotonic_ns()
    invalid = _observation()
    invalid["wrist_yaw.pos"] = float("inf")

    action, status = controller.update(
        _frame(now), invalid, 0.02, now_ns=now
    )

    assert action is None
    assert status.state is TeleopState.HOLD
    assert status.feedback_fault_count == 1
    assert not status.feedback_abort_requested


def test_feedback_hold_action_uses_current_finite_values_and_per_joint_fallback() -> None:
    observation = _observation()
    fallback = dict(observation)
    fallback["wrist_yaw.pos"] = 12.0
    observation["shoulder_pan.pos"] = 3.0
    observation["wrist_yaw.pos"] = float("nan")

    action = _feedback_hold_action(observation, fallback)

    assert action is not None
    assert action["shoulder_pan.pos"] == pytest.approx(3.0)
    assert action["wrist_yaw.pos"] == pytest.approx(12.0)


def test_feedback_hold_send_bypasses_relative_clamp_only_for_the_send() -> None:
    class FakeRobot:
        def __init__(self) -> None:
            self.config = type("Config", (), {"max_relative_target": 5.0})()
            self.limit_seen: float | None = 0.0

        def send_action(self, action):
            self.limit_seen = self.config.max_relative_target
            return dict(action)

    robot = FakeRobot()
    action = _observation()

    sent = _send_feedback_hold_action(robot, action)

    assert sent == action
    assert robot.limit_seen is None
    assert robot.config.max_relative_target == pytest.approx(5.0)


def test_wrist_and_gripper_cli_options_defaults_and_positive_validation() -> None:
    parser = _parser()
    help_text = parser.format_help()
    assert "--wrist-speed-rad-s" in help_text
    assert "--wrist-acceleration-rad-s2" in help_text
    assert "--wrist-relative-target-deg" in help_text
    assert "--gripper-relative-target-deg" in help_text

    defaults = parser.parse_args([])
    assert defaults.wrist_speed_rad_s == pytest.approx(12.0)
    assert defaults.wrist_acceleration_rad_s2 == pytest.approx(60.0)
    assert defaults.wrist_relative_target_deg == pytest.approx(20.0)
    assert defaults.gripper_relative_target_deg is None
    assert defaults.gripper_open_deg == pytest.approx(-180.0)
    assert defaults.gripper_closed_deg == pytest.approx(0.0)
    _validate_args(defaults)

    for option in (
        "--wrist-speed-rad-s",
        "--wrist-acceleration-rad-s2",
        "--wrist-relative-target-deg",
        "--gripper-relative-target-deg",
    ):
        invalid = parser.parse_args([option, "0"])
        with pytest.raises(ValueError, match="must be positive"):
            _validate_args(invalid)


def test_adaptive_qp_cli_defaults_modes_and_validation() -> None:
    parser = _parser()
    help_text = parser.format_help()
    for option in (
        "--ik-mode",
        "--qp-damping-min",
        "--qp-damping-max",
        "--qp-orientation-cost-min",
        "--singularity-threshold",
        "--singularity-critical-threshold",
        "--singularity-characteristic-length-m",
        "--qp-position-gain",
        "--qp-orientation-gain",
        "--arm-command-lookahead-ms",
        "--wrist-command-lookahead-ms",
    ):
        assert option in help_text

    defaults = parser.parse_args([])
    assert defaults.ik_mode == "pose"
    assert defaults.qp_damping == pytest.approx(1e-3)
    assert defaults.qp_damping_max == pytest.approx(0.1)
    assert defaults.qp_orientation_cost == pytest.approx(2.0)
    assert defaults.qp_orientation_cost_min == pytest.approx(0.05)
    assert defaults.singularity_threshold == pytest.approx(0.08)
    assert defaults.singularity_critical_threshold == pytest.approx(0.02)
    assert defaults.singularity_characteristic_length_m == pytest.approx(0.3)
    assert defaults.qp_position_gain == pytest.approx(10.0)
    assert defaults.qp_orientation_gain == pytest.approx(8.0)
    assert defaults.arm_command_lookahead_ms == pytest.approx(50.0)
    assert defaults.wrist_command_lookahead_ms == pytest.approx(25.0)
    _validate_args(defaults)

    position = parser.parse_args(["--ik-mode", "position"])
    _validate_args(position)
    assert position.ik_mode == "position"

    invalid = parser.parse_args(
        [
            "--singularity-threshold",
            "0.02",
            "--singularity-critical-threshold",
            "0.02",
        ]
    )
    with pytest.raises(ValueError, match="invalid QP parameters"):
        _validate_args(invalid)

    for option in (
        "--qp-position-gain",
        "--qp-orientation-gain",
        "--arm-command-lookahead-ms",
        "--wrist-command-lookahead-ms",
    ):
        invalid = parser.parse_args([option, "0"])
        with pytest.raises(ValueError, match="invalid QP parameters"):
            _validate_args(invalid)


def test_follower_wrist_limits_use_scalar_fallback_or_complete_motor_dict() -> None:
    velocity = _follower_pos_vel_velocity(5.5, 12.0, 3000.0)
    assert velocity == pytest.approx(
        [*([np.rad2deg(5.5)] * 3), *([np.rad2deg(12.0)] * 3), 3000.0]
    )
    assert _follower_relative_target(6.0, 6.0) == pytest.approx(6.0)
    assert _follower_relative_target(6.0, 20.0) == {
        "shoulder_pan": 6.0,
        "shoulder_lift": 6.0,
        "elbow_flex": 6.0,
        "wrist_flex": 20.0,
        "wrist_yaw": 20.0,
        "wrist_roll": 20.0,
        "gripper": 6.0,
    }
    assert _follower_relative_target(6.0, 6.0, 6.0) == pytest.approx(6.0)
    assert _follower_relative_target(6.0, 20.0, 25.0) == {
        "shoulder_pan": 6.0,
        "shoulder_lift": 6.0,
        "elbow_flex": 6.0,
        "wrist_flex": 20.0,
        "wrist_yaw": 20.0,
        "wrist_roll": 20.0,
        "gripper": 25.0,
    }
    assert _follower_relative_target(6.0, 6.0, 25.0) == {
        "shoulder_pan": 6.0,
        "shoulder_lift": 6.0,
        "elbow_flex": 6.0,
        "wrist_flex": 6.0,
        "wrist_yaw": 6.0,
        "wrist_roll": 6.0,
        "gripper": 25.0,
    }


def test_controller_passes_full_body_motion_limits(monkeypatch) -> None:
    shape_calls: list[tuple[np.ndarray, np.ndarray]] = []
    bound_calls: list[np.ndarray] = []
    gripper_bound_calls: list[float] = []
    original_shape = cartesian_controller_module.shape_joint_position_command
    original_bound = cartesian_controller_module.bound_position_command_to_feedback

    def recording_shape(**kwargs):
        if np.asarray(kwargs["previous_position"]).shape == (6,):
            shape_calls.append(
                (
                    np.asarray(kwargs["max_speed"], dtype=float).copy(),
                    np.asarray(kwargs["max_acceleration"], dtype=float).copy(),
                )
            )
        return original_shape(**kwargs)

    def recording_bound(command_position, feedback_position, max_error, **kwargs):
        if np.asarray(command_position).shape == (6,):
            bound_calls.append(np.asarray(max_error, dtype=float).copy())
        elif np.asarray(command_position).shape == (1,):
            gripper_bound_calls.append(float(max_error))
        return original_bound(command_position, feedback_position, max_error, **kwargs)

    monkeypatch.setattr(
        cartesian_controller_module, "shape_joint_position_command", recording_shape
    )
    monkeypatch.setattr(
        cartesian_controller_module, "bound_position_command_to_feedback", recording_bound
    )
    controller = FullBodyQPIKController(
        FakeKinematics(),
        xr_to_base_rotation=XR_TO_BASE,
        config=CartesianControlConfig(
            stale_timeout_s=1.0,
            max_joint_speed_rad_s=5.5,
            max_joint_acceleration_rad_s2=20.0,
            wrist_speed_rad_s=12.0,
            wrist_acceleration_rad_s2=60.0,
            max_command_feedback_error_deg=5.4,
            wrist_command_feedback_error_deg=18.0,
            gripper_command_feedback_error_deg=22.5,
        ),
        ik_worker=ImmediateIKWorker(),
    )
    now = time.monotonic_ns()
    controller.update(_frame(now), _observation(), 0.02, now_ns=now)

    assert shape_calls[-1][0] == pytest.approx([5.5, 5.5, 5.5, 12.0, 12.0, 12.0])
    assert shape_calls[-1][1] == pytest.approx([20.0, 20.0, 20.0, 60.0, 60.0, 60.0])
    assert bound_calls[-1] == pytest.approx(
        np.deg2rad([5.4, 5.4, 5.4, 18.0, 18.0, 18.0])
    )
    assert gripper_bound_calls[-1] == pytest.approx(22.5)


def test_controller_wrist_limit_none_falls_back_without_behavior_change() -> None:
    common = dict(
        position_filter_hz=0.0,
        orientation_filter_hz=0.0,
        position_deadband_m=0.0,
        orientation_deadband_rad=0.0,
        stale_timeout_s=1.0,
        max_joint_speed_rad_s=0.4,
        max_joint_acceleration_rad_s2=1.0,
        max_command_feedback_error_deg=1.8,
    )
    fallback = FullBodyQPIKController(
        FakeKinematics(),
        xr_to_base_rotation=XR_TO_BASE,
        config=CartesianControlConfig(**common),
        ik_worker=ImmediateIKWorker(),
    )
    explicit = FullBodyQPIKController(
        FakeKinematics(),
        xr_to_base_rotation=XR_TO_BASE,
        config=CartesianControlConfig(
            **common,
            wrist_speed_rad_s=0.4,
            wrist_acceleration_rad_s2=1.0,
            wrist_command_feedback_error_deg=1.8,
        ),
        ik_worker=ImmediateIKWorker(),
    )
    now = time.monotonic_ns()
    observation = _observation()
    frames = (
        _frame(now),
        _frame(now + 10_000_000, grip=1.0),
        _frame(
            now + 20_000_000,
            position=(0.1, 0.0, 0.0),
            quaternion=Rotation.from_rotvec([0.2, 0.1, -0.1]).as_quat(),
            grip=1.0,
        ),
    )
    fallback_result = None
    explicit_result = None
    for frame in frames:
        fallback_result = fallback.update(
            frame, observation, 0.02, now_ns=frame.received_monotonic_ns
        )
        explicit_result = explicit.update(
            frame, observation, 0.02, now_ns=frame.received_monotonic_ns
        )

    assert fallback_result is not None
    assert explicit_result is not None
    fallback_action, fallback_status = fallback_result
    explicit_action, explicit_status = explicit_result
    assert fallback_action == explicit_action
    np.testing.assert_array_equal(fallback_status.target_deg, explicit_status.target_deg)
    np.testing.assert_array_equal(fallback_status.command_deg, explicit_status.command_deg)


def test_reference_initial_pose_maps_q2_q3_to_dm_signs_and_moves_with_limits() -> None:
    lower = np.array([-2.8, -3.14, -3.14, -1.0, -1.0, -1.0])
    upper = np.array([2.8, 0.0, 0.0, 1.0, 1.0, 1.0])
    target = reference_initial_q_to_dm(
        [0.0, 0.8, 0.8, 0.0, 0.0, 0.0],
        lower_limit_rad=lower,
        upper_limit_rad=upper,
    )
    assert target == pytest.approx([0.0, -0.8, -0.8, 0.0, 0.0, 0.0])

    mover = StartupPoseMover(
        target,
        lower_limit_rad=lower,
        upper_limit_rad=upper,
        max_speed_rad_s=0.4,
        max_acceleration_rad_s2=1.0,
        tolerance_rad=0.01,
        settle_samples=2,
    )
    actual = np.array([0.0, -0.4, -0.4, 0.0, 0.0, 0.0])
    first = mover.update(actual, 0.1)
    assert first.command_rad[1:3] == pytest.approx([-0.41, -0.41])
    assert not first.done
    status = first
    for _ in range(100):
        status = mover.update(status.command_rad, 0.1)
        if status.done:
            break
    assert status.done
    assert status.command_rad == pytest.approx(target)


def test_initial_pose_motion_holds_measured_gripper_position() -> None:
    class Robot:
        def __init__(self) -> None:
            self.actions: list[dict[str, float]] = []

        def get_observation(self) -> dict[str, float]:
            return _observation(
                q_deg=(0.0, -45.0, -45.0, 0.0, 0.0, 0.0),
                gripper=-37.0,
            )

        def send_action(self, action: dict[str, float]) -> dict[str, float]:
            self.actions.append(action.copy())
            return action

    robot = Robot()
    target = np.deg2rad([0.0, -45.0, -45.0, 0.0, 0.0, 0.0])
    args = SimpleNamespace(
        max_joint_speed_rad_s=0.4,
        max_joint_acceleration_rad_s2=1.0,
        initial_move_tolerance_deg=2.0,
        max_relative_target_deg=20.0,
        initial_move_timeout=1.0,
        initial_stall_timeout=0.5,
        status_rate=1000.0,
        fps=1000.0,
        gripper_open_deg=-180.0,
    )

    reached = _move_to_initial_pose(
        robot,
        target_rad=target,
        lower_limit_rad=np.full(6, -3.0),
        upper_limit_rad=np.full(6, 3.0),
        args=args,
        should_stop=lambda: False,
    )

    assert reached
    assert len(robot.actions) == 3
    assert all(action["gripper.pos"] == -37.0 for action in robot.actions)


def test_startup_pose_waits_for_feedback_without_reaching_follower_clamp() -> None:
    lower = np.full(6, -3.0)
    upper = np.array([3.0, 0.0, 0.0, 3.0, 3.0, 3.0])
    max_tracking_error = np.deg2rad(1.8)
    mover = StartupPoseMover(
        np.array([0.0, -0.8, -0.8, 0.0, 0.0, 0.0]),
        lower_limit_rad=lower,
        upper_limit_rad=upper,
        max_speed_rad_s=0.4,
        max_acceleration_rad_s2=1.0,
        tolerance_rad=0.01,
        max_command_feedback_error_rad=max_tracking_error,
    )
    stalled_feedback = np.zeros(6)
    for _ in range(100):
        status = mover.update(stalled_feedback, 0.1)
        assert np.max(np.abs(status.command_rad - stalled_feedback)) <= (
            max_tracking_error + 1e-12
        )
    assert status.command_rad[1:3] == pytest.approx(
        [-max_tracking_error, -max_tracking_error]
    )

    moving_feedback = np.deg2rad([0.0, -1.0, -1.0, 0.0, 0.0, 0.0])
    resumed = mover.update(moving_feedback, 0.1)
    assert resumed.command_rad[1] < status.command_rad[1]
    assert resumed.command_rad[2] < status.command_rad[2]


def test_feedback_bound_stays_inside_hard_joint_limits() -> None:
    bounded = bound_position_command_to_feedback(
        np.array([-2.0, 2.0]),
        np.array([-0.1, -0.1]),
        0.2,
        lower_limit=np.array([-1.0, -1.0]),
        upper_limit=np.array([0.0, 0.0]),
    )
    assert bounded == pytest.approx([-0.3, 0.0])


def test_reference_initial_pose_rejects_target_outside_dm_limits() -> None:
    with pytest.raises(ValueError, match=r"joint\(s\): 2"):
        reference_initial_q_to_dm(
            [0.0, -0.5, 0.8, 0.0, 0.0, 0.0],
            lower_limit_rad=np.full(6, -3.0),
            upper_limit_rad=np.array([3.0, 0.0, 0.0, 3.0, 3.0, 3.0]),
        )


def test_startup_pose_accepts_one_degree_zero_feedback_tolerance() -> None:
    lower = np.full(6, -3.0)
    upper = np.array([3.0, 0.0, 0.0, 3.0, 3.0, 3.0])
    mover = StartupPoseMover(
        np.array([0.0, -0.8, -0.8, 0.0, 0.0, 0.0]),
        lower_limit_rad=lower,
        upper_limit_rad=upper,
        max_speed_rad_s=0.4,
        max_acceleration_rad_s2=1.0,
        tolerance_rad=0.01,
    )
    status = mover.update(
        np.deg2rad([0.0, -45.0, 0.5, 0.0, 0.0, 0.0]), 0.1
    )
    assert status.command_rad[2] <= 0.0

    with pytest.raises(RuntimeError, match=r"joint3=1\.20deg"):
        mover.update(np.deg2rad([0.0, -45.0, 1.2, 0.0, 0.0, 0.0]), 0.1)


def test_closed_loop_holds_feedback_outside_dm_kinematic_limits() -> None:
    controller = FullBodyQPIKController(
        FakeKinematics(),
        xr_to_base_rotation=XR_TO_BASE,
        ik_worker=ImmediateIKWorker(),
    )
    now = time.monotonic_ns()
    action, status = controller.update(
        _frame(now),
        _observation(q_deg=(0.0, -190.0, -70.0, 0.0, 0.0, 0.0)),
        0.02,
        now_ns=now,
    )
    assert action is None
    assert status.state is TeleopState.HOLD
    assert "shoulder_lift=-190.00deg" in status.feedback_fault_reason


def test_closed_loop_accepts_small_dm_zero_error_but_commands_hard_limit() -> None:
    controller = FullBodyQPIKController(
        FakeKinematics(),
        xr_to_base_rotation=XR_TO_BASE,
        ik_worker=ImmediateIKWorker(),
    )
    now = time.monotonic_ns()
    action, _ = controller.update(
        _frame(now),
        _observation(q_deg=(0.0, -60.0, 0.5, 0.0, 0.0, 0.0)),
        0.02,
        now_ns=now,
    )
    assert action["elbow_flex.pos"] == pytest.approx(0.0)


def test_closed_loop_holds_feedback_angle_beyond_dm_zero_tolerance() -> None:
    controller = FullBodyQPIKController(
        FakeKinematics(),
        xr_to_base_rotation=XR_TO_BASE,
        ik_worker=ImmediateIKWorker(),
    )
    now = time.monotonic_ns()
    action, status = controller.update(
        _frame(now),
        _observation(q_deg=(0.0, -60.0, 1.2, 0.0, 0.0, 0.0)),
        0.02,
        now_ns=now,
    )
    assert action is None
    assert status.state is TeleopState.HOLD
    assert "elbow_flex=1.20deg" in status.feedback_fault_reason
    assert "1.0deg" in status.feedback_fault_reason


def test_tracking_loss_holds_actual_and_new_stream_requires_release() -> None:
    worker = ImmediateIKWorker()
    controller = FullBodyQPIKController(
        FakeKinematics(),
        xr_to_base_rotation=XR_TO_BASE,
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
    observation = _observation()
    controller.update(_frame(now), observation, 0.02, now_ns=now)
    controller.update(
        _frame(now + 10_000_000, grip=1.0), observation, 0.02, now_ns=now + 10_000_000
    )
    controller.update(
        _frame(now + 20_000_000, position=(0.1, 0.0, 0.0), grip=1.0),
        observation,
        0.02,
        now_ns=now + 20_000_000,
    )
    moved_count = worker.submitted

    held, status = controller.update(
        _frame(now + 30_000_000, grip=1.0, tracking=False),
        observation,
        0.02,
        now_ns=now + 30_000_000,
    )
    assert status.state is TeleopState.STALE
    assert worker.submitted == moved_count
    assert [held[f"{name}.pos"] for name in ARM_JOINT_NAMES] == pytest.approx(
        [0.0, -60.0, -70.0, 0.0, 0.0, 0.0]
    )

    _, status = controller.update(
        _frame(now + 40_000_000, grip=1.0, stream_epoch=2),
        observation,
        0.02,
        now_ns=now + 40_000_000,
    )
    assert status.state is TeleopState.IDLE
    assert worker.submitted == moved_count
    controller.update(
        _frame(now + 50_000_000, grip=0.0, stream_epoch=2),
        observation,
        0.02,
        now_ns=now + 50_000_000,
    )
    _, status = controller.update(
        _frame(now + 60_000_000, grip=1.0, stream_epoch=2),
        observation,
        0.02,
        now_ns=now + 60_000_000,
    )
    assert status.state is TeleopState.ACTIVE
    assert worker.submitted == moved_count
    controller.update(
        _frame(now + 70_000_000, grip=1.0, stream_epoch=2),
        observation,
        0.02,
        now_ns=now + 70_000_000,
    )
    assert worker.submitted == moved_count + 1


def test_packaged_dm_urdf_documents_physical_wrist_axis_signs() -> None:
    pytest.importorskip("pinocchio")
    from lerobot_teleoperator_rebot_vr.kinematics import B601Kinematics

    kinematics = B601Kinematics()
    try:
        q_reference = np.array([0.0, -0.8, -0.8, 0.0, 0.0, 0.0])
        _, rotation_reference = kinematics.forward_kinematics(q_reference)
        expected_positive_pyr_axes_base = XR_TO_BASE @ np.eye(3)
        step_rad = 1e-5
        for wrist_index in range(3):
            moved = q_reference.copy()
            moved[3 + wrist_index] -= step_rad
            _, rotation_moved = kinematics.forward_kinematics(moved)
            physical_axis_base = (
                Rotation.from_matrix(
                    rotation_moved @ rotation_reference.T
                ).as_rotvec()
                / step_rad
            )
            assert physical_axis_base == pytest.approx(
                expected_positive_pyr_axes_base[:, wrist_index], abs=1e-4
            )
    finally:
        kinematics.close()


def test_packaged_dm_kinematics_uses_independent_pinocchio_data_per_thread() -> None:
    pytest.importorskip("pinocchio")
    from lerobot_teleoperator_rebot_vr.kinematics import B601Kinematics

    kinematics = B601Kinematics()
    main_data = kinematics._thread_data()
    worker_data: list[object] = []
    thread = threading.Thread(target=lambda: worker_data.append(kinematics._thread_data()))
    thread.start()
    thread.join()
    assert worker_data[0] is not main_data

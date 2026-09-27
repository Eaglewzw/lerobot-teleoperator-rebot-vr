from __future__ import annotations

import itertools

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from lerobot_teleoperator_rebot_vr.config_rebot_vr import DEFAULT_BASE_T_ANCHOR
from lerobot_teleoperator_rebot_vr.constants import (
    FOLLOWER_LOWER_DEG,
    FOLLOWER_UPPER_DEG,
)
from lerobot_teleoperator_rebot_vr.ik.kinematics import B601Kinematics
from lerobot_teleoperator_rebot_vr.ik.split_solver import SplitIKSolver


@pytest.fixture()
def split_model():
    model = B601Kinematics()
    lower = np.maximum(model.lower_position_limit, np.deg2rad(FOLLOWER_LOWER_DEG))
    upper = np.minimum(model.upper_position_limit, np.deg2rad(FOLLOWER_UPPER_DEG))
    solver = SplitIKSolver(
        model,
        joint_lower_limit_rad=lower,
        joint_upper_limit_rad=upper,
        joint_limit_margin_rad=0.0,
        max_solve_time_ms=30.0,
    )
    yield model, solver
    model.close()


def _arguments(model: B601Kinematics, q_reference: np.ndarray) -> dict[str, object]:
    position, _ = model.wrist_anchor_pose(q_reference)
    _, rotation = model.forward_kinematics(q_reference)
    return {
        "target_position": position,
        "target_rotation": rotation,
        "q_actual": q_reference,
        "dq_previous": np.zeros(6),
        "dt": 0.02,
        "q_nominal": q_reference,
        "q_seed": q_reference,
        "max_joint_speed": np.full(6, 10.0),
        "max_joint_acceleration": np.full(6, 1000.0),
    }


def test_joint4_anchor_position_depends_only_on_q1_to_q3(split_model) -> None:
    model, _ = split_model
    q = np.array([0.2, -0.8, -0.7, 0.3, -0.2, 0.4])
    moved_wrist = q.copy()
    moved_wrist[3:] = [-0.5, 0.6, -0.7]

    position, _ = model.wrist_anchor_pose(q)
    moved_position, _ = model.wrist_anchor_pose(moved_wrist)
    jacobian = model.wrist_anchor_jacobian(q)

    assert moved_position == pytest.approx(position, abs=1e-10)
    assert jacobian[:3, 3:] == pytest.approx(np.zeros((3, 3)), abs=1e-10)


def test_split_solver_separates_translation_and_rotation(split_model) -> None:
    model, solver = split_model
    q = np.array([0.0, -0.8, -0.8, 0.2, -0.1, 0.3])
    arguments = _arguments(model, q)

    neutral = solver.solve(**arguments)
    assert neutral.success
    assert neutral.joint_velocity_rad_s == pytest.approx(np.zeros(6), abs=1e-5)

    translated_arguments = dict(arguments)
    translated_arguments["target_position"] = (
        np.asarray(arguments["target_position"]) + np.array([0.005, 0.0, 0.0])
    )
    translated = solver.solve(**translated_arguments)
    assert translated.success
    assert np.linalg.norm(translated.joint_velocity_rad_s[:3]) > 1e-4
    assert translated.joint_velocity_rad_s[3:] == pytest.approx(
        np.zeros(3), abs=1e-5
    )

    xr_to_base = np.asarray(DEFAULT_BASE_T_ANCHOR, dtype=np.float64)[:3, :3]
    delta_world = (
        xr_to_base
        @ Rotation.from_rotvec([0.2, 0.0, 0.0]).as_matrix()
        @ xr_to_base.T
    )
    rotated_arguments = dict(arguments)
    rotated_arguments["target_rotation"] = (
        delta_world @ np.asarray(arguments["target_rotation"])
    )
    rotated = solver.solve(**rotated_arguments)
    assert rotated.success
    assert rotated.joint_velocity_rad_s[:3] == pytest.approx(
        np.zeros(3), abs=1e-8
    )
    assert abs(rotated.joint_velocity_rad_s[3]) > 0.1


def test_split_wrist_target_does_not_compensate_shoulder_motion(split_model) -> None:
    model, solver = split_model
    q_reference = np.array([0.0, -0.8, -0.8, 0.1, -0.2, 0.3])
    arguments = _arguments(model, q_reference)
    xr_to_base = np.asarray(DEFAULT_BASE_T_ANCHOR, dtype=np.float64)[:3, :3]
    delta_world = (
        xr_to_base
        @ Rotation.from_rotvec([0.1, -0.15, 0.05]).as_matrix()
        @ xr_to_base.T
    )
    target_rotation = delta_world @ np.asarray(arguments["target_rotation"])

    first_target, _, _ = solver._wrist_target(
        target_rotation, q_reference, q_reference[3:]
    )
    shoulder_moved = q_reference.copy()
    shoulder_moved[:3] += [0.25, -0.1, 0.08]
    second_target, _, _ = solver._wrist_target(
        target_rotation, q_reference, shoulder_moved[3:]
    )

    assert second_target == pytest.approx(first_target, abs=1e-12)


def test_split_wrist_decomposition_is_derived_from_urdf(split_model) -> None:
    model, solver = split_model
    q_reference = np.array([0.1, -0.9, -0.6, 0.0, 0.0, 0.0])
    desired = q_reference.copy()
    desired[3:] = [0.45, -0.35, 0.55]
    _, desired_rotation = model.forward_kinematics(desired)

    target, clip_rad, _ = solver._wrist_target(
        desired_rotation, q_reference, q_reference[3:]
    )

    assert solver._wrist_sequence == "ZYX"
    assert solver._wrist_axis_signs == pytest.approx(np.ones(3))
    assert clip_rad == pytest.approx(0.0)
    assert target == pytest.approx(desired[3:], abs=2e-5)


def test_split_reports_wrist_limit_clipping(split_model) -> None:
    model, solver = split_model
    q_reference = np.array([0.0, -0.8, -0.8, 0.0, 0.0, 0.0])
    desired = q_reference.copy()
    desired[3] = 2.0
    position, _ = model.wrist_anchor_pose(q_reference)
    _, target_rotation = model.forward_kinematics(desired)
    arguments = _arguments(model, q_reference)
    arguments["target_position"] = position
    arguments["target_rotation"] = target_rotation

    result = solver.solve(**arguments)

    assert result.success
    assert result.reason == "wrist_clipped"
    assert result.wrist_clip_rad > 0.1


def _scalar_wrist_choice(solver, candidates, seed):
    """Pre-optimization scoring oracle, intentionally evaluated one at a time."""
    lower = solver.lower_position_limit[3:] + solver.joint_limit_margin_rad
    upper = solver.upper_position_limit[3:] - solver.joint_limit_margin_rad
    raw = min(
        candidates,
        key=lambda candidate: (
            float(np.linalg.norm(candidate - np.clip(candidate, lower, upper))),
            float(np.linalg.norm(candidate - seed)),
        ),
    )
    clipped = np.clip(raw, lower, upper)
    return clipped, float(np.max(np.abs(raw - clipped)))


@pytest.mark.parametrize("margin", [0.0, np.deg2rad(2.0)])
def test_batched_wrist_scores_match_scalar_at_random_and_singular_poses(
    split_model, monkeypatch, margin
) -> None:
    model, solver = split_model
    solver.joint_limit_margin_rad = margin
    reference = np.array([0.0, -0.8, -0.8, 0.0, 0.0, 0.0])
    _, anchor = model.wrist_anchor_pose(reference)
    rng = np.random.default_rng(24)
    rotations = list(Rotation.random(600, random_state=rng).as_matrix())
    for first, middle, last in itertools.product(
        [-np.pi, 0.0, np.pi],
        [-np.pi / 2, -np.pi / 2 + 1e-7, np.pi / 2 - 1e-7, np.pi / 2],
        [-np.pi, 0.0, np.pi],
    ):
        rotations.append(
            anchor
            @ Rotation.from_euler(solver._wrist_sequence, [first, middle, last]).as_matrix()
            @ solver._wrist_zero_rotation
        )
    generate = solver._equivalent_wrist_candidates
    captured = []

    def capture(*args):
        candidates = generate(*args)
        captured[:] = candidates
        return candidates

    monkeypatch.setattr(solver, "_equivalent_wrist_candidates", capture)
    for index, rotation in enumerate(rotations):
        lower, upper = solver.lower_position_limit[3:], solver.upper_position_limit[3:]
        seed = [lower, upper, rng.uniform(lower, upper)][index % 3]
        target, clip, _ = solver._wrist_target(rotation, reference, seed)
        expected, expected_clip = _scalar_wrist_choice(solver, captured, seed)
        np.testing.assert_array_equal(target, expected)
        assert clip == expected_clip


@pytest.mark.parametrize(
    "candidates, expected",
    [
        ([[0.2, 0, 0], [-0.2, 0, 0]], [0.2, 0, 0]),
        ([[-0.2, 0, 0], [0.2, 0, 0]], [-0.2, 0, 0]),
        ([[0.4, 0, 0], [0.1, 0, 0]], [0.1, 0, 0]),
        # An in-range candidate wins even when it is farther from the seed.
        ([[0.51, 0, 0], [0.4, 0.4, 0.4]], [0.4, 0.4, 0.4]),
    ],
)
def test_batched_wrist_score_priority_and_stable_ties(
    split_model, monkeypatch, candidates, expected
) -> None:
    _, solver = split_model
    solver.lower_position_limit[3:] = -0.5
    solver.upper_position_limit[3:] = 0.5
    monkeypatch.setattr(
        solver, "_equivalent_wrist_candidates",
        lambda *args: [np.asarray(candidate, dtype=float) for candidate in candidates],
    )
    target, _, _ = solver._wrist_target(np.eye(3), np.zeros(6), np.zeros(3))
    np.testing.assert_array_equal(target, expected)


def test_batched_wrist_target_is_continuous_across_euler_wrap(split_model) -> None:
    model, solver = split_model
    reference = np.array([0.0, -0.8, -0.8, 0.0, 0.0, 0.0])
    solver.lower_position_limit[3:] = -2 * np.pi
    solver.upper_position_limit[3:] = 2 * np.pi
    _, anchor = model.wrist_anchor_pose(reference)
    seed = np.array([np.pi - 0.02, 0.2, 0.1]) / solver._wrist_axis_signs
    for first in np.linspace(np.pi - 0.01, np.pi + 0.01, 21):
        angles = np.array([first, 0.2, 0.1])
        rotation = (
            anchor @ Rotation.from_euler(solver._wrist_sequence, angles).as_matrix()
            @ solver._wrist_zero_rotation
        )
        target, clip, _ = solver._wrist_target(rotation, reference, seed)
        np.testing.assert_allclose(target, angles / solver._wrist_axis_signs, atol=1e-12)
        assert np.max(np.abs(target - seed)) < 0.011
        assert clip == 0.0
        seed = target

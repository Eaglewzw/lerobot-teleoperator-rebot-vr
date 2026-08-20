from __future__ import annotations

import pytest

from lerobot_teleoperator_rebot_vr.cartesian_controller import ARM_JOINT_NAMES
from lerobot_teleoperator_rebot_vr.gripper_test import (
    build_parser,
    feedback_hold_action,
    run_gripper_test,
    validate_args,
)


def _observation(gripper_deg: float = -40.0) -> dict[str, float]:
    result = {
        f"{name}.pos": float(index + 1)
        for index, name in enumerate(ARM_JOINT_NAMES)
    }
    result["gripper.pos"] = gripper_deg
    return result


class FakeRobot:
    def __init__(self, gripper_deg: float = -40.0) -> None:
        self.observation = _observation(gripper_deg)
        self.actions: list[dict[str, float]] = []

    def get_observation(self) -> dict[str, float]:
        return self.observation.copy()

    def send_action(self, action: dict[str, float]) -> dict[str, float]:
        self.actions.append(action.copy())
        self.observation["gripper.pos"] = action["gripper.pos"]
        return action.copy()


def test_direct_gripper_cli_validates_absolute_target() -> None:
    parser = build_parser()
    args = parser.parse_args(["--target-deg", "-100"])
    validate_args(args)
    assert args.target_deg == pytest.approx(-100.0)

    with pytest.raises(ValueError, match="within"):
        validate_args(parser.parse_args(["--target-deg", "1"]))


def test_feedback_hold_action_only_replaces_gripper_position() -> None:
    observation = _observation(-40.0)
    action, actual = feedback_hold_action(observation, -100.0)
    assert actual == pytest.approx(-40.0)
    assert [action[f"{name}.pos"] for name in ARM_JOINT_NAMES] == pytest.approx(
        [observation[f"{name}.pos"] for name in ARM_JOINT_NAMES]
    )
    assert action["gripper.pos"] == pytest.approx(-100.0)


def test_direct_gripper_move_reaches_target_without_vr_or_trigger() -> None:
    robot = FakeRobot()
    result = run_gripper_test(
        robot,
        target_deg=-100.0,
        speed_deg_s=100_000.0,
        acceleration_deg_s2=1_000_000_000.0,
        relative_target_deg=100.0,
        fps=1_000.0,
        timeout_s=1.0,
        tolerance_deg=0.01,
        settle_samples=1,
        status_rate=1.0,
    )
    assert result.reached
    assert result.actual_deg == pytest.approx(-100.0)
    assert robot.actions[-1]["gripper.pos"] == pytest.approx(-100.0)
    for name in ARM_JOINT_NAMES:
        assert robot.actions[-1][f"{name}.pos"] == pytest.approx(
            robot.observation[f"{name}.pos"]
        )

from __future__ import annotations

import importlib
import sys
import time
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np
import pytest

from lerobot.teleoperators.config import TeleoperatorConfig
from lerobot.teleoperators.utils import make_teleoperator_from_config
from lerobot.utils.import_utils import register_third_party_plugins
from lerobot_teleoperator_rebot_vr import RebotVRTeleop, RebotVRTeleopConfig
from lerobot_teleoperator_rebot_vr.config_rebot_vr import REBOT_JOINTS


def _config(tmp_path, **overrides) -> RebotVRTeleopConfig:
    values = {"calibration_dir": tmp_path}
    values.update(overrides)
    return RebotVRTeleopConfig(**values)


def _raw_action(**overrides) -> dict[str, object]:
    action: dict[str, object] = {
        "grip_pos": np.zeros(3),
        "grip_quat": np.array([0.0, 0.0, 0.0, 1.0]),
        "squeeze": 0.0,
        "trigger": 0.0,
        "is_tracking": True,
        "received_monotonic_ns": time.monotonic_ns(),
        "tracking_timestamp_ns": 1,
        "stream_epoch": 1,
        "side": "right",
        "primary_button": False,
        "secondary_button": False,
    }
    action.update(overrides)
    return action


class FakeController:
    def __init__(self, action: dict[str, object]) -> None:
        self.action = action
        self.connected = False

    @property
    def is_connected(self) -> bool:
        return self.connected

    @property
    def is_tracking(self) -> bool:
        return bool(self.action["is_tracking"])

    def connect(self) -> None:
        self.connected = True

    def latest_sample(self):
        return None

    def get_action(self) -> dict[str, object]:
        return self.action

    def disconnect(self) -> None:
        self.connected = False


class FakeArmController:
    def __init__(self) -> None:
        self.started = False
        self.frames = []

    def start(self) -> None:
        self.started = True

    def stop(self) -> None:
        self.started = False

    def update(self, frame, observation, dt_s):
        self.frames.append((frame, dict(observation), dt_s))
        return dict(observation), None


def _feedback() -> dict[str, float]:
    values = (0.0, -60.0, -70.0, 0.0, 0.0, 0.0, -270.0)
    return {f"{joint}.pos": value for joint, value in zip(REBOT_JOINTS, values)}


def test_config_registration_factory_and_third_party_discovery(tmp_path) -> None:
    assert TeleoperatorConfig._choice_registry["rebot_vr"] is RebotVRTeleopConfig
    device = make_teleoperator_from_config(_config(tmp_path))
    assert isinstance(device, RebotVRTeleop)

    fake_distribution = SimpleNamespace(metadata={"Name": "lerobot_teleoperator_rebot_vr"})
    with patch("importlib.metadata.distributions", return_value=[fake_distribution]):
        register_third_party_plugins()
    assert importlib.import_module("lerobot_teleoperator_rebot_vr") is sys.modules[
        "lerobot_teleoperator_rebot_vr"
    ]


def test_registered_teleop_features_include_required_robot_feedback(tmp_path) -> None:
    teleop = RebotVRTeleop(_config(tmp_path))
    expected = {f"{joint}.pos": float for joint in REBOT_JOINTS}
    assert teleop.action_features == expected
    assert teleop.feedback_features == expected
    assert teleop.is_calibrated


def test_registered_teleop_fails_closed_without_feedback_then_uses_split_controller(
    tmp_path,
) -> None:
    controller = FakeController(
        _raw_action(primary_button=True, secondary_button=True)
    )
    arm_controller = FakeArmController()
    teleop = RebotVRTeleop(
        _config(tmp_path),
        controller=controller,
        arm_controller=arm_controller,
    )
    teleop.connect()
    with pytest.raises(RuntimeError, match="requires current robot feedback"):
        teleop.get_action()

    feedback = _feedback()
    teleop.send_feedback(feedback)
    action = teleop.get_action()
    assert action == feedback
    assert arm_controller.frames[-1][0].primary_button is True
    assert arm_controller.frames[-1][0].secondary_button is True
    teleop.disconnect()
    assert not teleop.is_connected
    assert not arm_controller.started


def test_registered_teleop_rejects_incomplete_or_nonfinite_feedback(tmp_path) -> None:
    teleop = RebotVRTeleop(
        _config(tmp_path),
        controller=FakeController(_raw_action()),
        arm_controller=FakeArmController(),
    )
    teleop.connect()
    with pytest.raises(ValueError, match="seven finite"):
        teleop.send_feedback({})
    invalid = _feedback()
    invalid["elbow_flex.pos"] = float("nan")
    with pytest.raises(ValueError, match="seven finite"):
        teleop.send_feedback(invalid)
    teleop.disconnect()


def test_config_validation(tmp_path) -> None:
    with pytest.raises(ValueError, match="threshold"):
        _config(tmp_path, clutch_threshold=0.5, clutch_release_threshold=0.6)
    with pytest.raises(ValueError, match="vr_backend"):
        _config(tmp_path, vr_backend="unknown")
    with pytest.raises(ValueError, match="initial_q_rad"):
        _config(tmp_path, initial_q_rad=(0.0, 0.8))

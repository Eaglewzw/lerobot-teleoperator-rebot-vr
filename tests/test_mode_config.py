from pathlib import Path

import pytest
import yaml

from lerobot_teleoperator_rebot_vr.runtime.cli import (
    _flatten_mode_config,
    build_parser,
    validate_args,
)


MIT_PARAMETERS = {
    "mit_kp",
    "mit_kd",
    "mit_torque_limit_nm",
    "mit_gravity_scale",
    "mit_gravity_ramp_s",
    "mit_dynamics_urdf",
}


def test_motor_mode_automatically_loads_matching_yaml() -> None:
    parser = build_parser()

    pos_vel = parser.parse_args(["--motor-control-mode", "pos_vel"])
    mit = parser.parse_args(["--motor-control-mode", "mit"])

    assert pos_vel.control_config.name == "pos_vel.yaml"
    assert pos_vel.motor_control_mode == "pos_vel"
    assert pos_vel.max_joint_speed_rad_s == pytest.approx(5.5)
    assert pos_vel.max_joint_acceleration_rad_s2 == pytest.approx(20.0)
    assert pos_vel.wrist_speed_rad_s == pytest.approx(12.0)
    assert pos_vel.wrist_acceleration_rad_s2 == pytest.approx(60.0)
    assert pos_vel.gripper_max_speed_deg_s == pytest.approx(1200.0)
    assert pos_vel.gripper_max_acceleration_deg_s2 == pytest.approx(5000.0)
    assert mit.control_config.name == "mit.yaml"
    assert mit.motor_control_mode == "mit"
    assert mit.mit_kp == pytest.approx([36, 36, 36, 10, 10, 10])
    validate_args(pos_vel)
    validate_args(mit)


def test_explicit_cli_parameter_overrides_mode_yaml() -> None:
    args = build_parser().parse_args(
        ["--motor-control-mode", "mit", "--mit-kp", "20", "20", "20", "5", "5", "5"]
    )

    assert args.mit_kp == pytest.approx([20, 20, 20, 5, 5, 5])


def test_yaml_files_cover_every_applicable_cli_parameter() -> None:
    parser = build_parser()
    destinations = {
        action.dest for action in parser._actions if action.dest != "help"
    }
    config_dir = Path(__file__).parents[1] / "config"

    for mode, excluded in (
        ("pos_vel", {"control_config", *MIT_PARAMETERS}),
        ("mit", {"control_config"}),
    ):
        path = config_dir / f"{mode}.yaml"
        configured = set(
            _flatten_mode_config(yaml.safe_load(path.read_text()), path)
        )
        assert configured == destinations - excluded


def test_custom_config_must_match_selected_mode(tmp_path: Path) -> None:
    config = tmp_path / "wrong.yaml"
    config.write_text("motor_control_mode: pos_vel\n", encoding="utf-8")

    with pytest.raises(SystemExit):
        build_parser().parse_args(
            ["--motor-control-mode", "mit", "--control-config", str(config)]
        )

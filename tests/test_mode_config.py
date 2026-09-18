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
    assert mit.mit_kp == pytest.approx([25, 30, 30, 10, 10, 10])
    assert mit.mit_kd == pytest.approx([5, 5, 4, 0.5, 0.5, 0.5])
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


@pytest.mark.parametrize(
    ("filename", "motor_mode", "ik_mode"),
    (
        ("pos_vel_pose.yaml", "pos_vel", "pose"),
        ("pos_vel_split.yaml", "pos_vel", "split"),
        ("mit_pose.yaml", "mit", "pose"),
        ("mit_split.yaml", "mit", "split"),
    ),
)
def test_named_control_profiles_are_complete_and_loadable(
    filename: str, motor_mode: str, ik_mode: str
) -> None:
    parser = build_parser()
    config_path = Path(__file__).parents[1] / "config" / filename
    destinations = {
        action.dest for action in parser._actions if action.dest != "help"
    }
    excluded = {"control_config"}
    if motor_mode == "pos_vel":
        excluded.update(MIT_PARAMETERS)

    configured = set(
        _flatten_mode_config(yaml.safe_load(config_path.read_text()), config_path)
    )
    assert configured == destinations - excluded

    args = parser.parse_args(
        [
            "--motor-control-mode",
            motor_mode,
            "--control-config",
            str(config_path),
        ]
    )
    assert args.motor_control_mode == motor_mode
    assert args.ik_mode == ik_mode
    validate_args(args)


def test_mit_profiles_load_their_own_motor_settings() -> None:
    config_dir = Path(__file__).parents[1] / "config"
    parser = build_parser()

    for filename in ("mit_pose.yaml", "mit_split.yaml"):
        config_path = config_dir / filename
        configured = yaml.safe_load(config_path.read_text())
        args = parser.parse_args(
            [
                "--motor-control-mode",
                "mit",
                "--control-config",
                str(config_path),
            ]
        )

        assert args.mit_kp == pytest.approx(configured["mit"]["mit_kp"])
        assert args.mit_kd == pytest.approx(configured["mit"]["mit_kd"])
        assert args.mit_torque_limit_nm == pytest.approx(
            configured["mit"]["mit_torque_limit_nm"]
        )
        assert args.mit_gravity_scale == pytest.approx(
            configured["mit"]["mit_gravity_scale"]
        )


@pytest.mark.parametrize("mode", ("pos_vel", "mit"))
def test_explicit_pose_profile_matches_compatibility_default(mode: str) -> None:
    config_dir = Path(__file__).parents[1] / "config"
    default_path = config_dir / f"{mode}.yaml"
    pose_path = config_dir / f"{mode}_pose.yaml"

    assert _flatten_mode_config(
        yaml.safe_load(default_path.read_text()), default_path
    ) == _flatten_mode_config(yaml.safe_load(pose_path.read_text()), pose_path)


def test_custom_config_must_match_selected_mode(tmp_path: Path) -> None:
    config = tmp_path / "wrong.yaml"
    config.write_text("motor_control_mode: pos_vel\n", encoding="utf-8")

    with pytest.raises(SystemExit):
        build_parser().parse_args(
            ["--motor-control-mode", "mit", "--control-config", str(config)]
        )

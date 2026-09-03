"""Command-line parsing and display helpers for the real-robot runner."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

import numpy as np
import yaml

from ..control.startup import DEFAULT_INITIAL_Q_REFERENCE_RAD
from ..control.types import CartesianControlStatus


MODE_CONFIG_FILENAMES = {
    "pos_vel": "pos_vel.yaml",
    "mit": "mit.yaml",
}


def _default_mode_config_path(mode: str) -> Path:
    filename = MODE_CONFIG_FILENAMES[mode]
    candidates = (
        Path(__file__).resolve().parents[3] / "config" / filename,
        Path(sys.prefix)
        / "share"
        / "lerobot_teleoperator_rebot_vr"
        / "config"
        / filename,
        Path.cwd() / "config" / filename,
    )
    for candidate in candidates:
        if candidate.is_file():
            return candidate
    return candidates[1]


def _flatten_mode_config(data: object, path: Path) -> dict[str, object]:
    if not isinstance(data, dict):
        raise ValueError(f"mode config must contain a YAML mapping: {path}")
    flattened: dict[str, object] = {}
    for key, value in data.items():
        if isinstance(value, dict):
            for parameter, parameter_value in value.items():
                normalized = str(parameter).replace("-", "_")
                if normalized in flattened:
                    raise ValueError(
                        f"duplicate mode config parameter '{normalized}' in {path}"
                    )
                flattened[normalized] = parameter_value
        else:
            normalized = str(key).replace("-", "_")
            if normalized in flattened:
                raise ValueError(
                    f"duplicate mode config parameter '{normalized}' in {path}"
                )
            flattened[normalized] = value
    return flattened


class ModeConfigArgumentParser(argparse.ArgumentParser):
    """Argument parser whose defaults come from the selected motor-mode YAML."""

    def parse_args(self, args=None, namespace=None):
        arguments = list(sys.argv[1:] if args is None else args)
        selector = argparse.ArgumentParser(add_help=False)
        selector.add_argument(
            "--motor-control-mode",
            choices=tuple(MODE_CONFIG_FILENAMES),
            default="pos_vel",
        )
        selector.add_argument("--control-config", type=Path, default=None)
        selected, _ = selector.parse_known_args(arguments)
        config_path = (
            selected.control_config
            if selected.control_config is not None
            else _default_mode_config_path(selected.motor_control_mode)
        )

        try:
            with config_path.open("r", encoding="utf-8") as config_file:
                config_defaults = _flatten_mode_config(
                    yaml.safe_load(config_file), config_path
                )
        except (OSError, yaml.YAMLError, ValueError) as exc:
            self.error(f"cannot load control config '{config_path}': {exc}")

        valid_destinations = {
            action.dest for action in self._actions if action.dest != "help"
        }
        unknown = sorted(set(config_defaults) - valid_destinations)
        if unknown:
            self.error(
                f"unknown parameter(s) in control config '{config_path}': "
                + ", ".join(unknown)
            )
        configured_mode = config_defaults.get("motor_control_mode")
        if configured_mode != selected.motor_control_mode:
            self.error(
                f"control config '{config_path}' is for mode {configured_mode!r}, "
                f"not {selected.motor_control_mode!r}"
            )

        original_defaults = {action.dest: action.default for action in self._actions}
        original_parser_defaults = self._defaults.copy()
        try:
            self.set_defaults(**config_defaults)
            parsed = super().parse_args(arguments, namespace)
            parsed.control_config = config_path
            return parsed
        finally:
            for action in self._actions:
                action.default = original_defaults[action.dest]
            self._defaults.clear()
            self._defaults.update(original_parser_defaults)


def build_parser(description: str | None = None) -> argparse.ArgumentParser:
    parser = ModeConfigArgumentParser(description=description)
    robot = parser.add_argument_group("robot")
    robot.add_argument(
        "--control-config",
        type=Path,
        default=None,
        metavar="PATH",
        help=(
            "control YAML; defaults to config/pos_vel.yaml or config/mit.yaml "
            "according to --motor-control-mode"
        ),
    )
    robot.add_argument("--robot-port", default="/dev/ttyACM0")
    robot.add_argument("--robot-id", default="rebot_b601_vr")
    robot.add_argument("--can-adapter", choices=("damiao", "socketcan"), default="damiao")
    robot.add_argument("--dm-serial-baud", type=int, default=921600)
    robot.add_argument(
        "--motor-control-mode",
        choices=("pos_vel", "mit"),
        default="pos_vel",
        help="q1-q6 motor mode; MIT uses plugin-side velocity and gravity feedforward",
    )
    robot.add_argument("--gripper-control-mode", choices=("force_pos", "mit"), default="force_pos")
    robot.add_argument("--gripper-torque-ratio", type=float, default=0.2,
                       help="FORCE_POS maximum grip force ratio in [0, 1]")
    robot.add_argument("--max-relative-target-deg", type=float, default=20.0)
    robot.add_argument("--initial-q", type=float, nargs=6,
                       default=tuple(DEFAULT_INITIAL_Q_REFERENCE_RAD),
                       metavar=("Q1", "Q2", "Q3", "Q4", "Q5", "Q6"),
                       help="initial pose in validated RS-example radians; q2/q3 are converted to DM signs")
    robot.add_argument("--move-to-initial", action=argparse.BooleanOptionalAction, default=True,
                       help="move at bounded speed to --initial-q before accepting VR control")
    robot.add_argument("--initial-move-tolerance-deg", type=float, default=2.0)
    robot.add_argument("--initial-move-timeout", type=float, default=30.0)
    robot.add_argument("--initial-stall-timeout", type=float, default=5.0)
    robot.add_argument("--no-calibrate", action="store_true")
    robot.add_argument("--disable-torque-on-disconnect", action=argparse.BooleanOptionalAction,
                       default=True, help="disable motors when exiting (support the arm before using the default)")

    vr = parser.add_argument_group("VR source")
    vr.add_argument("--backend", choices=("xrobotoolkit_v1", "isaac"), default="xrobotoolkit_v1")
    vr.add_argument("--hand", choices=("left", "right"), default="right")
    vr.add_argument("--host", default="0.0.0.0")
    vr.add_argument("--port", type=int, default=63901)
    vr.add_argument("--no-cloudxr-launch", action="store_true")
    vr.add_argument("--stale-timeout", type=float, default=0.2)
    vr.add_argument("--grip-press", type=float, default=0.60)
    vr.add_argument("--grip-release", type=float, default=0.40)

    mapping = parser.add_argument_group("Cartesian mapping")
    mapping.add_argument("--position-scale", type=float, default=1.0)
    mapping.add_argument("--orientation-scale", type=float, default=1.0)
    mapping.add_argument("--position-filter-hz", type=float, default=0.0)
    mapping.add_argument("--orientation-filter-hz", type=float, default=0.0)
    mapping.add_argument("--position-deadband-m", type=float, default=0.0)
    mapping.add_argument("--orientation-deadband-deg", type=float, default=0.0)

    ik = parser.add_argument_group("IK and safety")
    ik.add_argument("--qp-solver", choices=("scipy", "osqp"), default="scipy")
    ik.add_argument(
        "--ik-mode",
        choices=("pose", "position"),
        default="pose",
        help="pose tracks XYZ and orientation; position tracks XYZ only",
    )
    ik.add_argument("--qp-position-cost", type=float, default=20.0)
    ik.add_argument("--qp-orientation-cost", type=float, default=2.0)
    ik.add_argument("--qp-orientation-cost-min", type=float, default=0.05)
    ik.add_argument(
        "--qp-position-gain",
        type=float,
        default=10.0,
        help="Cartesian position error feedback gain in 1/s",
    )
    ik.add_argument(
        "--qp-orientation-gain",
        type=float,
        default=8.0,
        help="Cartesian orientation error feedback gain in 1/s",
    )
    ik.add_argument(
        "--qp-damping",
        "--qp-damping-min",
        dest="qp_damping",
        type=float,
        default=1e-3,
        help="minimum QP damping away from singularities",
    )
    ik.add_argument("--qp-damping-max", type=float, default=0.1)
    ik.add_argument("--qp-smoothness-cost", type=float, default=0.05)
    ik.add_argument("--qp-posture-cost", type=float, default=0.01)
    ik.add_argument(
        "--singularity-threshold",
        type=float,
        default=0.08,
        help="dimensionless sigma_min where smooth adaptation starts",
    )
    ik.add_argument(
        "--singularity-critical-threshold",
        type=float,
        default=0.02,
        help="dimensionless sigma_min for maximum damping/orientation relaxation",
    )
    ik.add_argument(
        "--singularity-characteristic-length-m",
        type=float,
        default=0.3,
        help="length used to normalize linear Jacobian rows for SVD",
    )
    ik.add_argument("--joint-limit-margin-deg", type=float, default=2.0)
    ik.add_argument("--qp-max-solve-time-ms", type=float, default=8.0)
    ik.add_argument("--urdf", type=Path)
    ik.add_argument("--max-joint-speed-rad-s", type=float, default=5.5)
    ik.add_argument("--max-joint-acceleration-rad-s2", type=float, default=20.0)
    ik.add_argument("--wrist-speed-rad-s", type=float, default=12.0, help="q4-q6 speed limit (rad/s)")
    ik.add_argument("--wrist-acceleration-rad-s2", type=float, default=60.0, help="q4-q6 acceleration limit (rad/s^2)")
    ik.add_argument("--wrist-relative-target-deg", type=float, default=20.0, help="q4-q6 follower relative target limit (deg)")
    ik.add_argument(
        "--mit-kp",
        type=float,
        nargs=6,
        default=(36.0, 36.0, 36.0, 10, 10, 10),
        metavar=("Q1", "Q2", "Q3", "Q4", "Q5", "Q6"),
        help=(
            "MIT position gains for q1-q6 "
            "(default 45 45 45 10 10 10; valid range 0..500)"
        ),
    )
    ik.add_argument(
        "--mit-kd",
        type=float,
        nargs=6,
        default=(4.0, 4.0, 4.0, 1.0, 1.0, 1.0),
        metavar=("Q1", "Q2", "Q3", "Q4", "Q5", "Q6"),
        help="MIT velocity gains for q1-q6 (valid range 0..5)",
    )
    ik.add_argument(
        "--mit-torque-limit-nm",
        type=float,
        nargs=6,
        default=(27.0, 27.0, 27.0, 7.0, 7.0, 7.0),
        metavar=("Q1", "Q2", "Q3", "Q4", "Q5", "Q6"),
        help=(
            "absolute q1-q6 feedforward torque limits in N*m "
            "(defaults to the dynamics URDF effort limits)"
        ),
    )
    ik.add_argument(
        "--mit-gravity-scale",
        type=float,
        default=1.0,
        help="gravity feedforward multiplier in [0, 2]",
    )
    ik.add_argument(
        "--mit-gravity-ramp-s",
        type=float,
        default=0.0,
        help=(
            "time to ramp gravity feedforward after the first MIT command "
            "(default 0 matches the tuned controller's direct g(q) feedforward)"
        ),
    )
    ik.add_argument(
        "--mit-dynamics-urdf",
        type=Path,
        default=None,
        help="six-axis inertial URDF; defaults to the packaged B601-DM model",
    )
    ik.add_argument(
        "--arm-command-lookahead-ms",
        type=float,
        default=50.0,
        help="q1-q3 position-command lookahead in milliseconds",
    )
    ik.add_argument(
        "--wrist-command-lookahead-ms",
        type=float,
        default=25.0,
        help="q4-q6 position-command lookahead in milliseconds",
    )
    ik.add_argument("--feedback-fault-max-consecutive", type=int, default=5)
    ik.add_argument("--feedback-fault-settle-time", type=float, default=0.25)
    ik.add_argument("--gripper-max-speed-deg-s", type=float, default=1200.0)
    ik.add_argument("--gripper-max-acceleration-deg-s2", type=float, default=5000.0)
    ik.add_argument("--gripper-relative-target-deg", type=float, default=None, help="gripper follower relative target limit; defaults to --max-relative-target-deg")
    ik.add_argument("--gripper-open-deg", type=float, default=-180.0)
    ik.add_argument("--gripper-closed-deg", type=float, default=0.0)

    runtime = parser.add_argument_group("runtime")
    runtime.add_argument("--fps", type=float, default=90.0)
    runtime.add_argument("--duration", type=float, default=0.0, help="0 runs until Ctrl-C")
    runtime.add_argument("--status-rate", type=float, default=5.0)
    runtime.add_argument(
        "--csv-log",
        type=Path,
        default=None,
        metavar="PATH",
        help=(
            "write per-frame joint, IK, and end-to-end latency diagnostics to "
            "PATH; also writes *_latency_summary.csv (default: disabled)"
        ),
    )
    runtime.add_argument("--log-level", choices=("DEBUG", "INFO", "WARNING"), default="INFO")
    return parser


def _invalid_names(
    values: dict[str, float | None],
    *,
    allow_none: bool = False,
    allow_zero: bool = False,
) -> list[str]:
    """Return names whose values are not finite or violate the lower bound."""
    invalid = []
    for name, value in values.items():
        if value is None and allow_none:
            continue
        if value is None or not np.isfinite(value):
            invalid.append(name)
        elif value < 0.0 or (value == 0.0 and not allow_zero):
            invalid.append(name)
    return invalid


def _validate_named_values(
    values: dict[str, float | None],
    requirement: str,
    **options: bool,
) -> None:
    if invalid := _invalid_names(values, **options):
        raise ValueError(
            f"the following parameters must be {requirement}: {', '.join(invalid)}"
        )


def _is_finite_vector_in_range(
    values: np.ndarray,
    lower: float | np.ndarray,
    upper: float | np.ndarray,
    *,
    lower_inclusive: bool = True,
) -> bool:
    if values.shape != (6,) or not np.all(np.isfinite(values)):
        return False
    lower_ok = values >= lower if lower_inclusive else values > lower
    return bool(np.all(lower_ok) and np.all(values <= upper))


def validate_args(args: argparse.Namespace) -> None:
    """Reject unsafe or internally inconsistent command-line settings."""
    _validate_named_values(
        {
            "stale-timeout": args.stale_timeout,
            "max-joint-speed-rad-s": args.max_joint_speed_rad_s,
            "max-joint-acceleration-rad-s2": args.max_joint_acceleration_rad_s2,
            "max-relative-target-deg": args.max_relative_target_deg,
            "gripper-max-speed-deg-s": args.gripper_max_speed_deg_s,
            "gripper-max-acceleration-deg-s2": args.gripper_max_acceleration_deg_s2,
            "initial-move-tolerance-deg": args.initial_move_tolerance_deg,
            "initial-move-timeout": args.initial_move_timeout,
            "initial-stall-timeout": args.initial_stall_timeout,
            "fps": args.fps,
            "status-rate": args.status_rate,
        },
        "positive",
    )

    _validate_named_values(
        {
            "wrist-speed-rad-s": args.wrist_speed_rad_s,
            "wrist-acceleration-rad-s2": args.wrist_acceleration_rad_s2,
            "wrist-relative-target-deg": args.wrist_relative_target_deg,
            "gripper-relative-target-deg": args.gripper_relative_target_deg,
        },
        "positive",
        allow_none=True,
    )

    _validate_named_values(
        {
            "position-scale": args.position_scale,
            "orientation-scale": args.orientation_scale,
            "position-filter-hz": args.position_filter_hz,
            "orientation-filter-hz": args.orientation_filter_hz,
            "position-deadband-m": args.position_deadband_m,
            "orientation-deadband-deg": args.orientation_deadband_deg,
            "feedback-fault-settle-time": args.feedback_fault_settle_time,
        },
        "non-negative",
        allow_zero=True,
    )

    if args.duration < 0.0:
        raise ValueError("duration must be non-negative")

    # QP settings include relationships between values, so validate them together.
    qp_values = (
        args.qp_position_cost,
        args.qp_position_gain,
        args.qp_orientation_gain,
        args.qp_orientation_cost,
        args.qp_orientation_cost_min,
        args.qp_damping,
        args.qp_damping_max,
        args.qp_smoothness_cost,
        args.qp_posture_cost,
        args.singularity_threshold,
        args.singularity_critical_threshold,
        args.singularity_characteristic_length_m,
        args.joint_limit_margin_deg,
        args.qp_max_solve_time_ms,
        args.arm_command_lookahead_ms,
        args.wrist_command_lookahead_ms,
    )
    qp_valid = (
        np.all(np.isfinite(qp_values))
        and args.qp_position_cost > 0
        and args.qp_position_gain > 0
        and args.qp_orientation_gain > 0
        and args.qp_orientation_cost >= 0
        and args.qp_orientation_cost_min >= 0
        and not (
            args.qp_orientation_cost > 0
            and args.qp_orientation_cost_min > args.qp_orientation_cost
        )
        and 0 <= args.qp_damping <= args.qp_damping_max
        and args.qp_smoothness_cost >= 0
        and args.qp_posture_cost >= 0
        and 0 <= args.singularity_critical_threshold < args.singularity_threshold
        and args.singularity_characteristic_length_m > 0
        and args.joint_limit_margin_deg >= 0
        and args.qp_max_solve_time_ms > 0
        and args.arm_command_lookahead_ms > 0
        and args.wrist_command_lookahead_ms > 0
    )
    if not qp_valid:
        raise ValueError("invalid QP parameters")

    if args.feedback_fault_max_consecutive <= 0:
        raise ValueError("feedback-fault-max-consecutive must be positive")

    # Gripper positions use the follower's negative-degree convention.
    if not 0.0 <= args.gripper_torque_ratio <= 1.0:
        raise ValueError("gripper-torque-ratio must be in [0, 1]")
    if not np.all(np.isfinite([args.gripper_open_deg, args.gripper_closed_deg])):
        raise ValueError("gripper open and closed positions must be finite")
    if not -270.0 <= args.gripper_open_deg < args.gripper_closed_deg <= 0.0:
        raise ValueError("gripper positions must satisfy -270 <= open < closed <= 0 degrees")

    # MIT vectors always map to q1..q6 in order.
    mit_kp = np.asarray(args.mit_kp, dtype=np.float64)
    mit_kd = np.asarray(args.mit_kd, dtype=np.float64)
    mit_torque = np.asarray(args.mit_torque_limit_nm, dtype=np.float64)

    if not _is_finite_vector_in_range(mit_kp, 0.0, 500.0):
        raise ValueError("MIT Kp must contain six finite values in [0, 500]")
    if not _is_finite_vector_in_range(mit_kd, 0.0, 5.0):
        raise ValueError("MIT Kd must contain six finite values in [0, 5]")

    effort_limit = np.array([27.0, 27.0, 27.0, 7.0, 7.0, 7.0])
    if not _is_finite_vector_in_range(
        mit_torque, 0.0, effort_limit, lower_inclusive=False
    ):
        raise ValueError(
            "MIT torque limits must be positive and no greater than "
            "[27, 27, 27, 7, 7, 7] N*m"
        )

    if (
        not np.isfinite(args.mit_gravity_scale)
        or not 0 <= args.mit_gravity_scale <= 2
    ):
        raise ValueError("MIT gravity scale must be in [0, 2]")
    if not np.isfinite(args.mit_gravity_ramp_s) or args.mit_gravity_ramp_s < 0:
        raise ValueError("MIT gravity ramp must be non-negative")


def follower_pos_vel_velocity(arm_speed_rad_s: float, wrist_speed_rad_s: float, gripper_speed_deg_s: float) -> list[float]:
    return [*([float(np.rad2deg(arm_speed_rad_s))] * 3), *([float(np.rad2deg(wrist_speed_rad_s))] * 3), gripper_speed_deg_s]


def follower_relative_target(arm_relative_target_deg: float, wrist_relative_target_deg: float, gripper_relative_target_deg: float | None = None) -> float | dict[str, float]:
    gripper_value = arm_relative_target_deg if gripper_relative_target_deg is None else gripper_relative_target_deg
    if wrist_relative_target_deg == arm_relative_target_deg == gripper_value:
        return arm_relative_target_deg
    return {"shoulder_pan": arm_relative_target_deg, "shoulder_lift": arm_relative_target_deg, "elbow_flex": arm_relative_target_deg, "wrist_flex": wrist_relative_target_deg, "wrist_yaw": wrist_relative_target_deg, "wrist_roll": wrist_relative_target_deg, "gripper": gripper_value}


def status_line(status: CartesianControlStatus, sent_action: dict[str, float] | None = None) -> str:
    def vector(values: np.ndarray) -> str:
        return np.array2string(
            np.asarray(values), precision=1, suppress_small=True, max_line_width=120
        )

    if status.ik_success is None:
        ik = "WAIT"
    elif status.ik_success:
        ik = "OK"
    else:
        ik = f"HOLD({status.ik_reason})"

    loop = "--" if status.control_loop_hz is None else f"{status.control_loop_hz:.1f}Hz"
    lines = [
        (
            f"[{status.state.value.upper()}] "
            f"VR={'OK' if status.tracking else '--'}  "
            f"FB={'OK' if status.feedback_valid else 'FAULT'}  IK={ik}  "
            f"loop={loop}"
        )
    ]
    if not status.feedback_valid:
        lines.append(
            f"! fault #{status.feedback_fault_count}: {status.feedback_fault_reason}"
        )

    lines.extend(
        (
            f"  actual  {vector(status.actual_deg)} deg",
            f"  target  {vector(status.target_deg)} deg",
            f"  command {vector(status.command_deg)} deg",
        )
    )

    summary = []
    if status.tcp_position_error_m is not None:
        summary.append(f"TCP={status.tcp_position_error_m * 1000.0:.1f}mm")
    if status.orientation_error_deg is not None:
        summary.append(f"rot={status.orientation_error_deg:.1f}deg")
    if status.dq_norm_rad_s is not None:
        summary.append(f"dq={status.dq_norm_rad_s:.2f}rad/s")
    if status.qp_solve_time_ms is not None:
        summary.append(f"solve={status.qp_solve_time_ms:.2f}ms")
    if status.feedback_read_ms is not None:
        summary.append(f"read={status.feedback_read_ms:.2f}ms")
    if status.send_action_ms is not None:
        summary.append(f"send={status.send_action_ms:.2f}ms")
    if summary:
        lines.append("  " + "  ".join(summary))
    return "\n".join(lines)

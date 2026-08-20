"""Command-line parsing and display helpers for the real-robot runner."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from .cartesian_controller import CartesianControlStatus, GRIPPER_NAME
from .startup_pose import DEFAULT_INITIAL_Q_REFERENCE_RAD


def build_parser(description: str | None = None) -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=description)
    robot = parser.add_argument_group("robot")
    robot.add_argument("--robot-port", default="/dev/ttyACM0")
    robot.add_argument("--robot-id", default="rebot_b601_vr")
    robot.add_argument("--can-adapter", choices=("damiao", "socketcan"), default="damiao")
    robot.add_argument("--dm-serial-baud", type=int, default=921600)
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
    vr.add_argument("--grip-press", type=float, default=0.85)
    vr.add_argument("--grip-release", type=float, default=0.75)

    mapping = parser.add_argument_group("Cartesian mapping")
    mapping.add_argument("--position-scale", type=float, default=1.0)
    mapping.add_argument("--orientation-scale", type=float, default=1.0)
    mapping.add_argument("--position-filter-hz", type=float, default=8.0)
    mapping.add_argument("--orientation-filter-hz", type=float, default=6.0)
    mapping.add_argument("--position-deadband-m", type=float, default=5e-4)
    mapping.add_argument("--orientation-deadband-deg", type=float, default=0.25)

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
        "--arm-command-lookahead-ms",
        type=float,
        default=50.0,
        help="q1-q3 POS_VEL position-command lookahead in milliseconds",
    )
    ik.add_argument(
        "--wrist-command-lookahead-ms",
        type=float,
        default=25.0,
        help="q4-q6 POS_VEL position-command lookahead in milliseconds",
    )
    ik.add_argument("--feedback-fault-max-consecutive", type=int, default=5)
    ik.add_argument("--feedback-fault-settle-time", type=float, default=0.25)
    ik.add_argument("--gripper-max-speed-deg-s", type=float, default=1200.0)
    ik.add_argument("--gripper-max-acceleration-deg-s2", type=float, default=5000.0)
    ik.add_argument("--gripper-relative-target-deg", type=float, default=None, help="gripper follower relative target limit; defaults to --max-relative-target-deg")
    ik.add_argument("--gripper-open-deg", type=float, default=-180.0)
    ik.add_argument("--gripper-closed-deg", type=float, default=0.0)

    runtime = parser.add_argument_group("runtime")
    runtime.add_argument("--fps", type=float, default=60.0)
    runtime.add_argument("--duration", type=float, default=0.0, help="0 runs until Ctrl-C")
    runtime.add_argument("--status-rate", type=float, default=1.0)
    runtime.add_argument("--log-level", choices=("DEBUG", "INFO", "WARNING"), default="INFO")
    return parser


def validate_args(args: argparse.Namespace) -> None:
    positive = {
        "stale-timeout": args.stale_timeout,
        "max-joint-speed-rad-s": args.max_joint_speed_rad_s,
        "max-joint-acceleration-rad-s2": args.max_joint_acceleration_rad_s2,
        "max-relative-target-deg": args.max_relative_target_deg,
        "gripper-max-speed-deg-s": args.gripper_max_speed_deg_s,
        "gripper-max-acceleration-deg-s2": args.gripper_max_acceleration_deg_s2,
        "initial-move-tolerance-deg": args.initial_move_tolerance_deg,
        "initial-move-timeout": args.initial_move_timeout, "initial-stall-timeout": args.initial_stall_timeout,
        "fps": args.fps, "status-rate": args.status_rate,
    }
    invalid = [name for name, value in positive.items() if not np.isfinite(value) or value <= 0.0]
    if invalid:
        raise ValueError(f"the following parameters must be positive: {', '.join(invalid)}")
    optional_positive = {
        "wrist-speed-rad-s": args.wrist_speed_rad_s,
        "wrist-acceleration-rad-s2": args.wrist_acceleration_rad_s2,
        "wrist-relative-target-deg": args.wrist_relative_target_deg,
        "gripper-relative-target-deg": args.gripper_relative_target_deg,
    }
    invalid = [name for name, value in optional_positive.items() if value is not None and (not np.isfinite(value) or value <= 0.0)]
    if invalid:
        raise ValueError(f"the following parameters must be positive: {', '.join(invalid)}")
    non_negative = {
        "position-scale": args.position_scale, "orientation-scale": args.orientation_scale,
        "position-filter-hz": args.position_filter_hz, "orientation-filter-hz": args.orientation_filter_hz,
        "position-deadband-m": args.position_deadband_m, "orientation-deadband-deg": args.orientation_deadband_deg,
        "feedback-fault-settle-time": args.feedback_fault_settle_time,
    }
    invalid = [name for name, value in non_negative.items() if not np.isfinite(value) or value < 0.0]
    if invalid:
        raise ValueError(f"the following parameters must be non-negative: {', '.join(invalid)}")
    if args.duration < 0.0:
        raise ValueError("duration must be non-negative")
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
    if (
        not np.all(np.isfinite(qp_values))
        or args.qp_position_cost <= 0
        or args.qp_position_gain <= 0
        or args.qp_orientation_gain <= 0
        or args.qp_orientation_cost < 0
        or args.qp_orientation_cost_min < 0
        or (
            args.qp_orientation_cost > 0
            and args.qp_orientation_cost_min > args.qp_orientation_cost
        )
        or args.qp_damping < 0
        or args.qp_damping_max < args.qp_damping
        or args.qp_smoothness_cost < 0
        or args.qp_posture_cost < 0
        or args.singularity_critical_threshold < 0
        or args.singularity_threshold <= args.singularity_critical_threshold
        or args.singularity_characteristic_length_m <= 0
        or args.joint_limit_margin_deg < 0
        or args.qp_max_solve_time_ms <= 0
        or args.arm_command_lookahead_ms <= 0
        or args.wrist_command_lookahead_ms <= 0
    ):
        raise ValueError("invalid QP parameters")
    if args.feedback_fault_max_consecutive <= 0:
        raise ValueError("feedback-fault-max-consecutive must be positive")
    if not 0.0 <= args.gripper_torque_ratio <= 1.0:
        raise ValueError("gripper-torque-ratio must be in [0, 1]")
    if not np.all(np.isfinite([args.gripper_open_deg, args.gripper_closed_deg])):
        raise ValueError("gripper open and closed positions must be finite")
    if not -270.0 <= args.gripper_open_deg < args.gripper_closed_deg <= 0.0:
        raise ValueError("gripper positions must satisfy -270 <= open < closed <= 0 degrees")


def follower_pos_vel_velocity(arm_speed_rad_s: float, wrist_speed_rad_s: float, gripper_speed_deg_s: float) -> list[float]:
    return [*([float(np.rad2deg(arm_speed_rad_s))] * 3), *([float(np.rad2deg(wrist_speed_rad_s))] * 3), gripper_speed_deg_s]


def follower_relative_target(arm_relative_target_deg: float, wrist_relative_target_deg: float, gripper_relative_target_deg: float | None = None) -> float | dict[str, float]:
    gripper_value = arm_relative_target_deg if gripper_relative_target_deg is None else gripper_relative_target_deg
    if wrist_relative_target_deg == arm_relative_target_deg == gripper_value:
        return arm_relative_target_deg
    return {"shoulder_pan": arm_relative_target_deg, "shoulder_lift": arm_relative_target_deg, "elbow_flex": arm_relative_target_deg, "wrist_flex": wrist_relative_target_deg, "wrist_yaw": wrist_relative_target_deg, "wrist_roll": wrist_relative_target_deg, "gripper": gripper_value}


def status_line(status: CartesianControlStatus, sent_action: dict[str, float] | None = None) -> str:
    ik = "pending" if status.ik_success is None else (f"ok err={status.ik_error_m:.5f}m" if status.ik_success else f"hold {status.ik_reason}")
    sent_gripper_deg = status.gripper_command_deg if sent_action is None else float(sent_action.get(f"{GRIPPER_NAME}.pos", status.gripper_command_deg))
    orientation_error = "n/a" if status.orientation_error_deg is None else f"{status.orientation_error_deg:.3f}"
    position_error = "n/a" if status.tcp_position_error_m is None else f"{status.tcp_position_error_m:.5f}"
    sigma_min = "n/a" if status.sigma_min is None else f"{status.sigma_min:.5f}"
    condition = "n/a" if status.condition_number is None else f"{status.condition_number:.1f}"
    damping = "n/a" if status.current_damping is None else f"{status.current_damping:.6f}"
    orientation_weight = "n/a" if status.current_orientation_weight is None else f"{status.current_orientation_weight:.4f}"
    dq_norm = "n/a" if status.dq_norm_rad_s is None else f"{status.dq_norm_rad_s:.3f}"
    solve_ms = "n/a" if status.qp_solve_time_ms is None else f"{status.qp_solve_time_ms:.3f}"
    qp_age_ms = "n/a" if status.qp_result_age_ms is None else f"{status.qp_result_age_ms:.3f}"
    sample_age_ms = "n/a" if status.tracking_sample_age_ms is None else f"{status.tracking_sample_age_ms:.3f}"
    loop_hz = "n/a" if status.control_loop_hz is None else f"{status.control_loop_hz:.1f}"
    feedback_ms = "n/a" if status.feedback_read_ms is None else f"{status.feedback_read_ms:.3f}"
    send_ms = "n/a" if status.send_action_ms is None else f"{status.send_action_ms:.3f}"
    work_ms = "n/a" if status.cycle_work_ms is None else f"{status.cycle_work_ms:.3f}"
    tcp_actual = "n/a" if status.tcp_actual_position_m is None else np.array2string(status.tcp_actual_position_m, precision=4)
    tcp_target = "n/a" if status.tcp_target_position_m is None else np.array2string(status.tcp_target_position_m, precision=4)
    tcp_rotation_actual = "n/a" if status.tcp_actual_rotvec_rad is None else np.array2string(status.tcp_actual_rotvec_rad, precision=3)
    tcp_rotation_target = "n/a" if status.tcp_target_rotvec_rad is None else np.array2string(status.tcp_target_rotvec_rad, precision=3)
    qp_velocity = "n/a" if status.qp_joint_velocity_rad_s is None else np.array2string(status.qp_joint_velocity_rad_s, precision=3)
    target_linear_velocity = "n/a" if status.target_linear_velocity_m_s is None else np.array2string(status.target_linear_velocity_m_s, precision=3)
    target_angular_velocity = "n/a" if status.target_angular_velocity_rad_s is None else np.array2string(status.target_angular_velocity_rad_s, precision=3)
    feedback = "ok" if status.feedback_valid else f"HOLD {status.feedback_fault_count} reason={status.feedback_fault_reason}"
    trigger_control = "active" if status.gripper_trigger_active else "hold"
    return (f"state={status.state.value:<7} tracking={status.tracking} ik={ik} feedback={feedback} jobs={status.submitted}/{status.solved}/{status.rejected} A={status.primary_button} home={status.home_requested} B={status.secondary_button} zero={status.zero_requested}\n"
            f"  trigger={status.trigger:.3f} trigger_control={trigger_control} gripper_deg(actual/target/shaped/sent)={status.gripper_actual_deg:.1f}/{status.gripper_target_deg:.1f}/{status.gripper_command_deg:.1f}/{sent_gripper_deg:.1f}\n"
            f"  wrist_deg[q4/q5/q6] actual/target/command={np.array2string(status.actual_deg[3:6], precision=1)}/{np.array2string(status.target_deg[3:6], precision=1)}/{np.array2string(status.command_deg[3:6], precision=1)}\n"
            f"  position_error_m={position_error} orientation_error_deg={orientation_error}\n"
            f"  qp_mode={status.ik_mode} sigma_min={sigma_min} condition={condition} damping={damping} orientation_weight={orientation_weight} dq_norm_rad_s={dq_norm} solve_ms={solve_ms} result_age_ms={qp_age_ms}\n"
            f"  timing_hz={loop_hz} sample_age_ms={sample_age_ms} feedback_ms={feedback_ms} send_ms={send_ms} work_ms={work_ms}\n"
            f"  tcp_position_m actual/target={tcp_actual}/{tcp_target}\n"
            f"  target_twist linear_m_s/angular_rad_s={target_linear_velocity}/{target_angular_velocity}\n"
            f"  tcp_rotvec_rad actual/target={tcp_rotation_actual}/{tcp_rotation_target} qp_dq_rad_s={qp_velocity}\n"
            f"  actual_deg={np.array2string(status.actual_deg, precision=1, suppress_small=True)}\n"
            f"  target_deg={np.array2string(status.target_deg, precision=1, suppress_small=True)}\n"
            f"  command_deg={np.array2string(status.command_deg, precision=1, suppress_small=True)}")

"""Cached motor telemetry and lifecycle records; never request CAN feedback here."""

from __future__ import annotations

import time

import numpy as np

from ..control.types import ARM_JOINT_NAMES


MOTION_FIELDNAMES = (
    "arm_id", "phase", "row_kind", "motion_result", "motion_detail",
    "motion_elapsed_s", "motion_tolerance_deg", "motion_max_error_deg",
    "feedback_cache_read_monotonic_ns", "feedback_freshness",
    *(f"{kind}_{joint}_{unit}" for kind, unit in (
        ("sent", "deg"), ("error", "deg"), ("stalled", "s"),
        ("cached_position", "rad"), ("actual_velocity", "rad_s"),
        ("reported_torque", "nm"), ("motor_status", "code"),
        ("mos_temperature", "c"), ("rotor_temperature", "c"),
        ("mit_kp", "nm_rad"), ("mit_kd", "nm_s_rad"),
        ("estimated_pd_ff", "nm"), ("command_clipped", "flag"),
        ("zero_trim", "nm"),
    ) for joint in ARM_JOINT_NAMES),
    *(f"feedback_cache_error_{joint}" for joint in ARM_JOINT_NAMES),
    "sent_gripper_deg", "gripper_command_sent", "gripper_enabled", "vr_hand",
    "gripper_command_clipped_flag", "gripper_control_mode",
    "mit_kp_gripper_nm_rad", "mit_kd_gripper_nm_s_rad",
    "estimated_position_effort_gripper_nm",
    "mit_desired_velocity_gripper_deg_s",
)


def cached_motor_row(robot) -> dict[str, object]:
    """Snapshot SDK-reported values, not verified fresh physical measurements.

    The cache may advance independently of get_observation(); retain its own
    position for comparison. Its read timestamp is NOT a receive timestamp.
    """
    telemetry = getattr(robot, "motor_feedback_telemetry", None)
    if callable(telemetry):
        return telemetry()
    row = {"feedback_cache_read_monotonic_ns": time.monotonic_ns(),
           "feedback_freshness": "unknown_no_receive_timestamp"}
    motors = getattr(robot, "motors", {})
    for joint in ARM_JOINT_NAMES:
        try:
            motor = motors.get(joint)
            state = None if motor is None else motor.get_state()
            if state is None:
                row[f"feedback_cache_error_{joint}"] = "unavailable"
                continue
            for field, attribute, unit in (
                ("cached_position", "pos", "rad"),
                ("actual_velocity", "vel", "rad_s"),
                ("reported_torque", "torq", "nm"),
                ("motor_status", "status_code", "code"),
                ("mos_temperature", "t_mos", "c"),
                ("rotor_temperature", "t_rotor", "c"),
            ):
                value = getattr(state, attribute, None)
                if value is not None and np.isfinite(value):
                    row[f"{field}_{joint}_{unit}"] = float(value)
        except Exception as exc:
            # Optional diagnostics must not prevent cleanup or movement checks.
            row[f"feedback_cache_error_{joint}"] = str(exc)
    return row


def mit_command_row(robot, actual_deg, sent_deg, feedback) -> dict[str, object]:
    # Local import avoids importing the dynamics dependency for plain CSV use.
    from ..control.mit import MITCommandDispatcher

    if not isinstance(robot, MITCommandDispatcher):
        return {}
    row = {"motor_control_mode": "mit"}
    velocity = robot.desired_velocity_rad_s
    zero_trim = robot.__dict__.get("zero_trim")
    for i, joint in enumerate(ARM_JOINT_NAMES):
        row[f"zero_trim_{joint}_nm"] = 0.0 if zero_trim is None else float(zero_trim.torque_nm[i])
        row[f"mit_desired_velocity_{joint}_rad_s"] = float(velocity[i])
        row[f"mit_gravity_{joint}_nm"] = float(robot.last_gravity_torque_nm[i])
        row[f"mit_feedforward_{joint}_nm"] = float(robot.last_feedforward_torque_nm[i])
        row[f"mit_kp_{joint}_nm_rad"] = float(robot.kp[i])
        row[f"mit_kd_{joint}_nm_s_rad"] = float(robot.kd[i])
        measured_velocity = feedback.get(f"actual_velocity_{joint}_rad_s")
        if measured_velocity is not None:
            # Model estimate only: not measured torque or a total torque limit.
            row[f"estimated_pd_ff_{joint}_nm"] = float(
                robot.kp[i] * np.deg2rad(sent_deg[i] - actual_deg[i])
                + robot.kd[i] * (velocity[i] - measured_velocity)
                + robot.last_feedforward_torque_nm[i]
            )
    return row


def gripper_command_row(robot, status, sent_action) -> dict[str, object]:
    """Record returned send targets; no CAN reads and no torque-state claims."""
    from ..control.mit import MITCommandDispatcher

    sent = None if sent_action is None else sent_action.get("gripper.pos")
    row = {"gripper_command_sent": sent is not None}
    if sent is None:
        return row
    row["sent_gripper_deg"] = float(sent)
    row["gripper_command_clipped_flag"] = not np.isclose(
        sent, status.gripper_command_deg, rtol=0., atol=1e-6)
    if isinstance(robot, MITCommandDispatcher):
        row["gripper_control_mode"] = robot.config.gripper_control_mode
        if robot.config.gripper_control_mode == "mit":
            row["mit_desired_velocity_gripper_deg_s"] = robot.last_gripper_velocity_deg_s
            kp = float(robot.config.gripper_mit_kp)
            row["mit_kp_gripper_nm_rad"] = kp
            row["mit_kd_gripper_nm_s_rad"] = float(robot.config.gripper_mit_kd)
            if status.feedback_valid:
                # Only the position-error term, NOT measured/total torque:
                # RS velocity cache is not reliable enough to estimate damping.
                row["estimated_position_effort_gripper_nm"] = float(
                    kp * np.deg2rad(sent - status.gripper_actual_deg))
    return row


class MotionRecorder:
    def __init__(self, write_row, *, arm_id, phase, tolerance_deg):
        self.write_row = write_row
        self.context = {"arm_id": arm_id or "single", "phase": phase,
                        "motion_tolerance_deg": tolerance_deg}
        self.started = time.monotonic()
        self.latest = {}

    def sample(self, row):
        self.latest = dict(row)
        self._write({**row, "row_kind": "sample"})

    def event(self, result, detail=""):
        # Keep the last joint diagnostics, but don't duplicate latency samples.
        from .logger import LATENCY_FIELDNAMES
        last = {k: v for k, v in self.latest.items() if k not in LATENCY_FIELDNAMES}
        self._write({**last, "row_kind": "event", "motion_result": result,
                     "motion_detail": detail})

    def _write(self, row):
        if self.write_row is not None:
            self.write_row({**row, **self.context, "timestamp_ns": time.time_ns(),
                            "monotonic_timestamp_ns": time.monotonic_ns(),
                            "motion_elapsed_s": time.monotonic() - self.started})

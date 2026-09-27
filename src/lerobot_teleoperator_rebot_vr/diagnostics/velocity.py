"""Flat, scalar snapshots for request/result and motor velocity diagnostics."""

from ..constants import ARM_JOINT_NAMES


def vector_fields(prefix, values, suffix, names=ARM_JOINT_NAMES):
    return {f"{prefix}_{name}_{suffix}": float(value)
            for name, value in zip(names, values, strict=True)}


VELOCITY_FIELDNAMES = (
    "ik_request_linear_velocity_dt_s", "ik_request_linear_velocity_time_source",
    "ik_request_linear_velocity_jump_rejected", "ik_request_linear_velocity_speed_limited",
    "ik_request_linear_velocity_rate_limited",
    *(f"ik_request_linear_velocity_fast_brake_{a}_flag" for a in "xyz"),
    *(f"ik_request_raw_linear_velocity_{a}_m_s" for a in "xyz"),
    "ik_request_sequence", "ik_request_generation", "ik_request_sample_id",
    "ik_request_submitted_monotonic_ns", "ik_request_dt_s",
    "ik_request_sent_velocity_age_ms",
    "ik_request_dispatch_after_send",
    "ik_request_prepared_monotonic_ns",
    *(f"ik_request_target_position_{a}_m" for a in "xyz"),
    *(f"ik_request_linear_velocity_{a}_m_s" for a in "xyz"),
    *(f"ik_request_position_error_{a}_m" for a in "xyz"),
    *(f"ik_request_previous_velocity_{j}_rad_s" for j in ARM_JOINT_NAMES),
    *(f"qp_output_velocity_{j}_rad_s" for j in ARM_JOINT_NAMES),
    *(f"qp_{kind}_bound_active_{j}_flag" for kind in ("speed", "acceleration") for j in ARM_JOINT_NAMES),
    "ik_result_applied_this_cycle",
    *(f"mit_{kind}_velocity_{j}_rad_s" for kind in ("input", "target") for j in ARM_JOINT_NAMES),
    *(f"mit_{kind}_limited_{j}_flag" for kind in ("speed", "acceleration", "braking") for j in ARM_JOINT_NAMES),
    "mit_velocity_step_dt_s",
)

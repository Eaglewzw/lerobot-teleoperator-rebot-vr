"""Compatibility entry point for the real-robot teleoperation runtime."""

from .runtime.real import (
    _feedback_hold_action,
    _follower_pos_vel_velocity,
    _follower_relative_target,
    _move_to_initial_pose,
    _parser,
    _send_feedback_hold_action,
    _settle_persistent_feedback_fault,
    _status_line,
    _validate_args,
    main,
)


__all__ = ["main"]

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass

import numpy as np


GRIPPER_TRIGGER_ACTIVATION_DELTA = 0.05


@dataclass
class GripperController:
    """Own the Trigger latch and the shaped one-axis gripper command."""

    goal_deg: float = 0.0
    command_deg: float = 0.0
    velocity_deg_s: float = 0.0
    trigger_active: bool = True
    trigger_reference: float | None = None

    def synchronize_to_feedback(self, actual_deg: float) -> None:
        self.goal_deg = actual_deg
        self.command_deg = actual_deg
        self.velocity_deg_s = 0.0

    def reset_after_feedback_recovery(self, actual_deg: float) -> None:
        self.synchronize_to_feedback(actual_deg)
        self.trigger_active = True
        self.trigger_reference = None

    def reset_for_unavailable_tracking(self, actual_deg: float) -> None:
        self.reset_after_feedback_recovery(actual_deg)

    def request_closed(self, closed_deg: float, trigger: float) -> None:
        self.goal_deg = closed_deg
        self.trigger_active = False
        self.trigger_reference = trigger

    def enter_feedback_hold(self, *, hold_command: bool) -> None:
        self.trigger_active = False
        self.trigger_reference = None
        if hold_command:
            self.goal_deg = self.command_deg
            self.velocity_deg_s = 0.0

    def update_trigger_target(
        self,
        *,
        tracking_fresh: bool,
        trigger: float,
        open_deg: float,
        closed_deg: float,
    ) -> None:
        if tracking_fresh and not self.trigger_active and self.trigger_reference is None:
            self.trigger_reference = trigger
        if (
            tracking_fresh
            and not self.trigger_active
            and self.trigger_reference is not None
            and abs(trigger - self.trigger_reference)
            >= GRIPPER_TRIGGER_ACTIVATION_DELTA
        ):
            self.trigger_active = True
        if tracking_fresh and self.trigger_active:
            self.goal_deg = open_deg + trigger * (closed_deg - open_deg)

    def update_command(
        self,
        *,
        actual_deg: float,
        dt_s: float,
        open_deg: float,
        closed_deg: float,
        max_speed_deg_s: float,
        max_acceleration_deg_s2: float,
        feedback_error_deg: float | None,
        shape_fn: Callable[..., tuple[np.ndarray, np.ndarray]],
        bound_fn: Callable[..., np.ndarray],
        smooth_motion: bool = False,
    ) -> None:
        lower_deg = min(open_deg, closed_deg)
        upper_deg = max(open_deg, closed_deg)
        previous_command_deg = self.command_deg
        position, velocity = shape_fn(
            previous_position=np.array([self.command_deg]),
            previous_velocity=np.array([self.velocity_deg_s]),
            target_position=np.array([self.goal_deg]),
            dt_s=dt_s,
            max_speed=np.array([max_speed_deg_s]),
            max_acceleration=np.array([max_acceleration_deg_s2]),
            lower_limit=np.array([lower_deg]),
            upper_limit=np.array([upper_deg]),
            **({"brake_at_target": True} if smooth_motion else {}),
        )
        self.command_deg = float(position[0])
        self.velocity_deg_s = float(velocity[0])
        if feedback_error_deg is None:
            return

        unclipped_command_deg = self.command_deg
        self.command_deg = float(
            bound_fn(
                np.array([self.command_deg]),
                np.array([actual_deg]),
                feedback_error_deg,
                lower_limit=np.array([lower_deg]),
                upper_limit=np.array([upper_deg]),
            )[0]
        )
        if self.command_deg != unclipped_command_deg:
            # Keep the rate actually achieved after clipping. Resetting to
            # zero on every contact with a moving feedback bound creates a
            # repeated accelerate-stop cycle even with steady Trigger input.
            self.velocity_deg_s = (
                float(np.clip((self.command_deg - previous_command_deg) / dt_s,
                              -max_speed_deg_s, max_speed_deg_s))
                if smooth_motion and dt_s > 0. else 0.0
            )

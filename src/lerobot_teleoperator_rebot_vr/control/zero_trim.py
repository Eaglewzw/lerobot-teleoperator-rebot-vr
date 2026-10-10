"""Bounded slow torque correction for RS zero-pose static tracking error."""

import numpy as np


class ZeroPoseTrim:
    """Integrate only near zero with near-stationary, fresh position feedback.

    Keep the requested position inside hard limits. Correct model/friction error
    through feedforward instead of requesting an out-of-range negative angle.
    This is not a contact detector or permission to disable motor torque.
    """

    def __init__(self, torque_limit_nm):
        self.limit_nm = np.minimum(np.asarray(torque_limit_nm) * .25,
                                   [2., 2., 2., .75, .75, .75])
        self.torque_nm = np.zeros(6)
        self.active = False
        self._previous_q = None
        self._previous_s = None

    def update(self, actual_rad, command_rad, now_s):
        q = np.asarray(actual_rad, dtype=float)
        command = np.asarray(command_rad, dtype=float)
        if q.shape != (6,) or command.shape != (6,) or not (
            np.isfinite(q).all() and np.isfinite(command).all() and np.isfinite(now_s)
        ):
            raise ValueError("zero trim requires finite six-axis feedback and commands")
        dt = 0. if self._previous_s is None else now_s - self._previous_s
        if 0. < dt <= .1 and self._previous_q is not None:
            near_stationary = np.abs(q - self._previous_q) / dt <= np.deg2rad(5.)
            near_zero = (self.active & (np.abs(q) <= np.deg2rad(5.))
                         & (np.abs(command) <= np.deg2rad(.05)))
            eligible = near_zero & near_stationary
            error = np.where(np.abs(q) > np.deg2rad(.1), -q, 0.)
            # Motion after overcoming static friction freezes the correction;
            # do not withdraw it as soon as the joint starts moving.
            desired = np.where(near_zero, self.torque_nm, 0.)
            desired = np.where(eligible, self.torque_nm + 10. * error * dt, desired)
            desired = np.clip(desired, -self.limit_nm, self.limit_nm)
            # Also ramp correction out when Grip resumes or tracking is lost.
            self.torque_nm += np.clip(desired - self.torque_nm, -.5 * dt, .5 * dt)
        # A long feedback gap must never cause an integral or torque jump.
        self._previous_q = q.copy()
        self._previous_s = now_s
        return self.torque_nm.copy()

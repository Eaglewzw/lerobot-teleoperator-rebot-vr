"""Limit explicit startup feedback requests on the existing motor handles.

Only request_feedback is gated. Reception, cache reads, motion commands, and
the follower's relative-target checks continue at their original frequency.
"""

from contextlib import contextmanager
import math
import time


class _LimitedRequests:
    def __init__(self, motor, interval_s, clock):
        self._motor = motor
        self._interval_s = interval_s
        self._clock = clock
        self._last_sent_s = None

    def __getattr__(self, name):
        return getattr(self._motor, name)

    def request_feedback(self):
        now = self._clock()
        if self._last_sent_s is not None and now - self._last_sent_s < self._interval_s:
            return None
        result = self._motor.request_feedback()
        # Failed sends propagate and do not consume a request slot.
        self._last_sent_s = self._clock()
        return result


@contextmanager
def limit_startup_feedback_requests(robot, rate_hz, *, clock=time.monotonic):
    """Temporarily share a per-axis request limit across reads and send_action.

    Install after motor diagnostics so those logs count actual SDK requests.
    Zero disables throttling. Restore before VR following, homing, or shutdown,
    including when initial motion raises or is interrupted. Not a freshness
    check: unchanged get_state remains a potentially stale SDK cache read.
    """
    if not math.isfinite(rate_hz) or rate_hz < 0:
        raise ValueError("initial-feedback-request-hz must be finite and non-negative")
    if rate_hz == 0:
        yield
        return
    installed = {}
    try:
        for name, motor in list(robot.motors.items()):
            wrapper = _LimitedRequests(motor, 1.0 / rate_hz, clock)
            installed[name] = (motor, wrapper)
            robot.motors[name] = wrapper
        yield
    finally:
        for name, (motor, wrapper) in installed.items():
            if robot.motors.get(name) is wrapper:
                robot.motors[name] = motor

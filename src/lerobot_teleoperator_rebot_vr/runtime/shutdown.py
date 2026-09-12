"""VR-owned shutdown using the follower's existing MotorBridge connection.

No extra serial connection, mode changes, error clearing, or motion commands.
The caller must stop all command producers (including MIT dispatch) first.
"""

from __future__ import annotations

import json
import logging
import math
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Callable

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class ShutdownPolicy:
    attempts: int = 3
    interval_s: float = 0.02
    feedback_wait_s: float = 0.1
    request_feedback: bool = True

    def __post_init__(self):
        if type(self.request_feedback) is not bool:
            raise ValueError("disable-request-feedback must be boolean")
        if type(self.attempts) is not int or not 1 <= self.attempts <= 10:
            raise ValueError("disable-attempts must be an integer between 1 and 10")
        for name, value in (("disable-interval-s", self.interval_s),
                            ("disable-feedback-wait-s", self.feedback_wait_s)):
            if not math.isfinite(value) or not 0 < value <= 2:
                raise ValueError(f"{name} must be finite and in (0, 2]")


@dataclass(frozen=True)
class ShutdownFeedback:
    state: Any
    # Only a timestamp captured at CAN reception, atomic with this state, in
    # time.monotonic_ns()'s clock domain qualifies. Never use cache-read time.
    received_monotonic_ns: int | None = None


def read_cached_feedback(motor) -> ShutdownFeedback:
    # MotorBridge 0.3.9 and 0.5.3 expose no reception timestamp/counter here.
    return ShutdownFeedback(motor.get_state())


@dataclass
class MotorShutdownResult:
    joint: str
    send_id: int
    receive_id: int
    attempts: int = 0
    disable_sent: int = 0
    cached_status_code: int | None = None
    observed_status_codes: list[int] = field(default_factory=list)
    confirmed: bool = False
    reason: str = "not attempted"
    errors: list[str] = field(default_factory=list)


@dataclass
class ShutdownReport:
    disable_requested: bool
    motors: list[MotorShutdownResult]
    request_feedback: bool = True
    close_bus_completed: bool = False
    cleanup_errors: list[str] = field(default_factory=list)


class ManagedFollower:
    """Delegate normal I/O; own disconnect without calling LeRobot.disconnect.

    An optional feedback reader supports a future atomic reception-stamped SDK.
    The default reader deliberately cannot confirm torque-off from cached data.
    Shutdown is synchronous and repeat calls return the same report.
    """

    def __init__(self, robot, *, policy: ShutdownPolicy | None = None,
                 report_path: Path | None = None,
                 feedback_reader: Callable = read_cached_feedback):
        self.robot = robot
        self.policy = policy or ShutdownPolicy()
        self.report_path = report_path
        self.feedback_reader = feedback_reader
        self.shutdown_report: ShutdownReport | None = None
        self._shutdown_started = False

    def __getattr__(self, name):
        return getattr(self.robot, name)

    @property
    def needs_cleanup(self) -> bool:
        # is_connected may be False during a partially failed connect/configure.
        return (self.robot.bus is not None or bool(self.robot.motors)
                or any(camera.is_connected for camera in self.robot.cameras.values()))

    def connect(self, **kwargs):
        if self._shutdown_started:
            raise RuntimeError("cannot reconnect a follower after shutdown")
        return self.robot.connect(**kwargs)

    def send_action(self, action):
        if self._shutdown_started:
            raise RuntimeError("motion commands are blocked after shutdown starts")
        return self.robot.send_action(action)

    def disconnect(self) -> ShutdownReport:
        if self.shutdown_report is not None:
            return self.shutdown_report
        self._shutdown_started = True
        motors = dict(self.robot.motors)
        bus = self.robot.bus
        results = [MotorShutdownResult(name, *ids)
                   for name, ids in self.config.motor_can_ids.items()]
        report = ShutdownReport(bool(self.config.disable_torque_on_disconnect), results,
                                request_feedback=self.policy.request_feedback)
        self.shutdown_report = report
        try:
            if report.disable_requested:
                self._disable(motors, bus, results)
            else:
                for result in results:
                    result.reason = "SKIPPED: torque retention requested"
        except Exception as exc:
            self._cleanup_error(report, "disable sequence", exc)
        finally:
            # Keep the receiver and motor handles alive throughout verification.
            # close_bus is the SDK transport shutdown/flush API; do not use
            # shutdown(), which would introduce another SDK-owned disable pass.
            if bus is not None:
                try:
                    bus.close_bus()
                    report.close_bus_completed = True
                except Exception as exc:
                    self._cleanup_error(report, "close_bus", exc)
            for name, motor in motors.items():
                try:
                    motor.close()
                except Exception as exc:
                    self._cleanup_error(report, f"close motor {name}", exc)
            if bus is not None:
                try:
                    bus.close()
                except Exception as exc:
                    self._cleanup_error(report, "free controller", exc)
            self.robot.motors = {}
            self.robot.bus = None
            for name, camera in self.robot.cameras.items():
                try:
                    if camera.is_connected:
                        camera.disconnect()
                except Exception as exc:
                    self._cleanup_error(report, f"disconnect camera {name}", exc)
            self._report(report)
        return report

    def _disable(self, motors, bus, results):
        for result in results:
            result.reason = ("motor handle unavailable" if result.joint not in motors
                             else "no feedback observed after disable")
            if result.joint in motors:
                try:
                    # Preserve any cached pre-disable fault code for diagnosis.
                    self._observe(result, self.feedback_reader(motors[result.joint]), None)
                except Exception as exc:
                    self._axis_error(result, "get_state before disable", exc)
        for _ in range(self.policy.attempts):
            pending = [r for r in results if not r.confirmed and r.joint in motors]
            if not pending:
                break
            sent_at = {}
            for result in pending:
                result.attempts += 1
                started_ns = time.monotonic_ns()
                try:
                    motors[result.joint].disable()
                    result.disable_sent += 1
                    sent_at[result.joint] = started_ns
                except Exception as exc:
                    self._axis_error(result, "disable", exc)
                finally:
                    time.sleep(self.policy.interval_s)
            # The comparison option suppresses TX requests but preserves their
            # pacing slots, keeping the round timing otherwise unchanged.
            for result in pending:
                try:
                    if self.policy.request_feedback:
                        motors[result.joint].request_feedback()
                except Exception as exc:
                    self._axis_error(result, "request_feedback", exc)
                finally:
                    time.sleep(self.policy.interval_s)
            deadline = time.monotonic() + self.policy.feedback_wait_s
            while True:
                try:
                    if bus is not None:
                        bus.poll_feedback_once()
                except Exception as exc:
                    for result in pending:
                        self._axis_error(result, "poll_feedback_once", exc)
                for result in pending:
                    if result.confirmed:
                        continue
                    try:
                        sample = self.feedback_reader(motors[result.joint])
                        self._observe(result, sample, sent_at.get(result.joint))
                    except Exception as exc:
                        self._axis_error(result, "get_state", exc)
                remaining = deadline - time.monotonic()
                if remaining <= 0 or all(r.confirmed for r in pending):
                    break
                time.sleep(min(0.01, remaining))

    @staticmethod
    def _observe(result, sample, sent_ns):
        state = sample.state
        if state is None:
            result.reason = "no state available"
            return
        if state.can_id != result.send_id or state.arbitration_id != result.receive_id:
            result.reason = "feedback CAN identity mismatch"
            return
        result.cached_status_code = int(state.status_code)
        if result.cached_status_code not in result.observed_status_codes:
            result.observed_status_codes.append(result.cached_status_code)
        rx_ns = sample.received_monotonic_ns
        if rx_ns is None:
            result.reason = "MotorBridge has no feedback receive timestamp/counter"
        elif sent_ns is None or not sent_ns < rx_ns <= time.monotonic_ns():
            result.reason = "no fresh feedback after a successful disable call"
        elif result.cached_status_code != 0:
            result.reason = "fresh feedback does not report DISABLED"
        else:
            result.confirmed = True
            result.reason = "fresh feedback reports DISABLED"

    @staticmethod
    def _axis_error(result, operation, exc):
        message = f"{operation}: {type(exc).__name__}: {exc}"
        if message not in result.errors:
            result.errors.append(message)
        result.reason = message

    @staticmethod
    def _cleanup_error(report, operation, exc):
        message = f"{operation}: {type(exc).__name__}: {exc}"
        report.cleanup_errors.append(message)
        logger.error("VR shutdown: %s", message)

    def _report(self, report):
        for result in report.motors:
            status = {0: "DISABLED", 1: "ENABLED"}.get(result.cached_status_code,
                       f"RAW({result.cached_status_code})")
            outcome = ("SKIPPED" if not report.disable_requested else
                       "YES" if result.confirmed else "NO")
            log = logger.info if outcome != "NO" else logger.warning
            log("VR torque-off %s (ID %s): disable_sent=%s/%s, cached_status=%s, "
                "confirmed=%s (%s); status_history=%s; errors=%s", result.joint, result.send_id,
                result.disable_sent, result.attempts, status, outcome, result.reason,
                result.observed_status_codes, result.errors)
        logger.info("VR shutdown: close_bus_completed=%s, cleanup_errors=%s",
                    report.close_bus_completed, len(report.cleanup_errors))
        if self.report_path is not None:
            try:
                path = Path(self.report_path)
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(json.dumps(asdict(report), indent=2) + "\n", encoding="utf-8")
                logger.info("VR shutdown report: %s", path)
            except Exception as exc:
                self._cleanup_error(report, "write shutdown report", exc)

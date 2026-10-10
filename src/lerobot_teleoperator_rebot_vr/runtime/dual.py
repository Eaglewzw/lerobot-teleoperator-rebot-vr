"""One VR receiver, two independent CAN control loops, one fault coordinator."""

from __future__ import annotations

import argparse
from dataclasses import replace
import json
import logging
from pathlib import Path
import signal
import threading
import time

import yaml

from ..vr.adapter import sample_is_fresh
from ..vr.xr_v1 import BimanualSample, BimanualTrackingSource
from .dual_config import (DualConfig, load_dual_config, startup_zero_test_config,
                          validate_dual_rs_calibration)
from .real import ArmRuntime


logger = logging.getLogger(__name__)
SIDES = ("left", "right")
STARTUP_ZERO_TEST_HOLD_S = 2.0


class NoTrackingSource:
    """Diagnostic motion never opens a VR socket or accepts hand commands."""

    def start(self):
        pass

    def stop(self):
        pass

    def latest_pair(self):
        return BimanualSample()


class SharedHandController:
    """A view of the shared receiver; it never opens or closes a socket."""

    def __init__(self, session, side):
        self.session, self.side = session, side

    def connect(self):
        pass

    def disconnect(self):
        pass

    def latest_sample(self):
        return self.session.sample_for(self.side)


class DualArmSession:
    def __init__(self, config: DualConfig, source=None, *, clock_ns=time.monotonic_ns,
                 startup_zero_test=False):
        self.startup_zero_test = startup_zero_test
        self.config = startup_zero_test_config(config) if startup_zero_test else config
        self.source = (NoTrackingSource() if startup_zero_test else
                       source or BimanualTrackingSource(config.host, config.port,
                                                       on_status=logger.info))
        self._clock_ns = clock_ns
        self._stop = threading.Event()
        self._abort_return = threading.Event()
        self._return_requested = False
        self._lock = threading.RLock()
        self._ready = set()
        self._startup_connected = set()
        self._startup_release = threading.Event()
        self._feedback = {side: (False, 0) for side in SIDES}
        self._latched_fault = None
        self._generation = 0
        self._source_epoch = None
        self._allowed = False
        self._reason = "waiting for both arms"

    def hand_controller(self, side):
        return SharedHandController(self, side)

    def should_stop(self):
        return self._stop.is_set()

    def request_stop(self):
        """Non-SIGINT shutdown (error, duration, or unexpected worker exit)."""
        self._abort_return.set()
        self._stop.set()

    def handle_signal(self, signum, _frame=None):
        with self._lock:
            if signum == signal.SIGINT and not self.should_stop():
                self._return_requested = True
                now = self._clock_ns()
                # Capture feedback health now: it naturally stops refreshing
                # after shutdown begins, so do not age it during the return.
                if self._latched_fault or any(
                    not self._feedback[side][0]
                    or now - self._feedback[side][1] > self.config.heartbeat_timeout_s * 1e9
                    for side in self._ready
                ):
                    self._abort_return.set()
                self._stop.set()
            else:
                self.request_stop()

    def return_to_zero_requested(self):
        with self._lock:
            return self._return_requested and not self.zero_return_aborted()

    def zero_return_aborted(self):
        with self._lock:
            return self._abort_return.is_set() or self._latched_fault is not None

    def arm_ready(self, side):
        with self._lock:
            self._ready.add(side)

    def wait_for_startup(self, side, keep_alive):
        """Hold connected arms until both can begin their startup motion.

        IO stays outside the session lock. Stop/failure must interrupt this
        rendezvous, including when the peer never finishes connecting.
        """
        deadline = time.monotonic() + self.config.arms[side].initial_move_timeout
        if self.should_stop():
            return False
        keep_alive()
        with self._lock:
            if self.should_stop():
                return False
            self._startup_connected.add(side)
            if self._startup_connected == set(SIDES):
                self._startup_release.set()
        while not self.should_stop():
            if self._startup_release.wait(.01):
                return not self.should_stop()
            if time.monotonic() >= deadline:
                self.request_stop()
                raise RuntimeError(f"{side}: timed out waiting for both arms to connect")
            keep_alive()
        return False

    def arm_finished(self, side):
        # Any partial startup failure or unexpected exit stops the whole session.
        if not self.should_stop():
            self.request_stop()
        with self._lock:
            self._ready.discard(side)

    def report_feedback(self, side, valid):
        with self._lock:
            self._feedback[side] = (bool(valid), self._clock_ns())
            if not valid and self._return_requested:
                self._abort_return.set()
            self._refresh()

    def latch_fault(self, side, reason):
        with self._lock:
            if self._latched_fault is None:
                self._latched_fault = f"{side}: {reason}; restart required"
            self._refresh()

    def _refresh(self):
        """Caller holds the lock. Never wait for the other arm's IO or IK."""
        pair = self.source.latest_pair()
        now = self._clock_ns()
        reason = None
        if self.should_stop():
            reason = "session stopping"
        elif self._latched_fault:
            reason = self._latched_fault
        elif self._ready != set(SIDES):
            reason = "waiting for both arms"
        else:
            for side in SIDES:
                valid, timestamp = self._feedback[side]
                if not valid or now - timestamp > self.config.heartbeat_timeout_s * 1e9:
                    reason = f"{side} feedback invalid or control loop overdue"
                    break
                if not sample_is_fresh(getattr(pair, side), now,
                                       self.config.arms[side].stale_timeout):
                    reason = f"{side} tracking missing or stale"
                    break
        allowed = reason is None
        epochs = tuple(None if getattr(pair, s) is None else getattr(pair, s).stream_epoch
                       for s in SIDES)
        # Increment on every gate transition/connection epoch, including very
        # brief faults the other loop might not otherwise observe.
        if allowed != self._allowed or epochs != self._source_epoch:
            self._generation += 1
        if reason != self._reason:
            logger.info("dual session: %s", reason or "ready; release Grip before following")
        self._source_epoch = epochs
        self._allowed, self._reason = allowed, reason
        return pair

    def sample_for(self, side):
        with self._lock:
            pair = self._refresh()
            if not self._allowed:
                return None
            sample = getattr(pair, side)
            # Shared epoch forces both mappers to release/rearm after a fault.
            # Preserve each hand's buttons for the single-arm controller:
            # A/X -> home, B/Y -> zero, independently on the corresponding arm.
            return replace(sample, stream_epoch=self._generation)

    def run(self, runtime_factory=ArmRuntime):
        validate_dual_rs_calibration(self.config)
        errors = {}
        error_lock = threading.Lock()

        def run_arm(side):
            try:
                runtime_factory(self.config.arms[side], session=self, arm_id=side).run()
            except BaseException as exc:
                self.request_stop()
                with error_lock:
                    errors[side] = exc
                logger.exception("%s arm failed", side)
            finally:
                self.arm_finished(side)

        threads = []
        try:
            self.source.start()
            for side in SIDES:
                thread = threading.Thread(target=run_arm, args=(side,), name=f"rebot-{side}")
                thread.start()
                threads.append(thread)
            started = None
            while not self._stop.wait(0.01):
                with self._lock:
                    self._refresh()
                    if self._ready == set(SIDES) and started is None:
                        started = time.monotonic()
                if (started is not None and self.config.duration
                        and time.monotonic() - started >= self.config.duration):
                    self.request_stop()
                if (self.startup_zero_test and started is not None
                        and time.monotonic() - started >= STARTUP_ZERO_TEST_HOLD_S):
                    # Use the existing feedback-health gate and normal SIGINT
                    # return path, never home after an error or feedback fault.
                    self.handle_signal(signal.SIGINT)
        finally:
            if not self.should_stop():
                self.request_stop()
            # Each worker performs its own cleanup, even if its peer failed.
            try:
                for thread in threads:
                    thread.join()
            finally:
                self.source.stop()
        if errors:
            details = "; ".join(f"{side}: {error}" for side, error in errors.items())
            raise RuntimeError(f"dual-arm session failed: {details}") from next(iter(errors.values()))
        if self.startup_zero_test and (not self._return_requested or self.zero_return_aborted()):
            raise RuntimeError("startup-zero test interrupted or feedback unsafe; zero return not completed")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--dry-run", action="store_true",
                        help="validate and print effective settings without opening VR or CAN")
    parser.add_argument("--startup-zero-test", action="store_true",
                        help="REAL motion, no VR: cap speed/acceleration at 0.25 rad/s and "
                             "0.5 rad/s^2; startup, hold 2 s, then return to zero")
    args = parser.parse_args()
    try:
        config = load_dual_config(args.config)
        session = DualArmSession(config, startup_zero_test=args.startup_zero_test)
        config = session.config
    except (OSError, ValueError, TypeError, yaml.YAMLError) as exc:
        parser.error(str(exc))
    print(json.dumps({"vr": {"host": config.host, "port": config.port,
                              "enabled": not args.startup_zero_test},
                      "startup_zero_test": args.startup_zero_test,
                      "fault_policy": "hold_both", "duration": config.duration,
                      "heartbeat_timeout_s": config.heartbeat_timeout_s,
                      "arms": {s: vars(a) for s, a in config.arms.items()}},
                     indent=2, default=str), flush=True)
    if args.dry_run:
        return
    if any("REPLACE_" in arm.robot_port for arm in config.arms.values()):
        parser.error("replace both CAN adapter placeholders in the dual config first")
    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s %(levelname)s [%(threadName)s] %(message)s")
    if args.startup_zero_test:
        print("REAL low-speed startup/zero test: VR disabled; automatic return after 2 s. "
              "Support both arms before torque-off. No gain or tolerance relaxation.", flush=True)
    previous = {sig: signal.getsignal(sig) for sig in (signal.SIGINT, signal.SIGTERM)}
    try:
        for sig in previous:
            signal.signal(sig, session.handle_signal)
        session.run()
    finally:
        for sig, handler in previous.items():
            signal.signal(sig, handler)


if __name__ == "__main__":
    main()

"""B601-RS MotorBridge adapter. Public .pos values are degrees, wire values radians.

Motor encoder zeros must already match the RS URDF. This module never writes
zeros, clears faults, or substitutes cached/zero positions for a failed read.
"""

from dataclasses import dataclass, field
from concurrent.futures import ThreadPoolExecutor
import math
from pathlib import Path
import time

from ..constants import JOINT_NAMES
from .profiles import RS
from .rs_calibration import DEFAULT_ROBOT_ID, RSCalibrationStore


RS_MODELS = ("rs-06",) * 3 + ("rs-00",) * 4
MECH_POSITION = 0x7019


class RSConnection:
    """Opening and closing this transport does not enable or disable motors."""

    def __init__(self, port="can0", *, controller_factory=None):
        self.port = port
        self.controller_factory = controller_factory
        self.bus = None
        self.motors = {}
        self._reader = None

    def open(self):
        if self.bus is not None:
            raise RuntimeError("RS transport is already open")
        factory = self.controller_factory
        if factory is None:
            from motorbridge import Controller
            factory = Controller
        self.bus = factory(self.port)
        try:
            for index, (name, model) in enumerate(zip(JOINT_NAMES, RS_MODELS), 1):
                self.motors[name] = self.bus.add_robstride_motor(index, 0xFD, model)
            self._reader = ThreadPoolExecutor(max_workers=7, thread_name_prefix="rs-feedback")
        except BaseException as exc:
            try:
                self.close()
            except Exception as cleanup_error:
                exc.add_note(f"RS cleanup after open failure: {cleanup_error}")
            raise

    def read_position(self, name, timeout_ms=20):
        started_ns = time.monotonic_ns()
        # Explicit host avoids the SDK's fallback hosts and >=150ms-per-host
        # retry policy. Each call invalidates the parameter cache before sending.
        value = float(self.motors[name].robstride_get_param_f32_host_id(
            MECH_POSITION, 0xFD, timeout_ms))
        finished_ns = time.monotonic_ns()
        if not math.isfinite(value):
            raise RuntimeError(f"RS {name}: non-finite mechPos")
        return value, started_ns, finished_ns

    def read_positions(self, timeout_ms=20):
        """Overlap independent per-motor replies; fully join before any command.

        MotorBridge's parameter wait sleeps 8ms between polls. Its separate
        per-motor parameter locks allow seven reads to share that wait. There
        is only one outstanding request per motor, and this method never
        returns with C calls still using motor handles.
        """
        if self._reader is None:
            raise RuntimeError("RS transport is not open")
        futures = {name: self._reader.submit(self.read_position, name, timeout_ms)
                   for name in self.motors}
        results = {}
        for name, future in futures.items():
            try:
                results[name] = future.result()
            except Exception as exc:
                results[name] = exc
        return results

    def close(self):
        if self._reader is not None:
            self._reader.shutdown(wait=True)
            self._reader = None
        bus, motors = self.bus, self.motors
        self.bus, self.motors = None, {}
        if bus is None:
            return
        # Do not use Controller.__exit__/shutdown: they send disable commands.
        errors = []
        for operation in (bus.close_bus, *(motor.close for motor in motors.values()), bus.close):
            try:
                operation()
            except Exception as exc:
                errors.append(exc)
        if errors:
            raise RuntimeError(f"RS transport cleanup failed: {errors}") from errors[0]


@dataclass
class RSFollowerConfig:
    port: str = "can0"
    id: str = DEFAULT_ROBOT_ID
    calibration_dir: Path | None = None
    zero_confirmed: bool = False
    disable_torque_on_disconnect: bool = True
    gripper_control_mode: str = "mit"
    gripper_mit_kp: float = 5.
    gripper_mit_kd: float = .3
    max_relative_target: float | dict = 3.
    joint_limits: dict = field(default_factory=lambda: {
        name: (math.degrees(lo), math.degrees(hi))
        for name, lo, hi in zip(JOINT_NAMES[:6], RS.lower_rad, RS.upper_rad)
    })
    motor_can_ids: dict = field(default_factory=lambda: {
        name: (index, 0xFD) for index, name in enumerate(JOINT_NAMES, 1)
    })
    feedback_timeout_ms: int = 20
    max_feedback_span_s: float = .1


class RSFollower(RSConnection):
    """Follower-compatible connection; motion dispatch belongs to MITCommandDispatcher.

ManagedFollower owns cleanup, including partial-connect failures. Transport-only
probes use RSConnection instead and never enter this enable path.
"""

    status_reports_torque_state = False  # SDK status_code contains RS fault bits.

    def __init__(self, config: RSFollowerConfig, *, controller_factory=None):
        super().__init__(config.port, controller_factory=controller_factory)
        self.config = config
        self.calibration_store = RSCalibrationStore(config.id, config.calibration_dir)
        self.cameras = {}
        self.motor_names = list(JOINT_NAMES)
        self._connected = False
        self.feedback_intervals_ns = {}
        self._last_positions_rad = {}

    @property
    def is_calibrated(self):
        return self.config.zero_confirmed or bool(self.calibration_store.load())

    @property
    def is_connected(self):
        return self._connected and self.bus is not None

    def connect(self, calibrate=True):
        del calibrate  # --no-calibrate must never bypass the RS zero declaration.
        if not self.is_calibrated:
            raise RuntimeError("RS motor zeros are unconfirmed; run lerobot-calibrate --robot.type=rebot_b601_rs_follower with the same robot.id")
        from motorbridge import Mode
        self.open()
        # Validate ALL axes before mode changes or enable commands.
        self._check_startup_positions(self.get_observation())
        for motor in self.motors.values():
            motor.ensure_mode(Mode.MIT, 500)
        observation = self.get_observation()
        self._check_startup_positions(observation)
        # Seed the native controller with current positions, never a zero target.
        for name, motor in self.motors.items():
            target = observation[f"{name}.pos"]
            if name in self.config.joint_limits:
                lo, hi = self.config.joint_limits[name]
                target = min(hi, max(lo, target))
            motor.send_mit(math.radians(target), 0., 0., 0., 0.)
        for motor in self.motors.values():
            motor.enable()
        self._connected = True

    def _check_startup_positions(self, observation):
        for name, (lo, hi) in self.config.joint_limits.items():
            value = observation[f"{name}.pos"]
            tolerance = RS.feedback_tolerance_deg
            if not math.isfinite(value) or not lo - tolerance <= value <= hi + tolerance:
                raise RuntimeError(
                    f"RS {name}={value:.3f} deg outside [{lo:.3f}, {hi:.3f}] "
                    f"with feedback tolerance {tolerance:.3f} deg; check zero/limits"
                )

    def get_observation(self):
        if self.bus is None:
            raise RuntimeError("RS transport is not connected")
        self.feedback_intervals_ns = {}
        self._last_positions_rad = {}
        positions, intervals = {}, {}
        results = self.read_positions(self.config.feedback_timeout_ms)
        for name in self.motor_names:
            result = results[name]
            if isinstance(result, Exception):
                raise RuntimeError(f"RS feedback failed for {name}: {result}") from result
            pos, started_ns, finished_ns = result
            positions[name] = pos
            intervals[name] = (started_ns, finished_ns)
            state = self.motors[name].get_state()
            if state is not None and state.status_code:
                raise RuntimeError(f"RS {name} reports cached fault bits 0x{state.status_code:02x}")
        span = (max(end for _, end in intervals.values())
                - min(start for start, _ in intervals.values())) * 1e-9
        if span > self.config.max_feedback_span_s:
            raise RuntimeError(f"RS feedback read span {span:.3f}s exceeds freshness budget")
        self.feedback_intervals_ns = intervals
        self._last_positions_rad = positions
        return {f"{name}.pos": math.degrees(pos) for name, pos in positions.items()}

    def motor_feedback_telemetry(self):
        row = {"feedback_freshness": "rs_mechpos_request_response",
               "feedback_cache_read_monotonic_ns": time.monotonic_ns()}
        for name, pos in self._last_positions_rad.items():
            if name != "gripper":
                row[f"cached_position_{name}_rad"] = pos
        # RS firmware velocity/torque cache semantics are not verified. Leave
        # those optional fields empty rather than claiming actual measurements.
        return row

    def send_action(self, action):
        raise RuntimeError("RS actions must go through MITCommandDispatcher")

    def prepare_disconnect(self):
        # ManagedFollower will close bus/motor handles after this hook.
        if self._reader is not None:
            self._reader.shutdown(wait=True)
            self._reader = None
        self._connected = False

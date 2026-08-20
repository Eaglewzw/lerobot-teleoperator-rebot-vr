from __future__ import annotations

import threading
import time
from dataclasses import dataclass

import numpy as np

from .pose_mapping import PoseTarget


def _readonly_vector(value: object, size: int, name: str) -> np.ndarray:
    array = np.asarray(value, dtype=np.float64)
    if array.shape != (size,) or not np.all(np.isfinite(array)):
        raise ValueError(f"{name} must be a finite vector of shape ({size},)")
    result = array.copy()
    result.flags.writeable = False
    return result


@dataclass(frozen=True, init=False)
class IKRequest:
    sequence: int
    generation: int
    sample_id: int
    target_position: np.ndarray
    target_rotation: np.ndarray
    q_seed: np.ndarray
    q_actual: np.ndarray
    dq_previous: np.ndarray
    q_nominal: np.ndarray
    dt: float
    submitted_monotonic_ns: int

    def __init__(self, *, sequence: int, generation: int, sample_id: int | None = None,
                 target_position: np.ndarray | None = None, target_rotation: np.ndarray | None = None,
                 q_seed: np.ndarray | None = None, submitted_monotonic_ns: int | None = None,
                 target: PoseTarget | None = None, q_actual: np.ndarray | None = None,
                 dq_previous: np.ndarray | None = None, q_nominal: np.ndarray | None = None,
                 dt: float = 0.01) -> None:
        if target is not None:
            if sample_id is not None or target_position is not None or target_rotation is not None:
                raise ValueError("use either target or explicit target fields")
            sample_id, target_position, target_rotation = target.sample_id, target.position, target.rotation
        if sample_id is None or target_position is None or target_rotation is None or q_seed is None:
            raise ValueError("sample_id, target pose, and q_seed are required")
        sequence = int(sequence); generation = int(generation); sample_id = int(sample_id)
        submitted = time.monotonic_ns() if submitted_monotonic_ns is None else int(submitted_monotonic_ns)
        if sequence <= 0 or generation < 0 or sample_id < 0 or submitted < 0:
            raise ValueError("invalid request identity")
        rotation = np.asarray(target_rotation, dtype=np.float64)
        if rotation.shape != (3, 3) or not np.all(np.isfinite(rotation)):
            raise ValueError("target_rotation must be a finite 3x3 matrix")
        if not np.isfinite(float(dt)) or float(dt) <= 0.0:
            raise ValueError("dt must be finite and positive")
        object.__setattr__(self, "sequence", sequence)
        object.__setattr__(self, "generation", generation)
        object.__setattr__(self, "sample_id", sample_id)
        object.__setattr__(self, "target_position", _readonly_vector(target_position, 3, "target_position"))
        object.__setattr__(self, "target_rotation", rotation.copy())
        object.__setattr__(self, "q_seed", _readonly_vector(q_seed, 6, "q_seed"))
        object.__setattr__(self, "q_actual", _readonly_vector(q_seed if q_actual is None else q_actual, 6, "q_actual"))
        object.__setattr__(self, "dq_previous", _readonly_vector(np.zeros(6) if dq_previous is None else dq_previous, 6, "dq_previous"))
        object.__setattr__(self, "q_nominal", _readonly_vector(q_seed if q_nominal is None else q_nominal, 6, "q_nominal"))
        object.__setattr__(self, "dt", float(dt))
        object.__setattr__(self, "submitted_monotonic_ns", submitted)

    @property
    def target(self) -> PoseTarget:
        return PoseTarget(self.sample_id, self.target_position, self.target_rotation)

    @property
    def q_seed_rad(self) -> np.ndarray:
        return self.q_seed


@dataclass(frozen=True)
class IKResult:
    generation: int
    sequence: int
    sample_id: int
    q_target_rad: np.ndarray
    success: bool
    position_error_m: float
    solve_time_ms: float
    reason: str = ""

    def __post_init__(self) -> None:
        object.__setattr__(self, "q_target_rad", _readonly_vector(self.q_target_rad, 6, "q_target_rad"))
        if self.sequence <= 0 or self.generation < 0 or self.sample_id < 0:
            raise ValueError("invalid result identity")
        if not np.isfinite(self.solve_time_ms) or self.solve_time_ms < 0.0:
            raise ValueError("solve_time_ms must be finite and non-negative")

    @property
    def q_goal(self) -> np.ndarray:
        return self.q_target_rad


class LatestOnlyQPIKWorker:
    """Latest-only asynchronous worker for six-axis TCP QP IK."""

    def __init__(self, solver, *, max_joint_speed_rad_s: np.ndarray,
                 max_joint_acceleration_rad_s2: np.ndarray) -> None:
        self.solver = solver
        self.max_speed = np.asarray(max_joint_speed_rad_s, dtype=np.float64).copy()
        self.max_acceleration = np.asarray(max_joint_acceleration_rad_s2, dtype=np.float64).copy()
        if self.max_speed.shape != (6,) or self.max_acceleration.shape != (6,) or not np.all(np.isfinite(self.max_speed)) or not np.all(np.isfinite(self.max_acceleration)) or np.any(self.max_speed <= 0) or np.any(self.max_acceleration <= 0):
            raise ValueError("QP speed and acceleration limits must be positive six-vectors")
        self._condition = threading.Condition()
        self._pending = None
        self._latest_result = None
        self._running = False
        self._stop = False
        self._thread = None
        self.submitted = self.solved = self.rejected = 0

    def start(self) -> None:
        if self._thread is None:
            self._stop = False
            self._thread = threading.Thread(target=self._run, name="rebot-vr-qp", daemon=True)
            self._thread.start()

    def stop(self) -> None:
        with self._condition:
            self._stop = True
            self._condition.notify_all()
        if self._thread is not None:
            self._thread.join()
            self._thread = None

    def clear(self) -> None:
        with self._condition:
            self._pending = None
            self._latest_result = None

    def submit(self, request: IKRequest) -> None:
        with self._condition:
            self._pending = request
            self.submitted += 1
            self._condition.notify()

    def latest_result(self) -> IKResult | None:
        with self._condition:
            return self._latest_result

    @property
    def has_pending_or_running(self) -> bool:
        with self._condition:
            return self._pending is not None or self._running

    def _run(self) -> None:
        while True:
            with self._condition:
                while self._pending is None and not self._stop:
                    self._condition.wait()
                if self._stop:
                    return
                request = self._pending
                self._pending = None
                self._running = True
            result = self._solve(request)
            with self._condition:
                self._latest_result = result
                self._running = False

    def _solve(self, request: IKRequest) -> IKResult:
        started = time.monotonic_ns()
        try:
            q, success, error, solve_ms, reason = self.solver.solve(
                target_position=request.target_position, target_rotation=request.target_rotation,
                q_actual=request.q_actual, dq_previous=request.dq_previous, dt=request.dt,
                q_nominal=request.q_nominal, max_joint_speed=self.max_speed,
                max_joint_acceleration=self.max_acceleration,
            )
            q = np.asarray(q, dtype=np.float64)
            valid = q.shape == (6,) and np.all(np.isfinite(q))
            if not success or not valid:
                self.rejected += 1
                q = request.q_seed.copy()
            else:
                # The solver returns the next target from the measured feedback.
                # Do not add this increment to q_seed again: doing so turns
                # feedback latency into an uncontrolled target integrator.
                self.solved += 1
            return IKResult(request.generation, request.sequence, request.sample_id, q, bool(success and valid), float(error) if np.isfinite(error) else float("nan"), float(solve_ms), reason)
        except Exception as exc:
            self.rejected += 1
            return IKResult(request.generation, request.sequence, request.sample_id, request.q_seed.copy(), False, float("nan"), (time.monotonic_ns() - started) * 1e-6, f"solver_exception:{type(exc).__name__}")

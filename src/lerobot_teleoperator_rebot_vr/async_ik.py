from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Protocol

import numpy as np

from .pose_mapping import PoseTarget


class PositionIK(Protocol):
    lower_position_limit: np.ndarray
    upper_position_limit: np.ndarray

    def solve_position(
        self,
        target_position: np.ndarray,
        q_init_rad: np.ndarray,
        *,
        max_iterations: int,
        tolerance_m: float,
        damping: float,
        step_size: float,
        control_mode: str,
        active_joint_indices: tuple[int, ...],
    ) -> tuple[np.ndarray, bool, float]: ...


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
    q_seed: np.ndarray
    submitted_monotonic_ns: int
    target_rotation: np.ndarray

    def __init__(
        self,
        *,
        sequence: int,
        generation: int,
        sample_id: int | None = None,
        target_position: np.ndarray | None = None,
        q_seed: np.ndarray | None = None,
        submitted_monotonic_ns: int | None = None,
        target: PoseTarget | None = None,
        q_seed_rad: np.ndarray | None = None,
        wrist_target_rad: np.ndarray | None = None,
    ) -> None:
        if target is not None:
            if sample_id is not None or target_position is not None:
                raise ValueError("use either target or sample_id/target_position")
            sample_id = target.sample_id
            target_position = target.position
            target_rotation = target.rotation
        else:
            target_rotation = np.eye(3, dtype=np.float64)
        if sample_id is None or target_position is None:
            raise ValueError("sample_id and target_position are required")
        if q_seed is not None and q_seed_rad is not None:
            raise ValueError("use either q_seed or q_seed_rad")
        seed_value = q_seed if q_seed is not None else q_seed_rad
        if seed_value is None:
            raise ValueError("q_seed is required")

        sequence_value = int(sequence)
        generation_value = int(generation)
        sample_id_value = int(sample_id)
        submitted_ns = (
            time.monotonic_ns()
            if submitted_monotonic_ns is None
            else int(submitted_monotonic_ns)
        )
        if sequence_value <= 0:
            raise ValueError("sequence must be positive")
        if generation_value < 0 or sample_id_value < 0 or submitted_ns < 0:
            raise ValueError(
                "generation, sample_id, and submission time must be non-negative"
            )
        position = _readonly_vector(target_position, 3, "target_position")
        seed = _readonly_vector(seed_value, 6, "q_seed").copy()
        if wrist_target_rad is not None:
            seed[3:6] = _readonly_vector(
                wrist_target_rad, 3, "wrist_target_rad"
            )
        seed.flags.writeable = False
        rotation = np.asarray(target_rotation, dtype=np.float64)
        if rotation.shape != (3, 3) or not np.all(np.isfinite(rotation)):
            raise ValueError("target_rotation must be a finite 3x3 matrix")
        rotation = rotation.copy()
        rotation.flags.writeable = False

        object.__setattr__(self, "sequence", sequence_value)
        object.__setattr__(self, "generation", generation_value)
        object.__setattr__(self, "sample_id", sample_id_value)
        object.__setattr__(self, "target_position", position)
        object.__setattr__(self, "q_seed", seed)
        object.__setattr__(self, "submitted_monotonic_ns", submitted_ns)
        object.__setattr__(self, "target_rotation", rotation)

    @property
    def target(self) -> PoseTarget:
        return PoseTarget(self.sample_id, self.target_position, self.target_rotation)

    @property
    def q_seed_rad(self) -> np.ndarray:
        return self.q_seed

    @property
    def wrist_target_rad(self) -> np.ndarray:
        return self.q_seed[3:6]


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
        q_target = _readonly_vector(self.q_target_rad, 6, "q_target_rad")
        if self.sequence <= 0 or self.generation < 0 or self.sample_id < 0:
            raise ValueError("invalid IK result identity")
        if not np.isfinite(self.solve_time_ms) or self.solve_time_ms < 0.0:
            raise ValueError("solve_time_ms must be finite and non-negative")
        object.__setattr__(self, "q_target_rad", q_target)

    @property
    def q_goal(self) -> np.ndarray:
        return self.q_target_rad


class LatestOnlyIKWorker:
    """Solve only the newest pending target so old VR frames cannot queue up."""

    def __init__(
        self,
        kinematics: PositionIK,
        *,
        rate_hz: float = 100.0,
        max_iterations: int = 50,
        tolerance_m: float = 5e-4,
        damping: float = 1e-4,
        max_solution_jump_rad: float = 0.5,
        lower_limit_rad: np.ndarray | None = None,
        upper_limit_rad: np.ndarray | None = None,
    ) -> None:
        self.kinematics = kinematics
        rate_hz = float(rate_hz)
        if not np.isfinite(rate_hz) or rate_hz <= 0.0:
            raise ValueError("rate_hz must be finite and positive")
        self.period_s = 1.0 / rate_hz
        self.max_iterations = int(max_iterations)
        self.tolerance_m = float(tolerance_m)
        self.damping = float(damping)
        self.max_solution_jump_rad = float(max_solution_jump_rad)
        if self.max_iterations <= 0:
            raise ValueError("max_iterations must be positive")
        if not np.isfinite(self.tolerance_m) or self.tolerance_m <= 0.0:
            raise ValueError("tolerance_m must be finite and positive")
        if not np.isfinite(self.damping) or self.damping < 0.0:
            raise ValueError("damping must be finite and non-negative")
        if (
            not np.isfinite(self.max_solution_jump_rad)
            or self.max_solution_jump_rad <= 0.0
        ):
            raise ValueError("max_solution_jump_rad must be finite and positive")
        self.lower_limit_rad = np.asarray(
            kinematics.lower_position_limit if lower_limit_rad is None else lower_limit_rad,
            dtype=np.float64,
        )
        self.upper_limit_rad = np.asarray(
            kinematics.upper_position_limit if upper_limit_rad is None else upper_limit_rad,
            dtype=np.float64,
        )
        if (
            self.lower_limit_rad.shape != (6,)
            or self.upper_limit_rad.shape != (6,)
            or not np.all(np.isfinite(self.lower_limit_rad))
            or not np.all(np.isfinite(self.upper_limit_rad))
            or np.any(self.lower_limit_rad > self.upper_limit_rad)
        ):
            raise ValueError("IK joint limits must be finite ordered six-vectors")

        self._condition = threading.Condition()
        self._pending: IKRequest | None = None
        self._latest_result: IKResult | None = None
        self._last_success: np.ndarray | None = None
        self._last_success_generation: int | None = None
        self._stop = False
        self._thread: threading.Thread | None = None
        self.submitted = 0
        self.solved = 0
        self.rejected = 0

    def start(self) -> None:
        if self._thread is not None:
            return
        self._stop = False
        self._thread = threading.Thread(target=self._run, name="rebot-vr-ik", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        with self._condition:
            self._stop = True
            self._condition.notify_all()
        if self._thread is not None:
            self._thread.join()
        self._thread = None

    def submit(self, request: IKRequest) -> None:
        with self._condition:
            self._pending = request
            self.submitted += 1
            self._condition.notify()

    def clear(self) -> None:
        with self._condition:
            self._pending = None
            self._latest_result = None
            self._last_success = None
            self._last_success_generation = None

    def latest_result(self) -> IKResult | None:
        with self._condition:
            return self._latest_result

    def _run(self) -> None:
        next_solve_s = 0.0
        while True:
            with self._condition:
                while True:
                    if self._stop:
                        return
                    if self._pending is None:
                        self._condition.wait()
                        continue
                    delay_s = next_solve_s - time.monotonic()
                    if delay_s > 0.0:
                        # submit() may replace _pending while rate limiting.
                        self._condition.wait(timeout=delay_s)
                        continue
                    request = self._pending
                    self._pending = None
                    break

            assert request is not None
            solve_started_s = time.monotonic()
            result = self._solve(request)
            next_solve_s = solve_started_s + self.period_s
            with self._condition:
                self._latest_result = result

    def _solve(self, request: IKRequest) -> IKResult:
        started_ns = time.monotonic_ns()
        seed = request.q_seed.copy()
        if (
            self._last_success is not None
            and self._last_success_generation == request.generation
        ):
            seed[:3] = self._last_success[:3]
        seed[3:6] = request.q_seed[3:6]

        try:
            candidate, success, error_m = self.kinematics.solve_position(
                request.target_position,
                seed,
                max_iterations=self.max_iterations,
                tolerance_m=self.tolerance_m,
                damping=self.damping,
                step_size=0.5,
                control_mode="position",
                active_joint_indices=(0, 1, 2),
            )
        except Exception as exc:
            self.rejected += 1
            return IKResult(
                generation=request.generation,
                sequence=request.sequence,
                sample_id=request.sample_id,
                q_target_rad=seed,
                success=False,
                position_error_m=float("nan"),
                solve_time_ms=(time.monotonic_ns() - started_ns) * 1e-6,
                reason=f"solver_exception:{type(exc).__name__}",
            )
        error_value = self._error_value(error_m)
        try:
            candidate = np.asarray(candidate, dtype=np.float64)
        except (TypeError, ValueError):
            candidate = np.empty(0, dtype=np.float64)
        try:
            success = bool(success)
        except (TypeError, ValueError):
            success = False
            candidate = np.empty(0, dtype=np.float64)
        if candidate.shape != (6,) or not np.all(np.isfinite(candidate)):
            self.rejected += 1
            return IKResult(
                generation=request.generation,
                sequence=request.sequence,
                sample_id=request.sample_id,
                q_target_rad=seed,
                success=False,
                position_error_m=error_value,
                solve_time_ms=(time.monotonic_ns() - started_ns) * 1e-6,
                reason="invalid_solution",
            )
        reason = ""
        if success and np.max(np.abs(candidate[:3] - request.q_seed[:3])) > self.max_solution_jump_rad:
            success = False
            reason = "branch_jump"
        elif not success:
            reason = "position_not_converged"

        candidate = np.clip(candidate, self.lower_limit_rad, self.upper_limit_rad)
        candidate[3:6] = np.clip(
            request.q_seed[3:6],
            self.lower_limit_rad[3:6],
            self.upper_limit_rad[3:6],
        )

        if success:
            self._last_success = candidate.copy()
            self._last_success_generation = request.generation
            self.solved += 1
        else:
            self.rejected += 1

        return IKResult(
            generation=request.generation,
            sequence=request.sequence,
            sample_id=request.sample_id,
            q_target_rad=candidate,
            success=success,
            position_error_m=error_value,
            solve_time_ms=(time.monotonic_ns() - started_ns) * 1e-6,
            reason=reason,
        )

    @staticmethod
    def _error_value(value: object) -> float:
        try:
            parsed = float(value)
        except (TypeError, ValueError, OverflowError):
            return float("nan")
        return parsed if np.isfinite(parsed) and parsed >= 0.0 else float("nan")

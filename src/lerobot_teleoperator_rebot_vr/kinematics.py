"""B601-DM forward kinematics and full-body differential IK."""

from __future__ import annotations

import threading
import time
from contextlib import ExitStack
from importlib.resources import as_file, files
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
from scipy.optimize import minimize


NUM_ARM_JOINTS = 6


class FullBodyQPIKSolver:
    """Convex differential TCP IK with box constraints on joint velocity."""

    def __init__(self, kinematics: "B601Kinematics", *, solver: str = "scipy", position_cost: float = 20.0,
                 orientation_cost: float = 2.0, damping: float = 1e-3,
                 smoothness_cost: float = 0.05, posture_cost: float = 0.01,
                 joint_limit_margin_rad: float = 0.03, max_solve_time_ms: float = 8.0) -> None:
        self.kinematics = kinematics
        self.solver = str(solver).lower()
        if self.solver not in ("scipy", "osqp"):
            raise ValueError("qp solver must be scipy or osqp")
        values = (position_cost, orientation_cost, damping, smoothness_cost, posture_cost,
                  joint_limit_margin_rad, max_solve_time_ms)
        if not np.all(np.isfinite(values)) or position_cost <= 0 or orientation_cost < 0 or damping < 0 or smoothness_cost < 0 or posture_cost < 0 or joint_limit_margin_rad < 0 or max_solve_time_ms <= 0:
            raise ValueError("invalid QP parameters")
        self.position_cost = float(position_cost)
        self.orientation_cost = float(orientation_cost)
        self.damping = float(damping)
        self.smoothness_cost = float(smoothness_cost)
        self.posture_cost = float(posture_cost)
        self.joint_limit_margin_rad = float(joint_limit_margin_rad)
        self.max_solve_time_ms = float(max_solve_time_ms)

    def solve(self, *, target_position: object, target_rotation: object, q_actual: object,
              dq_previous: object, dt: float, q_nominal: object,
              max_joint_speed: object, max_joint_acceleration: object) -> tuple[np.ndarray, bool, float, float, str]:
        started = time.monotonic_ns()
        q = np.asarray(q_actual, dtype=np.float64)
        dq_prev = np.asarray(dq_previous, dtype=np.float64)
        q_nom = np.asarray(q_nominal, dtype=np.float64)
        speed = np.asarray(max_joint_speed, dtype=np.float64)
        accel = np.asarray(max_joint_acceleration, dtype=np.float64)
        if any(v.shape != (6,) for v in (q, dq_prev, q_nom, speed, accel)) or not all(np.all(np.isfinite(v)) for v in (q, dq_prev, q_nom, speed, accel)):
            return np.zeros(6), False, float("inf"), 0.0, "invalid_input"
        dt = float(dt)
        if not np.isfinite(dt) or dt <= 0.0:
            return np.zeros(6), False, float("inf"), 0.0, "invalid_dt"
        lower = np.asarray(self.kinematics.lower_position_limit, dtype=np.float64)
        upper = np.asarray(self.kinematics.upper_position_limit, dtype=np.float64)
        if np.any(q < lower) or np.any(q > upper):
            return np.zeros(6), False, float("inf"), (time.monotonic_ns()-started)*1e-6, "feedback_outside_limits"
        # Keep zero feasible while the feedback is in the inward margin; only
        # enforce the margin when moving toward an already-safe limit.
        safe_lo = np.where(q <= lower + self.joint_limit_margin_rad, q, lower + self.joint_limit_margin_rad)
        safe_hi = np.where(q >= upper - self.joint_limit_margin_rad, q, upper - self.joint_limit_margin_rad)
        # dq_prev is a velocity (rad/s), not a per-step displacement.  The
        # acceleration constraint must therefore limit the change from the
        # previous velocity: |dq - dq_prev| <= acceleration * dt.  Only bound
        # an out-of-range/stale previous velocity to the configured speed
        # envelope; clipping it to acceleration*dt would silently reduce the
        # reachable speed to roughly 2*acceleration*dt.
        dq_constraint_prev = np.clip(dq_prev, -speed, speed)
        lo = np.maximum.reduce(
            (-speed, (safe_lo - q) / dt, dq_constraint_prev - accel * dt)
        )
        hi = np.minimum.reduce(
            (speed, (safe_hi - q) / dt, dq_constraint_prev + accel * dt)
        )
        if np.any(lo > hi + 1e-10):
            return np.zeros(6), False, float("inf"), (time.monotonic_ns()-started)*1e-6, "infeasible_constraints"
        try:
            error = self.kinematics.tcp_pose_error(q, target_position, target_rotation)
            jac = self.kinematics.tcp_jacobian(q)
            wp = np.sqrt(self.position_cost)
            wo = np.sqrt(self.orientation_cost)
            A = np.vstack((wp * jac[:3], wo * jac[3:], np.sqrt(self.damping) * np.eye(6),
                           np.sqrt(self.smoothness_cost) * np.eye(6), np.sqrt(self.posture_cost) * dt * np.eye(6)))
            b = np.concatenate((wp * error[:3] / dt, wo * error[3:] / dt,
                                np.zeros(6), np.sqrt(self.smoothness_cost) * dq_constraint_prev,
                                np.sqrt(self.posture_cost) * (q_nom - q)))
            H = A.T @ A + 1e-12 * np.eye(6)
            g = -(A.T @ b)
            x0 = np.clip(dq_prev, lo, hi)
            if self.solver == "osqp":
                import osqp  # optional, explicit backend only
                from scipy import sparse
                problem = osqp.OSQP()
                problem.setup(P=sparse.csc_matrix(H), q=g, A=sparse.eye(6), l=lo, u=hi,
                              verbose=False, time_limit=self.max_solve_time_ms / 1000.0)
                result = problem.solve()
                dq = np.asarray(result.x, dtype=np.float64) if result.x is not None else x0
                ok = result.info.status.lower().startswith("solved")
            else:
                deadline = started + int(self.max_solve_time_ms * 1e6)
                def fun(x): return 0.5 * float(x @ H @ x) + float(g @ x)
                def jac_fun(x): return H @ x + g
                result = minimize(fun, x0, jac=jac_fun, bounds=list(zip(lo, hi)), method="L-BFGS-B",
                                  options={"maxiter": 40, "ftol": 1e-10, "gtol": 1e-7, "maxls": 10})
                dq = np.asarray(result.x if result.x is not None else x0, dtype=np.float64)
                ok = bool(result.success) or np.linalg.norm(jac_fun(dq), np.inf) < 1e-4
                if time.monotonic_ns() > deadline:
                    ok = False
                    return np.zeros(6), False, float(np.linalg.norm(error[:3])), (time.monotonic_ns()-started)*1e-6, "solve_timeout"
            if dq.shape != (6,) or not np.all(np.isfinite(dq)) or np.any(dq < lo - 1e-7) or np.any(dq > hi + 1e-7):
                return np.zeros(6), False, float(np.linalg.norm(error[:3])), (time.monotonic_ns()-started)*1e-6, "invalid_solution"
            q_next = q + dq * dt
            residual = self.kinematics.tcp_pose_error(q_next, target_position, target_rotation)
            return q_next, ok, float(np.linalg.norm(residual[:3])), (time.monotonic_ns()-started)*1e-6, ("" if ok else "qp_failed")
        except Exception as exc:
            return np.zeros(6), False, float("inf"), (time.monotonic_ns()-started)*1e-6, f"solver_exception:{type(exc).__name__}"


def default_urdf_path() -> Path:
    return Path(
        str(files("lerobot_teleoperator_rebot_vr").joinpath("assets/rebot_b601_dm_kinematics.urdf"))
    )


class B601Kinematics:
    """Pinocchio DLS solver for the packaged B601-DM kinematic chain."""

    def __init__(
        self,
        urdf_path: str | Path | None = None,
        end_effector_frame: str = "gripper_end",
    ) -> None:
        try:
            import pinocchio as pin
        except ImportError as exc:
            raise ImportError(
                "B601 IK requires Pinocchio. Install this package with its 'ik' extra."
            ) from exc

        self.pin = pin
        self._resource_stack = ExitStack()
        if urdf_path is None:
            resource = files("lerobot_teleoperator_rebot_vr").joinpath(
                "assets/rebot_b601_dm_kinematics.urdf"
            )
            self.urdf_path = Path(self._resource_stack.enter_context(as_file(resource)))
        else:
            self.urdf_path = Path(urdf_path)
        if not self.urdf_path.is_file():
            raise FileNotFoundError(f"B601-DM URDF not found: {self.urdf_path}")
        self.model = pin.buildModelFromUrdf(str(self.urdf_path))
        if self.model.nq != NUM_ARM_JOINTS:
            raise ValueError(f"expected a 6-DOF B601 model, got nq={self.model.nq}")
        self._thread_local = threading.local()
        self.frame_id = self.model.getFrameId(end_effector_frame)
        if self.frame_id >= self.model.nframes:
            raise ValueError(f"end-effector frame not found: {end_effector_frame}")
        self.end_effector_frame = end_effector_frame

    def close(self) -> None:
        self._resource_stack.close()

    def __del__(self) -> None:
        resource_stack = getattr(self, "_resource_stack", None)
        if resource_stack is not None:
            resource_stack.close()

    @property
    def lower_position_limit(self) -> np.ndarray:
        return np.asarray(self.model.lowerPositionLimit[:NUM_ARM_JOINTS], dtype=float).copy()

    @property
    def upper_position_limit(self) -> np.ndarray:
        return np.asarray(self.model.upperPositionLimit[:NUM_ARM_JOINTS], dtype=float).copy()

    def forward_kinematics(self, q_rad: object) -> tuple[np.ndarray, np.ndarray]:
        """Return the complete gripper_end TCP pose."""
        q = self._joint_vector(q_rad)
        data = self._thread_data()
        self.pin.framesForwardKinematics(self.model, data, q)
        pose = data.oMf[self.frame_id]
        return np.asarray(pose.translation, dtype=float).copy(), np.asarray(pose.rotation, dtype=float).copy()

    def tcp_jacobian(self, q_rad: object) -> np.ndarray:
        """Return gripper_end Jacobian as [linear_world; angular_world]."""
        q = self._joint_vector(q_rad)
        data = self._thread_data()
        self.pin.computeJointJacobians(self.model, data, q)
        self.pin.updateFramePlacements(self.model, data)
        jacobian = self.pin.getFrameJacobian(
            self.model, data, self.frame_id, self.pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
        )
        return np.asarray(jacobian, dtype=np.float64)[:, :NUM_ARM_JOINTS].copy()

    def tcp_pose_error(self, q_rad: object, target_position: object, target_rotation: object) -> np.ndarray:
        """World-frame SE(3) error [position; rotation-vector], target minus actual."""
        position, rotation = self.forward_kinematics(q_rad)
        target_p = np.asarray(target_position, dtype=np.float64)
        if target_p.shape != (3,) or not np.all(np.isfinite(target_p)):
            raise ValueError("target_position must be a finite three-vector")
        target_r = self._rotation_matrix(target_rotation, "target_rotation")
        rotvec = Rotation.from_matrix(target_r @ rotation.T).as_rotvec()
        return np.concatenate((target_p - position, rotvec))


    def _thread_data(self):
        data = getattr(self._thread_local, "data", None)
        if data is None:
            data = self.model.createData()
            self._thread_local.data = data
        return data

    @staticmethod
    def _joint_vector(value: object) -> np.ndarray:
        q = np.asarray(value, dtype=float)
        if q.shape != (NUM_ARM_JOINTS,) or not np.all(np.isfinite(q)):
            raise ValueError("joint vector must contain six finite values")
        return q.copy()

    @staticmethod
    def _rotation_matrix(value: object, name: str) -> np.ndarray:
        matrix = np.asarray(value, dtype=np.float64)
        if (
            matrix.shape != (3, 3)
            or not np.all(np.isfinite(matrix))
            or not np.allclose(matrix.T @ matrix, np.eye(3), atol=1e-6)
            or not np.isclose(np.linalg.det(matrix), 1.0, atol=1e-6)
        ):
            raise ValueError(f"{name} must be a right-handed rotation matrix")
        return matrix.copy()


__all__ = ["B601Kinematics", "FullBodyQPIKSolver", "NUM_ARM_JOINTS", "default_urdf_path"]

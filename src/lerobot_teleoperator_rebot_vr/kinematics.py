"""B601-DM forward and position-only inverse kinematics."""

from __future__ import annotations

import threading
from contextlib import ExitStack
from importlib.resources import as_file, files
from pathlib import Path

import numpy as np


NUM_ARM_JOINTS = 6
POSITION_JOINT_INDICES = np.asarray((0, 1, 2), dtype=int)


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
        self.wrist_joint_id = self.model.getJointId("joint4")
        if self.wrist_joint_id <= 0 or self.wrist_joint_id >= self.model.njoints:
            raise ValueError("joint4 is required for closed-form wrist orientation")

        zero_q = np.zeros(NUM_ARM_JOINTS, dtype=np.float64)
        self._wrist_frame_constant = (
            self.wrist_base_rotation(zero_q[:3]).T
            @ self.forward_kinematics(zero_q)[1]
        )
        self._wrist_frame_constant.flags.writeable = False
        probe_q123 = np.array([0.3, -0.9, -0.7], dtype=np.float64)
        probe_constant = (
            self.wrist_base_rotation(probe_q123).T
            @ self.forward_kinematics(
                np.concatenate((probe_q123, np.zeros(3, dtype=np.float64)))
            )[1]
        )
        if not np.allclose(
            probe_constant, self._wrist_frame_constant, atol=1e-6, rtol=0.0
        ):
            raise ValueError(
                "B601 wrist orientation chain is not independent of q1-q3"
            )

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

    @property
    def wrist_frame_constant(self) -> np.ndarray:
        return self._wrist_frame_constant.copy()

    def forward_kinematics(self, q_rad: object) -> tuple[np.ndarray, np.ndarray]:
        """Return the gripper_end pose used by the wrist orientation solver."""
        q = self._joint_vector(q_rad)
        data = self._thread_data()
        self.pin.framesForwardKinematics(self.model, data, q)
        pose = data.oMf[self.frame_id]
        return np.asarray(pose.translation, dtype=float).copy(), np.asarray(pose.rotation, dtype=float).copy()

    def wrist_center_position(self, q_rad: object) -> np.ndarray:
        """Return the joint4 origin controlled by the q1-q3 position IK.

        The joint4 origin is the end of the three-axis arm and is independent of
        q4-q6.  It is intentionally distinct from the gripper_end TCP.
        """
        q = self._joint_vector(q_rad)
        data = self._thread_data()
        self.pin.forwardKinematics(self.model, data, q)
        return np.asarray(
            data.oMi[self.wrist_joint_id].translation, dtype=float
        ).copy()

    def wrist_base_rotation(self, q123_rad: object) -> np.ndarray:
        """Return the joint4 pre-rotation frame orientation for q1-q3."""
        q123 = np.asarray(q123_rad, dtype=np.float64)
        if q123.shape != (3,) or not np.all(np.isfinite(q123)):
            raise ValueError("q123_rad must be a finite three-vector")
        q = np.zeros(NUM_ARM_JOINTS, dtype=np.float64)
        q[:3] = q123
        data = self._thread_data()
        self.pin.forwardKinematics(self.model, data, q)
        return np.asarray(data.oMi[self.wrist_joint_id].rotation, dtype=float).copy()

    def solve_wrist_orientation(
        self,
        q123_rad: object,
        target_rotation: object,
        *,
        previous_q4_rad: float = 0.0,
    ) -> tuple[np.ndarray, bool]:
        """Solve q4-q6 exactly from q1-q3 and a target end-effector rotation.

        The second return value reports whether the ZYX wrist decomposition used
        its singular fallback. The exact result is intentionally not clipped.
        """
        q123 = np.asarray(q123_rad, dtype=np.float64)
        if q123.shape != (3,) or not np.all(np.isfinite(q123)):
            raise ValueError("q123_rad must be a finite three-vector")
        target = self._rotation_matrix(target_rotation, "target_rotation")
        previous_q4 = float(previous_q4_rad)
        if not np.isfinite(previous_q4):
            raise ValueError("previous_q4_rad must be finite")

        normalized = (
            self.wrist_base_rotation(q123).T
            @ target
            @ self._wrist_frame_constant.T
        )
        sine_q5 = float(np.clip(normalized[2, 0], -1.0, 1.0))
        q5 = -float(np.arcsin(sine_q5))
        cosine_q5 = float(np.cos(q5))
        singular = abs(cosine_q5) < 1e-6
        if not singular:
            q4 = float(np.arctan2(normalized[1, 0], normalized[0, 0]))
            q6 = float(np.arctan2(normalized[2, 1], normalized[2, 2]))
        else:
            # At q5=+/-pi/2 only q4 +/- q6 is observable. The controller
            # supplies the preceding q4 so the fallback does not jump.
            q4 = previous_q4
            if q5 >= 0.0:
                q6 = q4 + float(np.arctan2(normalized[0, 1], normalized[1, 1]))
            else:
                q6 = float(np.arctan2(-normalized[0, 1], normalized[1, 1])) - q4
            # If q6 is later allowed beyond +/-pi, unwrap it relative to the
            # previous q6 before applying joint limits.
        result = np.asarray((q4, q5, q6), dtype=np.float64)
        if not np.all(np.isfinite(result)):
            raise ValueError("closed-form wrist solution is non-finite")
        return result, singular

    def solve_position(
        self,
        target_position: object,
        q_init_rad: object,
        *,
        max_iterations: int = 50,
        tolerance_m: float = 5e-4,
        step_size: float = 0.5,
        damping: float = 1e-4,
        control_mode: str = "position",
        active_joint_indices: tuple[int, ...] = (0, 1, 2),
    ) -> tuple[np.ndarray, bool, float]:
        """Solve the joint4-origin XYZ using joint1-3.

        q4-q6 are preserved in the returned vector but do not affect this
        position objective.
        """
        target = np.asarray(target_position, dtype=float)
        if target.shape != (3,) or not np.all(np.isfinite(target)):
            raise ValueError("target_position must be a finite three-vector")
        q = np.clip(
            self._joint_vector(q_init_rad),
            self.lower_position_limit,
            self.upper_position_limit,
        )
        if control_mode != "position":
            raise ValueError("B601Kinematics supports position-only IK")
        if tuple(active_joint_indices) != (0, 1, 2):
            raise ValueError("active_joint_indices must be exactly (0, 1, 2)")
        if int(max_iterations) <= 0 or tolerance_m <= 0.0 or step_size <= 0.0 or damping < 0.0:
            raise ValueError("invalid IK solver parameters")

        error, jacobian = self._position_error_and_jacobian(q, target)
        error_norm = float(np.linalg.norm(error))
        for _ in range(int(max_iterations)):
            if error_norm <= tolerance_m:
                return q.copy(), True, error_norm
            active_jacobian = jacobian[:, POSITION_JOINT_INDICES]
            adaptive_damping = float(damping) * max(1.0, error_norm * 10.0)
            normal = active_jacobian @ active_jacobian.T
            normal.flat[:: normal.shape[0] + 1] += adaptive_damping
            try:
                delta = float(step_size) * (
                    active_jacobian.T @ np.linalg.solve(normal, error)
                )
            except np.linalg.LinAlgError:
                break
            max_delta = float(np.max(np.abs(delta)))
            if max_delta > 0.2:
                delta *= 0.2 / max_delta

            improved = False
            alpha = 1.0
            for _ in range(6):
                candidate = q.copy()
                candidate[POSITION_JOINT_INDICES] += alpha * delta
                candidate = np.clip(
                    candidate,
                    self.lower_position_limit,
                    self.upper_position_limit,
                )
                candidate_error, candidate_jacobian = self._position_error_and_jacobian(
                    candidate, target
                )
                candidate_norm = float(np.linalg.norm(candidate_error))
                if candidate_norm < error_norm:
                    q = candidate
                    error = candidate_error
                    jacobian = candidate_jacobian
                    error_norm = candidate_norm
                    improved = True
                    break
                alpha *= 0.5
            if not improved:
                break
        return q.copy(), error_norm <= tolerance_m, error_norm

    def _position_error_and_jacobian(
        self, q: np.ndarray, target: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray]:
        data = self._thread_data()
        self.pin.forwardKinematics(self.model, data, q)
        actual = np.asarray(
            data.oMi[self.wrist_joint_id].translation, dtype=float
        )
        self.pin.computeJointJacobians(self.model, data, q)
        jacobian = self.pin.getJointJacobian(
            self.model,
            data,
            self.wrist_joint_id,
            self.pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
        )[:3, :NUM_ARM_JOINTS]
        return target - actual, np.asarray(jacobian, dtype=float).copy()

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


__all__ = ["B601Kinematics", "NUM_ARM_JOINTS", "POSITION_JOINT_INDICES", "default_urdf_path"]

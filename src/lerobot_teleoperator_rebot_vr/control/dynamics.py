"""Pinocchio dynamics helpers for B601-DM MIT feedforward."""

from __future__ import annotations

from pathlib import Path

import numpy as np


ARM_JOINT_MODEL_NAMES = tuple(f"joint{index}" for index in range(1, 7))


def default_dynamics_urdf_path() -> Path:
    return Path(__file__).resolve().parents[1] / "assets" / "rebot_b601_dm_dynamics.urdf"


class B601GravityCompensator:
    """Compute six-axis generalized gravity torque from the fixed-end model."""

    def __init__(self, urdf_path: str | Path | None = None) -> None:
        try:
            import pinocchio as pin
        except ImportError as exc:
            raise ImportError("MIT gravity feedforward requires Pinocchio") from exc

        self._pin = pin
        self.urdf_path = (
            default_dynamics_urdf_path()
            if urdf_path is None
            else Path(urdf_path).expanduser().resolve()
        )
        if not self.urdf_path.is_file():
            raise FileNotFoundError(f"MIT dynamics URDF not found: {self.urdf_path}")
        self.model = pin.buildModelFromUrdf(str(self.urdf_path))
        model_joint_names = tuple(str(name) for name in self.model.names[1:])
        if self.model.nq != 6 or self.model.nv != 6:
            raise ValueError(
                "MIT dynamics URDF must contain exactly six movable joints; "
                f"got nq={self.model.nq}, nv={self.model.nv}"
            )
        if model_joint_names != ARM_JOINT_MODEL_NAMES:
            raise ValueError(
                "MIT dynamics joint order must be joint1..joint6; "
                f"got {model_joint_names}"
            )
        self.data = self.model.createData()

    def gravity_torque(self, q_rad: np.ndarray) -> np.ndarray:
        q = np.asarray(q_rad, dtype=np.float64)
        if q.shape != (6,) or not np.all(np.isfinite(q)):
            raise ValueError("q_rad must contain six finite joint positions")
        torque = np.asarray(
            self._pin.computeGeneralizedGravity(self.model, self.data, q),
            dtype=np.float64,
        ).reshape(-1)
        if torque.shape != (6,) or not np.all(np.isfinite(torque)):
            raise RuntimeError("Pinocchio returned invalid gravity torque")
        return torque.copy()

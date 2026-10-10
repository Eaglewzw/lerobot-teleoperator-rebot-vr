"""Native joint conventions and immutable hardware ceilings for B601 variants."""

from dataclasses import dataclass
from pathlib import Path

import numpy as np

from ..constants import ARM_EFFORT_LIMIT_NM, FOLLOWER_LOWER_DEG, FOLLOWER_UPPER_DEG


@dataclass(frozen=True)
class ArmProfile:
    name: str
    lower_rad: tuple[float, ...]
    upper_rad: tuple[float, ...]
    effort_nm: tuple[float, ...]
    kinematics_filename: str
    dynamics_filename: str
    feedback_tolerance_deg: float = 0.0

    def model_path(self, *, dynamics=False) -> Path:
        filename = self.dynamics_filename if dynamics else self.kinematics_filename
        return Path(__file__).resolve().parents[1] / "urdf" / filename


DM = ArmProfile(
    "b601_dm", tuple(np.deg2rad(FOLLOWER_LOWER_DEG)),
    tuple(np.deg2rad(FOLLOWER_UPPER_DEG)), ARM_EFFORT_LIMIT_NM,
    "rebot_b601_dm_kinematics.urdf", "rebot_b601_dm_dynamics.urdf",
)
RS = ArmProfile(
    "b601_rs", (-2.8, 0., 0., -1.57, -1.57, -3.14),
    (2.8, 3.14, 3.14, 1.57, 1.57, 3.14), (36., 36., 36., 14., 14., 14.),
    "rebot_b601_rs.urdf", "rebot_b601_rs.urdf",
    feedback_tolerance_deg=0.1,
)


def arm_profile(name: str = "b601_dm") -> ArmProfile:
    try:
        return {DM.name: DM, RS.name: RS}[name]
    except KeyError as exc:
        raise ValueError(f"unsupported robot model: {name}") from exc

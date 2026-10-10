"""Configuration registered with LeRobot's standard robot factory."""

from dataclasses import dataclass

from lerobot.robots.config import RobotConfig

from .rs import RSFollowerConfig
from .rs_calibration import DEFAULT_ROBOT_ID, ROBOT_TYPE


@RobotConfig.register_subclass(ROBOT_TYPE)
@dataclass
class RebotB601RSFollowerConfig(RobotConfig, RSFollowerConfig):
    id: str = DEFAULT_ROBOT_ID
    calibrate_gripper: bool = True

    def __post_init__(self):
        super().__post_init__()
        if self.calibration_dir is not None:
            self.calibration_dir = self.calibration_dir.expanduser()
        if self.motor_can_ids != RSFollowerConfig().motor_can_ids:
            raise ValueError("RS calibration uses fixed motor IDs 1–7 and host ID 0xFD")
        if type(self.calibrate_gripper) is not bool:
            raise ValueError("calibrate_gripper must be boolean")

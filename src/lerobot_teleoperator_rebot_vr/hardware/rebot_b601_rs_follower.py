"""LeRobot calibration adapter for RS; motion uses the VR MIT dispatcher."""

from datetime import datetime
import json
import logging

from lerobot.robots.robot import Robot

from ..constants import JOINT_NAMES
from .config_rebot_b601_rs_follower import RebotB601RSFollowerConfig
from .rs import RSFollower
from .rs_calibration import CalibrationRecorder, ROBOT_TYPE, calibrate


logger = logging.getLogger(__name__)


class RebotB601RSFollower(RSFollower, Robot):
    config_class = RebotB601RSFollowerConfig
    name = ROBOT_TYPE

    def __init__(self, config, *, controller_factory=None):
        RSFollower.__init__(self, config, controller_factory=controller_factory)
        Robot.__init__(self, config)

    @property
    def observation_features(self):
        return {f"{name}.pos": float for name in JOINT_NAMES}

    @property
    def action_features(self):
        return self.observation_features

    def _load_calibration(self, fpath=None):
        # RS records include model identity and checked write/readback state.
        # They do not claim to measure unknown gripper travel.
        if fpath is not None and fpath != self.calibration_store.path:
            raise ValueError("RS calibration must use the configured robot.id and calibration_dir")
        self.calibration = self.calibration_store.load()

    def connect(self, calibrate=True):
        # lerobot-calibrate calls connect(calibrate=False) BEFORE prompting.
        # Opening this adapter never enables or configures motor modes.
        self.open()
        self._connected = True
        try:
            if calibrate and not self.is_calibrated:
                self.calibrate()
        except BaseException:
            self.disconnect()
            raise

    def configure(self):
        raise RuntimeError("RS motion is configured by rebot-vr-teleoperate --robot-model b601_rs")

    def calibrate(self):
        if not self.is_connected:
            raise RuntimeError("Connect the RS transport before calibration")
        if self.calibration_store.load():
            answer = input(
                f"Press ENTER to use provided calibration file associated with the id {self.id}, "
                "or type 'c' and press ENTER to run calibration: "
            )
            if answer.strip().lower() != "c":
                self._load_calibration()
                logger.info(f"Using calibration file associated with the id {self.id}")
                return
        logger.info(f"\nRunning calibration of {self}")
        print("\nCalibration: set zero position.")
        if self.config.calibrate_gripper:
            print("Manually move the reBot B601 to its ZERO POSITION and close the gripper.")
        else:
            print("Manually move the reBot B601 to its ZERO POSITION. Gripper zero will be preserved.")
        print("See the B601 manual for the zero pose (the default sit-down position).\n")
        input("Press ENTER when ready...")
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        log_path = self.calibration_dir / f"{self.id}_{stamp}.jsonl"
        print(f"Calibration log: {log_path}", flush=True)
        with log_path.open("x", encoding="utf-8") as output:
            recorder = CalibrationRecorder(self.calibration_store, output)
            def record(row):
                recorder(row)
                if row["event"] == "zero_readback":
                    print(f"{row['joint']}: {row['position_deg']:+.3f} deg", flush=True)
            try:
                calibrate(self, record, include_gripper=self.config.calibrate_gripper, execute=True)
            except BaseException as exc:
                output.write(json.dumps({"event": "error", "error": str(exc)}) + "\n")
                self._load_calibration()
                raise
        self._load_calibration()
        logger.info("Arm zero position set.")
        print(f"Calibration saved to {self.calibration_fpath}")
        print("Zero readback passed; motors were not enabled. Verify zeros after power cycling.")

    def disconnect(self):
        # Calibration already sends disable explicitly. Closing a transport
        # used only for reads/cancelled calibration must not add motor writes.
        self._connected = False
        self.close()

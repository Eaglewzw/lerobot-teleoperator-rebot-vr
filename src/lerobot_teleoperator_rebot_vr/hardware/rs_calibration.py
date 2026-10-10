"""RS zero writes and durable, robot-ID-specific calibration records."""

from datetime import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import tempfile
import time

from ..constants import JOINT_NAMES
from .profiles import RS


ROBOT_TYPE = "rebot_b601_rs_follower"
DEFAULT_ROBOT_ID = "rebot_b601_rs_vr"


class RSCalibrationStore:
    """A completed write/readback record, not proof of physical alignment.

    The plugin owns this schema: gripper travel is not measured by zeroing,
    so do not invent a MotorCalibration range for that axis.
    """

    def __init__(self, robot_id=DEFAULT_ROBOT_ID, directory=None):
        if not isinstance(robot_id, str) or not robot_id or robot_id in (".", "..") or any(
            char in robot_id for char in ("/", "\\")
        ):
            raise ValueError("RS robot.id must be a nonempty filename without path separators")
        if directory is None:
            from lerobot.utils.constants import HF_LEROBOT_CALIBRATION, ROBOTS
            directory = HF_LEROBOT_CALIBRATION / ROBOTS / ROBOT_TYPE
        self.robot_id = robot_id
        self.path = Path(directory).expanduser() / f"{robot_id}.json"

    def identity(self):
        return {
            "schema_version": 1, "robot_type": ROBOT_TYPE, "robot_id": self.robot_id,
            "model_sha256": hashlib.sha256(RS.model_path().read_bytes()).hexdigest(),
            "motors": {name: {"id": i, "host_id": 0xFD,
                              "model": "rs-06" if i <= 3 else "rs-00"}
                       for i, name in enumerate(JOINT_NAMES, 1)},
        }

    def load(self):
        try:
            data = json.loads(self.path.read_text(encoding="utf-8"))
            if not isinstance(data, dict) or data.get("status") != "complete":
                return {}
            if any(data.get(key) != value for key, value in self.identity().items()):
                return {}
            selected = data.get("zeroed_joints")
            if selected not in (list(JOINT_NAMES[:6]), list(JOINT_NAMES)):
                return {}
            positions = data["positions_deg"]
            if not isinstance(positions, dict) or set(positions) != set(JOINT_NAMES):
                return {}
            if any(type(v) not in (int, float) or not math.isfinite(v) for v in positions.values()):
                return {}
            if any(abs(positions[name]) > .5 for name in selected):
                return {}
            return data
        except (OSError, ValueError, TypeError, KeyError):
            return {}

    def _write(self, data):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        fd, temporary = tempfile.mkstemp(prefix=f".{self.robot_id}.", dir=self.path.parent)
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as output:
                json.dump(data, output, indent=2, allow_nan=False)
                output.write("\n")
                output.flush()
                os.fsync(output.fileno())
            os.replace(temporary, self.path)
            directory_fd = os.open(self.path.parent, os.O_RDONLY | os.O_DIRECTORY)
            try:
                os.fsync(directory_fd)
            finally:
                os.close(directory_fd)
        finally:
            if os.path.exists(temporary):
                os.unlink(temporary)

    def invalidate(self):
        # Invalidate BEFORE the first zero command. Even interruption or a
        # lost ACK must not leave a previous success record usable by VR.
        previous = self.path.read_text(encoding="utf-8") if self.path.exists() else None
        self._write({**self.identity(), "status": "incomplete",
                     "previous_record_text": previous,
                     "started_at": datetime.now().isoformat()})

    def save(self, positions, selected):
        self._write({**self.identity(), "status": "complete",
                     "completed_at": datetime.now().isoformat(),
                     "zeroed_joints": list(selected), "positions_deg": positions,
                     "gripper_closed_deg": 0.0 if "gripper" in selected else None,
                     "gripper_open_deg": None})


class CalibrationRecorder:
    """Flush audit events, invalidate old records, then commit verified zeros."""

    def __init__(self, store, output):
        self.store, self.output = store, output
        self.started = False
        self.after = None

    def __call__(self, row):
        self.output.write(json.dumps({"host_time_ns": time.time_ns(), **row}) + "\n")
        self.output.flush()
        if row["event"] == "zero_attempt" and not self.started:
            self.store.invalidate()
            self.started = True
        elif row["event"] == "after":
            self.after = row["positions_deg"]
        elif row["event"] == "complete":
            self.store.save(self.after, row["zeroed_joints"])


def calibrate(connection, record, *, include_gripper=False, execute=False):
    """Caller confirms the physical pose; this function never enables motors.

    Record every completed write immediately: a failure can leave only some
    axes zeroed, and there is no automatic rollback of encoder calibration.
    """
    selected = JOINT_NAMES if include_gripper else JOINT_NAMES[:6]

    def snapshot(stage):
        results = connection.read_positions(100)
        positions = {}
        for name in JOINT_NAMES:
            result = results[name]
            if isinstance(result, Exception):
                raise RuntimeError(f"{stage}: {name} feedback failed: {result}") from result
            value = result[0]
            if not math.isfinite(value):
                raise RuntimeError(f"{stage}: {name} feedback is not finite")
            positions[name] = math.degrees(value)
        record({"event": stage, "positions_deg": positions})
        return positions

    before = snapshot("before")
    if not execute:
        return
    # Disable every axis, including an unselected gripper. A failed disable
    # aborts ALL zero writes; still attempt to disable the remaining motors.
    errors = []
    for name in JOINT_NAMES:
        try:
            connection.motors[name].disable()
            record({"event": "disable_sent", "joint": name})
        except Exception as exc:
            errors.append(f"{name}: {exc}")
    if errors:
        raise RuntimeError(f"Disable failed; no zeros written: {errors}")
    time.sleep(.3)
    settled = snapshot("after_disable")
    if any(abs(settled[name] - before[name]) > .5 for name in JOINT_NAMES):
        raise RuntimeError("Pose moved after disable; support and realign the arm before retrying")
    for name in selected:
        record({"event": "zero_attempt", "joint": name})
        connection.motors[name].set_zero_position()
        record({"event": "zero_acknowledged", "joint": name})
        time.sleep(.1)
        value, _, _ = connection.read_position(name, 100)
        deg = math.degrees(value)
        record({"event": "zero_readback", "joint": name, "position_deg": deg})
        if not math.isfinite(deg) or abs(deg) > .5:
            raise RuntimeError(f"{name}: zero readback {deg:.3f} deg outside +/-0.5 deg")
    after = snapshot("after")
    if any(abs(after[name]) > .5 for name in selected):
        raise RuntimeError("Final zero readback failed; inspect the log and physical alignment")
    record({"event": "complete", "zeroed_joints": list(selected)})

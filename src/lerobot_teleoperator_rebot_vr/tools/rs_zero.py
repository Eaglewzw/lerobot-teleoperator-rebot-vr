"""Set RS encoder zeros at a manually aligned pose, without enabling motors."""

import argparse
from datetime import datetime
import json
from pathlib import Path
import time

from ..hardware.rs import RSConnection
from ..hardware.rs_calibration import (
    CalibrationRecorder, DEFAULT_ROBOT_ID, RSCalibrationStore, calibrate,
)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="can0")
    parser.add_argument("--robot-id", default=DEFAULT_ROBOT_ID)
    parser.add_argument("--calibration-dir", type=Path)
    parser.add_argument("--execute", action="store_true", help="confirm interactively and write zeros")
    parser.add_argument("--include-gripper", action="store_true",
                        help="also define the fully closed gripper position as zero")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.execute:
        print("Stop other motor programs. Support the arm: this operation disables all seven motors.")
        print("Align all six joints to the official RS zero pose BEFORE continuing.")
        if args.include_gripper:
            print("The gripper must also be fully closed, without forcing the mechanical stop.")
        print("This overwrites encoder zeros at the CURRENT pose; it does not move the arm to zero.")
        try:
            answer = input("Confirm physical alignment and support by typing ZERO: ")
        except (EOFError, KeyboardInterrupt):
            raise SystemExit("Cancelled; no hardware opened")
        if answer.strip() != "ZERO":
            raise SystemExit("Cancelled; no hardware opened")
    path = args.output or Path("logs/rs_zero") / (
        datetime.now().strftime("%Y%m%d_%H%M%S_%f") + ".jsonl")
    path.parent.mkdir(parents=True, exist_ok=True)
    connection = RSConnection(args.port)
    store = RSCalibrationStore(args.robot_id, args.calibration_dir)
    with path.open("x", encoding="utf-8") as output:
        recorder = CalibrationRecorder(store, output)
        def record(row):
            row = {"host_time_ns": time.time_ns(), **row}
            recorder(row)
            print(json.dumps(row, ensure_ascii=False), flush=True)
        print(f"Log: {path.resolve()}", flush=True)
        record({"event": "start", "port": args.port, "execute": args.execute,
                "include_gripper": args.include_gripper})
        try:
            connection.open()
            calibrate(connection, record, include_gripper=args.include_gripper,
                      execute=args.execute)
        except BaseException as exc:
            record({"event": "error", "error": str(exc),
                    "note": "If writes began, calibration may be partial. No automatic rollback."})
            raise
        finally:
            connection.close()
    if args.execute:
        print(f"Calibration saved to {store.path}")
        print("Zero readback passed. No enable or motion commands sent.")
        print("Verify physical alignment and recheck readings after a supported power cycle.")
    else:
        print("Read-only preview complete. Use --execute to write zeros after physical alignment.")


if __name__ == "__main__":
    main()

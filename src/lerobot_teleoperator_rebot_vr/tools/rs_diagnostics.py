"""Read RS elbow/wrist parameters without changing motor state or targets.

Register names/types follow MotorBridge robstride/registers.rs. Firmware may
not support every parameter; preserve errors rather than inventing zero values.
Current/voltage are snapshots at read time, not measurements of an earlier run.
Position/velocity-loop gains and limits need not apply to native MIT mode.
"""

import argparse
from datetime import datetime
import json
import math
from pathlib import Path
import time

from ..constants import JOINT_NAMES
from ..hardware.rs import RSConnection


# Read-only whitelist, even for registers that the firmware also allows writing.
FLOAT_REGISTERS = (
    (0x7019, "position_rad"), (0x701B, "velocity_rad_s"),
    (0x701A, "iq_filtered_a"), (0x701C, "bus_voltage_v"),
    (0x700B, "torque_limit_nm"), (0x7018, "position_velocity_current_limit_a"),
    (0x7017, "position_speed_limit_rad_s"), (0x7010, "current_loop_kp"),
    (0x7011, "current_loop_ki"), (0x7014, "current_filter_gain"),
    (0x701E, "position_loop_kp"), (0x701F, "velocity_loop_kp"),
    (0x7020, "velocity_loop_ki"),
)


def read_motor_diagnostics(motor, *, port, joint, sample, timeout_ms):
    row = {"port": port, "joint": joint, "sample": sample,
           "motor_id": JOINT_NAMES.index(joint) + 1,
           "host_time_ns": time.time_ns(), "read_started_ns": time.monotonic_ns(),
           "errors": {}}
    for register, name in FLOAT_REGISTERS:
        try:
            value = float(motor.robstride_get_param_f32_host_id(register, 0xFD, timeout_ms))
            if not math.isfinite(value):
                raise ValueError("non-finite register value")
            row[name] = value
        except Exception as exc:
            row["errors"][name] = str(exc)
    try:
        # Wire run_mode uses 0=MIT (different from MotorBridge Mode.MIT enum).
        row["run_mode_raw"] = motor.robstride_get_param_i8(0x7005, timeout_ms)
    except Exception as exc:
        row["errors"]["run_mode_raw"] = str(exc)
    row["read_finished_ns"] = time.monotonic_ns()
    if "position_rad" in row:
        row["position_deg"] = math.degrees(row["position_rad"])
    return row


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ports", nargs="+", default=["can0", "can1"])
    parser.add_argument("--joints", nargs="+", choices=JOINT_NAMES,
                        default=["elbow_flex", "wrist_flex"])
    parser.add_argument("--samples", type=int, default=3)
    parser.add_argument("--timeout-ms", type=int, default=100)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if not 1 <= args.samples <= 100 or not 1 <= args.timeout_ms <= 1000:
        parser.error("samples must be 1..100 and timeout-ms 1..1000")
    if len(set(args.ports)) != len(args.ports):
        parser.error("ports must be distinct")
    path = args.output or Path("logs/rs_diagnostics") / (
        datetime.now().strftime("%Y%m%d_%H%M%S_%f") + ".jsonl")
    path.parent.mkdir(parents=True, exist_ok=True)
    failed = False
    with path.open("x", encoding="utf-8") as output:
        print(f"Read-only RS parameter snapshots; log: {path.resolve()}", flush=True)
        for port in args.ports:
            connection = RSConnection(port)
            try:
                connection.open()
                for sample in range(args.samples):
                    for joint in args.joints:
                        row = read_motor_diagnostics(connection.motors[joint], port=port,
                            joint=joint, sample=sample, timeout_ms=args.timeout_ms)
                        failed |= bool(row["errors"])
                        output.write(json.dumps(row, allow_nan=False) + "\n")
                        output.flush()
                        print(f"{port} {joint} sample={sample}: " + json.dumps({
                            key: row.get(key) for key in ("position_deg", "run_mode_raw",
                                "iq_filtered_a", "bus_voltage_v", "torque_limit_nm",
                                "position_velocity_current_limit_a", "errors")}), flush=True)
            except Exception as exc:
                failed = True
                output.write(json.dumps({"port": port, "error": str(exc)}) + "\n")
                output.flush()
                print(f"{port}: ERROR {exc}", flush=True)
            finally:
                connection.close()
    if failed:
        raise SystemExit(1)


if __name__ == "__main__":
    main()

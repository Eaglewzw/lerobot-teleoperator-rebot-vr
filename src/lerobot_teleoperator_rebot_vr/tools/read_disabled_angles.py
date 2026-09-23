"""Read B601-DM cached angles without enable, mode, motion or zero commands.

Feedback requests transmit query frames: this is not a passive CAN sniffer.
The SDK has no receive timestamp, so a disabled cached status is not a safety
confirmation. Physically support the arm; do not move it during powered reads.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
import time
import uuid

from ..constants import JOINT_NAMES
from ..runtime.dual_config import load_dual_config


MOTORS = tuple((name, i, i + 0x10, "4340P" if i <= 3 else "4310")
               for i, name in enumerate(JOINT_NAMES, 1))
FIELDS = ("timestamp_ns", "cache_read_monotonic_ns", "joint", "can_id",
          "arbitration_id", "status_code", "position_deg", "velocity_rad_s",
          "reported_torque_nm", "mos_temperature_c", "rotor_temperature_c",
          "feedback_freshness")


def validate_limits(duration_s, rate_hz):
    if not math.isfinite(duration_s) or not 0.5 <= duration_s <= 30:
        raise ValueError("duration must be finite and in [0.5, 30] seconds")
    if not math.isfinite(rate_hz) or not 1 <= rate_hz <= 20:
        raise ValueError("rate must be finite and in [1, 20] Hz")


def read_disabled_angles(open_controller, write_row, report, *, duration_s=3., rate_hz=10.,
                         clock=time.monotonic, sleep=time.sleep):
    """Only register handles, request/poll/read feedback, and close transport.

    Deliberately no Controller context manager: its __exit__ calls shutdown(),
    which sends disable commands. An unexpected state aborts without changing it.
    """
    validate_limits(duration_s, rate_hz)
    report.update(result="incomplete", feedback_freshness="unknown_no_receive_timestamp",
                  torque_off_confirmed=False, samples=0, cleanup_errors=[], joints={})
    controller = None
    motors = {}
    try:
        controller = open_controller()
        for name, motor_id, feedback_id, model in MOTORS:
            motors[name] = controller.add_damiao_motor(motor_id, feedback_id, model)
        started = clock()
        while clock() - started < duration_s:
            cycle = clock()
            for motor in motors.values():
                motor.request_feedback()
            sleep(.02)
            controller.poll_feedback_once()
            for name, motor_id, feedback_id, _ in MOTORS:
                state = motors[name].get_state()
                if state is None:
                    continue  # Never replace unavailable feedback with zero.
                row = dict(zip(FIELDS, (
                    time.time_ns(), time.monotonic_ns(), name, state.can_id,
                    state.arbitration_id, state.status_code, math.degrees(state.pos),
                    state.vel, state.torq, state.t_mos, state.t_rotor,
                    "unknown_no_receive_timestamp",
                ), strict=True))
                write_row(row)  # Preserve the unexpected sample before aborting.
                report["samples"] += 1
                if state.can_id != motor_id or state.arbitration_id != feedback_id:
                    raise RuntimeError(f"{name}: unexpected feedback IDs; inspect recorded sample")
                if state.status_code != 0:
                    raise RuntimeError(f"{name}: cached status {state.status_code}, expected disabled (0); "
                                       "aborting without sending disable or other control commands")
                if not all(math.isfinite(value) for value in
                           (state.pos, state.vel, state.torq, state.t_mos, state.t_rotor)):
                    raise RuntimeError(f"{name}: non-finite feedback")
                angle = row["position_deg"]
                summary = report["joints"].setdefault(name, {
                    "samples": 0, "first_deg": angle, "last_deg": angle,
                    "min_deg": angle, "max_deg": angle, "cached_status_code": 0,
                })
                summary.update(samples=summary["samples"] + 1, last_deg=angle,
                               min_deg=min(summary["min_deg"], angle),
                               max_deg=max(summary["max_deg"], angle))
            sleep(max(0., 1. / rate_hz - (clock() - cycle)))
        missing = [name for name in motors if name not in report["joints"]]
        if missing:
            raise RuntimeError(f"no feedback for: {', '.join(missing)}; check power/wiring, do not enable motors")
        report["result"] = "read_complete_freshness_unverified"
    except BaseException as exc:
        report.update(result="failed", error=str(exc) or type(exc).__name__)
        raise
    finally:
        # Close the bus first. Never call SDK shutdown() or follower.disconnect().
        cleanup = []
        if controller is not None:
            cleanup.append(("close_bus", controller.close_bus))
        cleanup.extend((f"close_{name}", motor.close) for name, motor in motors.items())
        if controller is not None:
            cleanup.append(("free_controller", controller.close))
        for name, close in cleanup:
            try:
                close()
            except Exception as exc:
                report["cleanup_errors"].append(f"{name}: {exc}")
        if report["cleanup_errors"] and report["result"] != "failed":
            report["result"] = "failed"
            raise RuntimeError("read-only probe cleanup failed: " + "; ".join(report["cleanup_errors"]))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--arm", choices=("left", "right"), default="right")
    parser.add_argument("--duration", type=float, default=3.)
    parser.add_argument("--rate", type=float, default=10.)
    parser.add_argument("--pose-label", choices=("unknown", "failed-return", "reference-zero"), default="unknown",
                        help="operator-supplied description, not an independently verified zero")
    parser.add_argument("--output-dir", type=Path, default=Path("logs/zero_check"))
    parser.add_argument("--confirm-supported", action="store_true",
                        help="confirm physical support, no other controller, and no powered manual movement")
    parser.add_argument("--dry-run", action="store_true", help="validate only; no serial connection")
    args = parser.parse_args(argv)
    try:
        validate_limits(args.duration, args.rate)
        arm = load_dual_config(args.config).arms[args.arm]
        if arm.can_adapter != "damiao":
            raise ValueError("this read-only probe only supports B601-DM via damiao serial")
    except (OSError, ValueError, TypeError) as exc:
        parser.error(str(exc))
    report = {"arm": args.arm, "robot_id": arm.robot_id, "port": arm.robot_port,
              "baud": arm.dm_serial_baud, "pose_label_operator_supplied": args.pose_label,
              "duration_s": args.duration, "rate_hz": args.rate,
              "zero_persistence_verified": False,
              "note": "one snapshot cannot prove zero persistence across power cycles"}
    print(json.dumps(report, indent=2), flush=True)
    if args.dry_run:
        return
    if not args.confirm_supported:
        parser.error("physically support the arm and stop all other controllers; then use --confirm-supported")
    # Importing the SDK and opening CAN happen only after all validation.
    from motorbridge import Controller

    output = args.output_dir / uuid.uuid4().hex[:12]
    output.mkdir(parents=True, exist_ok=False)
    print(f"READ-ONLY QUERY: no enable/mode/zero/motion/disable commands. Output: {output}", flush=True)
    print("A disabled cached status is NOT a fresh torque-off confirmation. Keep the arm supported.", flush=True)
    try:
        with (output / f"{args.arm}.csv").open("x", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=FIELDS)
            writer.writeheader()

            def write(row):
                writer.writerow(row)
                stream.flush()

            read_disabled_angles(
                lambda: Controller.from_dm_serial(arm.robot_port, arm.dm_serial_baud),
                write, report, duration_s=args.duration, rate_hz=args.rate,
            )
    finally:
        (output / "report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        print(json.dumps(report, indent=2), flush=True)


if __name__ == "__main__":
    main()

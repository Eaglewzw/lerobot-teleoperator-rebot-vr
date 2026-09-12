"""Compare base-ID and legacy mode-ID torque-off commands without enabling motors.

Run only after the teleoperation process has exited. No configuration registers,
zero positions, enable frames, or position/velocity targets are written.
The DM serial framing matches MotorBridge 0.3.9 motor_core/src/dm_serial.rs.
"""

from __future__ import annotations

import argparse
import time
from dataclasses import dataclass


JOINTS = (
    "shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex",
    "wrist_yaw", "wrist_roll", "gripper",
)
MODE_OFFSETS = {"mit": 0, "pos_vel": 0x100, "force_pos": 0x300}
DISABLE_PAYLOAD = b"\xff" * 7 + b"\xfd"
STATUS_NAMES = {
    0: "DISABLED", 1: "ENABLED", 8: "OVER_VOLTAGE", 9: "UNDER_VOLTAGE",
    10: "OVER_CURRENT", 11: "MOS_OVER_TEMP", 12: "ROTOR_OVER_TEMP",
    13: "LOST_COMM", 14: "OVERLOAD",
}


@dataclass(frozen=True)
class DisableTarget:
    name: str
    motor_id: int
    feedback_id: int
    mode: str

    @property
    def mode_can_id(self) -> int:
        return self.motor_id + MODE_OFFSETS[self.mode]


def disable_frame(can_id: int) -> bytes:
    """Encode one standard CAN torque-off frame for the DM serial bridge."""
    if not 1 <= can_id <= 0x7FF:
        raise ValueError("disable CAN ID must be in 1..0x7FF")
    frame = bytearray(30)
    frame[:4] = b"\x55\xaa\x1e\x03"
    frame[4:8] = (1).to_bytes(4, "little")
    frame[8:12] = (10).to_bytes(4, "little")
    frame[13:17] = can_id.to_bytes(4, "little")
    frame[18] = 8
    frame[21:29] = DISABLE_PAYLOAD
    return bytes(frame)


def pop_feedback(buffer: bytearray) -> list[tuple[int, int, int]]:
    """Extract (arbitration ID, motor ID nibble, status) from normal RX frames."""
    result = []
    while buffer:
        if buffer[0] != 0xAA:
            del buffer[0]
            continue
        if len(buffer) < 16:
            break
        # Only eight-byte, standard data frames; reject register replies below.
        if buffer[1] != 0x11 or buffer[2] != 8 or buffer[15] != 0x55:
            del buffer[0]
            continue
        raw = bytes(buffer[:16])
        del buffer[:16]
        # Register replies encode the motor ID in two bytes, followed by the
        # operation byte. Reject them conservatively even on a feedback CAN ID.
        if raw[8] <= 0x0F and raw[9] in (0x33, 0x55):
            continue
        result.append((int.from_bytes(raw[3:7], "little"), raw[7] & 0x0F, raw[7] >> 4))
    return result


def send_disable_and_read(port, target: DisableTarget, can_id: int, timeout_s: float) -> int | None:
    """Read a matching raw status after sending; never consult an SDK cache."""
    port.reset_input_buffer()
    frame = disable_frame(can_id)
    if port.write(frame) != len(frame):
        raise OSError("incomplete serial write of disable frame")
    port.flush()
    deadline = time.monotonic() + timeout_s
    buffer = bytearray()
    last_status = None
    while time.monotonic() < deadline:
        buffer.extend(port.read(max(1, min(port.in_waiting, 4096))))
        for feedback_id, motor_id, status in pop_feedback(buffer):
            if feedback_id == target.feedback_id and motor_id == target.motor_id:
                last_status = status
                if status == 0:
                    return status
    return last_status


def probe(port, targets: list[DisableTarget], *, attempts: int = 2, timeout_s: float = 0.2) -> bool:
    """Try base addressing first, then configured-mode addressing when needed."""
    all_disabled = True
    for target in targets:
        disabled = False
        addresses = [("base", target.motor_id)]
        if target.mode_can_id != target.motor_id:
            addresses.append(("mode", target.mode_can_id))
        for phase, can_id in addresses:
            for attempt in range(1, attempts + 1):
                try:
                    status = send_disable_and_read(port, target, can_id, timeout_s)
                    label = "NO_REPLY" if status is None else STATUS_NAMES.get(status, f"UNKNOWN({status})")
                except Exception as exc:
                    status = None
                    label = f"IO_ERROR({exc})"
                print(
                    f"{target.name} ID={target.motor_id} {phase}_can_id=0x{can_id:03X} "
                    f"attempt={attempt}/{attempts} raw_status={label}",
                    flush=True,
                )
                time.sleep(0.02)
                if status == 0:
                    disabled = True
                    break
            if disabled:
                break
        all_disabled = all_disabled and disabled
    return all_disabled


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-port", default="/dev/ttyACM1")
    parser.add_argument("--baud", type=int, default=921600)
    parser.add_argument("--arm-mode", choices=("mit", "pos_vel"), default="pos_vel")
    parser.add_argument("--gripper-mode", choices=("mit", "force_pos"), default="force_pos")
    parser.add_argument("--send", action="store_true", help="Send torque-off frames; otherwise only print the plan")
    args = parser.parse_args()
    targets = [
        DisableTarget(name, index, 0x10 + index, args.gripper_mode if index == 7 else args.arm_mode)
        for index, name in enumerate(JOINTS, 1)
    ]
    for target in targets:
        print(
            f"{target.name}: base=0x{target.motor_id:03X}, "
            f"mode=0x{target.mode_can_id:03X} ({target.mode}), feedback=0x{target.feedback_id:03X}"
        )
    if not args.send:
        print("Plan only. After teleoperation exits and the arm is supported, add --send to disable motors.")
        return
    from serial import Serial

    print("Sending ONLY torque-off frames. The arm must be supported; teleoperation must already be stopped.", flush=True)
    with Serial(args.robot_port, args.baud, timeout=0.01, write_timeout=0.5, exclusive=True) as port:
        success = probe(port, targets)
        port.flush()
    print(
        "All seven motors reported DISABLED in received frames."
        if success else "Some motors did not report DISABLED; retain the per-motor output and check LEDs.",
        flush=True,
    )
    raise SystemExit(0 if success else 1)


if __name__ == "__main__":
    main()

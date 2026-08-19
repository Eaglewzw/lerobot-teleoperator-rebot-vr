"""Print the selected PICO controller sample without connecting the robot."""

from __future__ import annotations

import argparse
import signal
import time

import numpy as np
from scipy.spatial.transform import Rotation

from .config_rebot_vr import RebotVRConfig
from .vr_controller import make_vr_controller


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--backend",
        choices=("isaac", "xrobotoolkit_v1"),
        default="xrobotoolkit_v1",
    )
    parser.add_argument("--hand", choices=("left", "right"), default="right")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=63901)
    parser.add_argument("--rate", type=float, default=10.0, help="print rate in Hz")
    parser.add_argument("--duration", type=float, default=0.0, help="0 runs until Ctrl-C")
    parser.add_argument("--no-cloudxr-launch", action="store_true")
    return parser


def _format_pose(position: object, quaternion: object) -> str:
    if position is None or quaternion is None:
        return "not provided by this backend"
    pos = np.asarray(position, dtype=float)
    quat = np.asarray(quaternion, dtype=float)
    rpy = Rotation.from_quat(quat).as_euler("xyz", degrees=True)
    return (
        f"pos_m={np.array2string(pos, precision=4, separator=',')} "
        f"quat_xyzw={np.array2string(quat, precision=4, separator=',')} "
        f"rpy_deg={np.array2string(rpy, precision=2, separator=',')}"
    )


def main() -> None:
    args = _parser().parse_args()
    if args.rate <= 0.0 or args.duration < 0.0:
        raise ValueError("rate must be positive and duration must be non-negative")
    config = RebotVRConfig(
        vr_backend=args.backend,
        hand_side=args.hand,
        ws_host=args.host,
        ws_port=args.port,
        auto_launch_cloudxr=not args.no_cloudxr_launch,
    )
    controller = make_vr_controller(config)
    stop = False

    def stop_now(_signal_number, _frame) -> None:
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, stop_now)
    signal.signal(signal.SIGTERM, stop_now)

    if args.backend == "xrobotoolkit_v1":
        print(
            f"Listening for XRoboToolkit V1 on {args.host}:{args.port}; "
            "start the PICO Tracking sender."
        )
    else:
        print("Starting Isaac Teleop/CloudXR; connect the PICO 4 web client when ready.")

    controller.connect()
    started = time.monotonic()
    try:
        while not stop:
            if args.duration and time.monotonic() - started >= args.duration:
                break
            action = controller.get_action()
            print(
                f"tracking={bool(action['is_tracking'])} "
                f"grip={float(action['squeeze']):.3f} "
                f"trigger={float(action['trigger']):.3f} "
                f"primary={bool(action.get('primary_button', False))} "
                f"secondary={bool(action.get('secondary_button', False))}"
            )
            print(f"  controller: {_format_pose(action['grip_pos'], action['grip_quat'])}")
            time.sleep(1.0 / args.rate)
    finally:
        controller.disconnect()


if __name__ == "__main__":
    main()

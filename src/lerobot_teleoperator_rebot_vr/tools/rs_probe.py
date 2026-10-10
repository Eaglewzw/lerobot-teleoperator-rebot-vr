"""Read RS mechPos registers without enable, mode, zero, or motion writes."""

import argparse
from datetime import datetime
import json
import math
from pathlib import Path
import time

from ..constants import JOINT_NAMES
from ..hardware.rs import RSConnection


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="can0")
    parser.add_argument("--samples", type=int, default=3)
    parser.add_argument("--interval", type=float, default=.2)
    parser.add_argument("--timeout-ms", type=int, default=100)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if not 1 <= args.samples <= 100 or not 1 <= args.timeout_ms <= 1000:
        parser.error("samples must be 1..100 and timeout-ms 1..1000")
    if not math.isfinite(args.interval) or not 0 <= args.interval <= 10:
        parser.error("interval must be 0..10 seconds")
    path = args.output or Path("logs/rs_probe") / (datetime.now().strftime("%Y%m%d_%H%M%S_%f") + ".jsonl")
    path.parent.mkdir(parents=True, exist_ok=True)
    connection = RSConnection(args.port)
    failed = False
    with path.open("x", encoding="utf-8") as output:
        print(f"Read-only RS probe on {args.port}; log: {path.resolve()}", flush=True)
        try:
            connection.open()
            for sample in range(args.samples):
                batch_started = time.monotonic()
                results = connection.read_positions(args.timeout_ms)
                print(f"Batch {sample}: {(time.monotonic()-batch_started)*1000:.2f} ms", flush=True)
                for name in JOINT_NAMES:
                    row = {"sample": sample, "joint": name, "host_time_ns": time.time_ns()}
                    try:
                        result = results[name]
                        if isinstance(result, Exception):
                            raise result
                        pos, start, end = result
                        row.update(position_rad=pos, position_deg=math.degrees(pos),
                                   read_started_ns=start, read_finished_ns=end)
                        print(f"{sample} {name}: {math.degrees(pos):+.3f} deg ({(end-start)*1e-6:.2f} ms)", flush=True)
                    except Exception as exc:
                        failed = True
                        row["error"] = str(exc)
                        print(f"{sample} {name}: ERROR {exc}", flush=True)
                    output.write(json.dumps(row) + "\n")
                    output.flush()
                if sample + 1 < args.samples:
                    time.sleep(args.interval)
        finally:
            connection.close()
    if failed:
        raise SystemExit(1)


if __name__ == "__main__":
    main()

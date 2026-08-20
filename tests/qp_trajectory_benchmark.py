"""Offline VR-style trajectory injection benchmark for the full-body QP IK."""
from __future__ import annotations

import json
import numpy as np
from lerobot_teleoperator_rebot_vr.kinematics import B601Kinematics, FullBodyQPIKSolver


def main() -> None:
    k = B601Kinematics()
    qp = FullBodyQPIKSolver(k, max_solve_time_ms=20.0)
    report = {}
    for name, values in {
        "past_90": np.linspace(-1.8, 1.8, 80),
        "wide_arc": np.linspace(-2.3, 2.3, 100),
        "circle": np.linspace(0.0, 2.0 * np.pi, 120),
    }.items():
        q = np.array([0.0, -0.8, -0.8, 0.0, 0.0, 0.0])
        dq = np.zeros(6)
        errors, solve_ms, jumps, failures = [], [], [], 0
        for value in values:
            target_q = q.copy()
            target_q[0] = value if name != "circle" else 0.8 * np.sin(value)
            target_q[3:] = [0.4 * np.sin(value), 0.3 * np.cos(value), 0.2 * np.sin(value)]
            p, r = k.forward_kinematics(target_q)
            q_next, ok, _, elapsed, _ = qp.solve(
                target_position=p, target_rotation=r, q_actual=q, dq_previous=dq,
                dt=0.01, q_nominal=q, max_joint_speed=np.full(6, 2.0),
                max_joint_acceleration=np.full(6, 8.0),
            )
            solve_ms.append(elapsed)
            if not ok:
                failures += 1
                q_next = q.copy()
            actual_p, _ = k.forward_kinematics(q_next)
            errors.append(float(np.linalg.norm(actual_p - p)))
            jumps.append(float(np.max(np.abs(q_next - q))))
            dq = (q_next - q) / 0.01
            q = q_next
        report[name] = {
            "tcp_position_error_mean_m": float(np.mean(errors)),
            "qp_failures": failures,
            "max_single_frame_joint_change_rad": max(jumps),
            "solve_time_mean_ms": float(np.mean(solve_ms)),
            "solve_time_p95_ms": float(np.percentile(solve_ms, 95)),
            "solve_time_max_ms": max(solve_ms),
        }
    print(json.dumps(report, indent=2, sort_keys=True))
    k.close()


if __name__ == "__main__":
    main()

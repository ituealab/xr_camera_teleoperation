#!/usr/bin/env python
"""IK solve-time benchmark (SafeG1_29_ArmIK.solve_ik): free-space fast path vs. self-collision-constrained solves.

    data/xr-env/bin/python -m xr_camera_teleop.tools.bench_ik [--trials 40] [--plain]

Random reachable wrist targets (FK of random joint configurations, smooth ramps like a moving operator); every
4th trial pulls the targets into the torso so the constrained problem is exercised. Run it ON THE ROBOT (PC2,
Jetson Orin NX) to get the numbers that matter for the control loop; the laptop numbers are only a reference.
"""
from __future__ import annotations

import argparse
import platform
import time

import numpy as np


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--trials", type=int, default=40)
    ap.add_argument("--steps", type=int, default=12)
    ap.add_argument("--plain", action="store_true", help="benchmark the plain upstream IK (no self-collision)")
    a = ap.parse_args(argv)

    import pinocchio as pin
    from xr_camera_teleop.robot.ik import make_arm_ik

    t0 = time.perf_counter()
    ik = make_arm_ik(self_collision=not a.plain)
    init_s = time.perf_counter() - t0
    m = ik.reduced_robot.model
    lo, hi = m.lowerPositionLimit, m.upperPositionLimit
    rng = np.random.default_rng(0)
    d = m.createData()

    def fk(q):
        pin.framesForwardKinematics(m, d, q)
        return d.oMf[ik.L_hand_id].homogeneous.copy(), d.oMf[ik.R_hand_id].homogeneous.copy()

    free, cons = [], []
    for trial in range(a.trials):
        if hasattr(ik, "reset_tracking"):
            ik.reset_tracking(np.zeros(14))
        q_cur = np.zeros(14)
        qa, qb = 0.5 * rng.uniform(lo, hi), 0.5 * rng.uniform(lo, hi)
        for k in range(a.steps):
            L, R = fk((1 - min(1.0, k / 8)) * qa + min(1.0, k / 8) * qb)
            if trial % 4 == 3:
                L[1, 3] -= 0.1
                R[1, 3] += 0.1
                L[0, 3] *= 0.5
                R[0, 3] *= 0.5
            n0 = getattr(ik, "stats", {}).get("constrained", 0)
            t = time.perf_counter()
            q_cur, _ = ik.solve_ik(L, R, q_cur, np.zeros(14))
            ms = (time.perf_counter() - t) * 1e3
            q_cur = np.asarray(q_cur).copy()
            (cons if getattr(ik, "stats", {}).get("constrained", 0) > n0 else free).append(ms)

    def row(name, x):
        if not x:
            return f"  {name:<28} n=0"
        x = np.asarray(x)
        return (f"  {name:<28} n={len(x):4d}  median {np.median(x):6.2f} ms  p95 {np.percentile(x, 95):6.2f} ms  "
                f"max {x.max():6.2f} ms")

    print(f"host: {platform.node()} {platform.machine()} python {platform.python_version()}  "
          f"pinocchio {pin.__version__}")
    print(f"IK: {type(ik).__name__}, construction {init_s:.1f} s")
    print(row("free space (fast path)", free))
    print(row("constrained (collision)", cons))
    print(row("all solves", free + cons))
    if hasattr(ik, "stats"):
        print("  stats:", {k: v for k, v in ik.stats.items() if k != "last_ms"})


if __name__ == "__main__":
    main()

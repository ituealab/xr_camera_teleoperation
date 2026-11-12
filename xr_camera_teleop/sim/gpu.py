"""The sim machine's GPU: power-budget preflight and sustained clock sampling (nvidia-smi, journalctl).

Why (docs/quest-sonic/P1_sim.md §3a): on the MSI the dGPU once kept a 20 W budget after a failed resume from s2idle.
The kernel logged `NVRM: rm_power_source_change_event: Failed to handle Power Source change event` and
`RmHandleDNotifierEvent: Failed to handle ACPI D-Notifier event`; the GPU stayed at its battery-level limit, and
with the robot cameras on SONIC's policy missed its 20 ms budget and the robot fell. A reboot (or replugging the
charger) restores the budget. `preflight()` detects the state; the sim warns at startup and tools/sim_perf.py gives
no pass/fail verdict in it.

`ClockSampler` records graphics clock, power draw and active clock-event (throttle) reasons once a second, so a
performance run shows what the GPU actually sustained (it boosts to ~2.7 GHz briefly, then settles near 1.6 GHz).
"""
from __future__ import annotations

import os
import subprocess
import threading
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

MIN_POWER_LIMIT_W = 50.0
DRIVER_POWER_FAILURES = ("Failed to handle Power Source change event", "Failed to handle ACPI D-Notifier event")


def _smi(query: str) -> list[str]:
    out = subprocess.run(["nvidia-smi", f"--query-gpu={query}", "--format=csv,noheader,nounits"],
                         capture_output=True, text=True, timeout=5, check=True).stdout
    return [x.strip() for x in out.strip().splitlines()[0].split(",")]


def enforced_power_limit_w() -> Optional[float]:
    try:
        return float(_smi("enforced.power.limit")[0])
    except (OSError, subprocess.SubprocessError, ValueError, IndexError):
        return None


def driver_power_failures() -> Optional[list[str]]:
    """NVRM power-source / D-Notifier failures in this boot's kernel log; None if the kernel journal is not readable
    by this user (that needs the systemd-journal or adm group: a booted system always has kernel messages)."""
    def kernel_log(*extra):
        return subprocess.run(["journalctl", "-k", "-b", "-q", "--no-pager", *extra], capture_output=True, text=True,
                              timeout=10).stdout
    try:
        if not kernel_log("-n", "1").strip():
            return None
        log = kernel_log("--grep", "NVRM")
    except (OSError, subprocess.SubprocessError):
        return None
    return [ln for ln in log.splitlines() if any(p in ln for p in DRIVER_POWER_FAILURES)]


@dataclass(frozen=True)
class Preflight:
    power_limit_w: Optional[float]
    driver_failures: Optional[list] = field(default=None)    # None: journal not readable

    @property
    def ok(self) -> bool:
        """False only for a known-bad budget; no GPU / no nvidia-smi is not this check's business."""
        return self.power_limit_w is None or self.power_limit_w >= MIN_POWER_LIMIT_W

    def message(self) -> str:
        if self.power_limit_w is None:
            return "GPU power limit unknown (nvidia-smi not available)"
        if self.ok:
            return f"GPU enforced power limit {self.power_limit_w:g} W: ok"
        lines = [f"GPU ENFORCED POWER LIMIT IS {self.power_limit_w:g} W (< {MIN_POWER_LIMIT_W:g} W).",
                 "The GPU kept a stale battery-level power budget, typically after a failed resume from sleep.",
                 "SONIC's policy will miss its 20 ms budget with the robot cameras on. Fix: reboot, or replug the "
                 "charger."]
        if self.driver_failures is None:
            lines.append(f"(This boot's kernel journal is not readable by {os.environ.get('USER', 'this user')}; "
                         "NVRM power-event failures could not be checked.)")
        elif self.driver_failures:
            lines.append(f"The kernel journal confirms it: {len(self.driver_failures)} NVRM power-event failure(s) "
                         f"this boot, e.g. {self.driver_failures[0].split('NVRM: ', 1)[-1]}")
        else:
            lines.append("No NVRM power-event failure in this boot's kernel journal.")
        return "\n".join(lines)


def preflight() -> Preflight:
    limit = enforced_power_limit_w()
    failures = driver_power_failures() if limit is not None and limit < MIN_POWER_LIMIT_W else None
    return Preflight(limit, failures)


class ClockSampler(threading.Thread):
    """nvidia-smi once a second: graphics clock (MHz), power draw (W), utilisation (%), clock-event reasons."""

    QUERY = "clocks.gr,power.draw,utilization.gpu,clocks_event_reasons.active"

    def __init__(self, period_s: float = 1.0):
        super().__init__(daemon=True)
        self.period_s = period_s
        self.samples: list[tuple[float, float, float, int]] = []
        self.stop = threading.Event()

    def run(self):
        while not self.stop.is_set():
            try:
                c, p, u, r = _smi(self.QUERY)
                self.samples.append((float(c), float(p), float(u), int(r, 16)))
            except (OSError, subprocess.SubprocessError, ValueError, IndexError):
                pass
            self.stop.wait(self.period_s)

    def summary(self, skip_s: float = 10.0) -> dict:
        """Medians after the first skip_s (boost bursts) plus the extremes; reasons as a hex mask union."""
        s = self.samples[int(skip_s / self.period_s):] or self.samples
        if not s:
            return {}
        a = np.array([x[:3] for x in s])
        reasons = 0
        for x in s:
            reasons |= x[3]
        return {"gpu_clock_mhz_median": float(np.median(a[:, 0])), "gpu_clock_mhz_min": float(a[:, 0].min()),
                "gpu_clock_mhz_max": float(a[:, 0].max()), "gpu_power_w_median": float(np.median(a[:, 1])),
                "gpu_util_pct_median": float(np.median(a[:, 2])), "gpu_clock_event_reasons": hex(reasons),
                "gpu_samples": len(s)}


def warn_at_startup(prefix: str = "[sim]") -> Preflight:
    pf = preflight()
    if pf.ok:
        print(f"{prefix} {pf.message()}", flush=True)
    else:
        bar = "!" * 100
        print(f"{prefix} {bar}", flush=True)
        for line in pf.message().splitlines():
            print(f"{prefix} GPU WARNING: {line}", flush=True)
        print(f"{prefix} {bar}", flush=True)
    return pf

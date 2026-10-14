"""Four-timestamp clock probes; remote monotonic time -> MSI monotonic time."""
from collections import deque
from dataclasses import dataclass, asdict
import math


@dataclass(frozen=True)
class ClockProbe:
    local_send_ns: int
    remote_receive_ns: int
    remote_send_ns: int
    local_receive_ns: int

    @property
    def delay_ns(self):
        return (self.local_receive_ns - self.local_send_ns
                - (self.remote_send_ns - self.remote_receive_ns))

    @property
    def offset_ns(self):
        return ((self.local_send_ns - self.remote_receive_ns)
                + (self.local_receive_ns - self.remote_send_ns)) / 2


class ClockMapper:
    def __init__(self, max_probes=120):
        self.probes = deque(maxlen=max_probes)
        self.session = None

    def add(self, session, probe):
        if probe.delay_ns < 0 or probe.remote_send_ns < probe.remote_receive_ns:
            raise ValueError("invalid four-timestamp probe")
        if session != self.session:
            self.probes.clear()
            self.session = session
        self.probes.append(probe)

    def calibration(self):
        if not self.probes:
            return None
        # Low-delay probes reduce, but do not eliminate, asymmetric network error.
        chosen = sorted(self.probes, key=lambda p: p.delay_ns)[:max(1, len(self.probes)//4)]
        best = chosen[0]
        origin = (best.remote_receive_ns + best.remote_send_ns) / 2
        xs = [(p.remote_receive_ns + p.remote_send_ns)/2 - origin for p in chosen]
        ys = [p.offset_ns for p in chosen]
        mx, my = sum(xs)/len(xs), sum(ys)/len(ys)
        spread = sum((x-mx)**2 for x in xs)
        drift = sum((x-mx)*(y-my) for x,y in zip(xs,ys))/spread if spread else 0.0
        # Do not fit drift until probes span at least 30 seconds.
        if max(xs)-min(xs) < 30_000_000_000:
            drift = 0.0
        if abs(drift) > .001 or not math.isfinite(drift):
            drift = 0.0
        offset = my - drift*mx
        residual = max(abs(y-(offset+drift*x)) for x,y in zip(xs,ys))
        return {"session": self.session, "origin_ns": origin, "offset_ns": offset,
                "drift": drift, "uncertainty_estimate_ns": best.delay_ns/2 + residual,
                "asymmetric_delay_not_bounded": True,
                "probe_count": len(self.probes), "best_round_trip_ns": best.delay_ns,
                "latest_probe": asdict(self.probes[-1])}

    def to_local(self, remote_ns):
        model = self.calibration()
        if model is None:
            raise ValueError("clock is not calibrated")
        return round(remote_ns + model["offset_ns"]
                     + model["drift"]*(remote_ns-model["origin_ns"]))

    def to_remote(self, local_ns):
        model = self.calibration()
        if model is None:
            raise ValueError("clock is not calibrated")
        return round((local_ns-model["offset_ns"]+model["drift"]*model["origin_ns"])
                     /(1+model["drift"]))

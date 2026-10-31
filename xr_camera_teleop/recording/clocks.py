"""PC2 clock check for the recorder on MSI."""
from collections import deque
import statistics
import time


class WallClockGuard:
    """PC2 minus MSI Unix time, measured from the same probes. Camera headers carry
    PC2's clock, so a recorder on MSI can only pair them with state while the two agree."""
    def __init__(self, limit_ms=5, window=8, stale_seconds=5, now=time.monotonic):
        self.limit_ms,self.stale_seconds,self.now = limit_ms,stale_seconds,now
        self.samples = deque(maxlen=window)
        self.episode = None

    def add(self, send_unix_ns, remote_unix_ns, receive_unix_ns):
        delay = receive_unix_ns-send_unix_ns
        if delay<=0:
            return  # MSI's clock stepped during this probe.
        self.samples.append((self.now(),remote_unix_ns-send_unix_ns-delay/2,delay))
        reading = self.reading()
        if self.episode is not None:
            self.episode.append((receive_unix_ns,reading["offset_ms"],reading["uncertainty_ms"]))

    def reading(self):
        if not self.samples:
            return {"ok":False,"offset_ms":None,"uncertainty_ms":None,"age_seconds":None,
                    "limit_ms":self.limit_ms,"error":"no wall clock probes yet"}
        # Low-delay probes bound the error; a short window keeps a clock step visible.
        chosen = sorted(self.samples,key=lambda sample:sample[2])[:max(1,len(self.samples)//2)]
        offset = statistics.median(sample[1] for sample in chosen)/1e6
        uncertainty = chosen[0][2]/2e6
        age = self.now()-self.samples[-1][0]
        error = None
        if age>self.stale_seconds:
            error = "wall clock probes stale: %.1f s" % age
        elif abs(offset)-uncertainty>self.limit_ms:
            # Only a provable offset fails: a slow reply skews one probe by up to its delay.
            error = "PC2 and MSI clocks differ by %.1f ms (limit %s ms)" % (offset,self.limit_ms)
        return {"ok":error is None,"offset_ms":offset,"uncertainty_ms":uncertainty,"age_seconds":age,
                "limit_ms":self.limit_ms,"error":error}

    def begin_episode(self):
        self.episode = []

    def end_episode(self):
        """Offset history is kept so timestamps can be corrected offline, never rewritten."""
        history,self.episode = self.episode or [],None
        worst = max(history,key=lambda item:abs(item[1])-item[2]) if history else None
        return {"ok":bool(history) and abs(worst[1])-worst[2]<=self.limit_ms,"limit_ms":self.limit_ms,"probes":len(history),
                "worst_offset_ms":worst[1] if worst else None,"history_unix_ns_offset_ms_uncertainty_ms":history}

"""In-memory arm backend for offline IK/transport checks. No hardware imports."""
import time
import numpy as np


class DryRunArm:
    def __init__(self, frequency, velocity_limit, clock=time.monotonic):
        self.q = np.zeros(14)
        self.dq = np.zeros(14)
        self.target = self.q.copy()
        self.velocity_limit = velocity_limit
        self.clock = clock
        self.last_update = clock()

    def _advance(self):
        now = self.clock()
        dt = max(0.0, now - self.last_update)
        if dt > 0:
            step = np.clip(self.target - self.q, -self.velocity_limit * dt, self.velocity_limit * dt)
            self.q += step
            self.dq = step / dt
        self.last_update = now

    def get_current_dual_arm_q(self):
        self._advance()
        return self.q.copy()

    def get_current_dual_arm_dq(self):
        self._advance()
        return self.dq.copy()

    def ctrl_dual_arm(self, q, tau):
        self._advance()
        self.target = np.asarray(q, float).copy()

    def ctrl_dual_arm_go_home(self):
        self.q[:] = 0
        self.target[:] = 0
        self.dq[:] = 0

    def stop(self):
        pass

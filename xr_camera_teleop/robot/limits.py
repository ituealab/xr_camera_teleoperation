"""Position command slew limiting with a bounded lead over measured motor position."""
import numpy as np


class JointTargetLimiter:
    def __init__(self, initial_q, max_lead=0.05):
        self.command = np.asarray(initial_q, float).copy()
        self.max_lead = float(max_lead)

    def step(self, target, measured, velocity_limit, dt):
        target, measured = np.asarray(target, float), np.asarray(measured, float)
        if not np.all(np.isfinite(target)) or not np.all(np.isfinite(measured)):
            raise ValueError("joint targets and measurements must be finite")
        command = slew_targets(self.command, target, velocity_limit, dt)
        # Prevent an unresponsive joint from accumulating a large position error/PD torque.
        self.command = np.clip(command, measured - self.max_lead, measured + self.max_lead)
        return self.command.copy()


def slew_targets(previous, target, velocity_limit, dt):
    """Independent joint slew, matching the kinematic preview's per-joint limits."""
    previous, target = np.asarray(previous, float), np.asarray(target, float)
    if not np.isfinite(velocity_limit) or velocity_limit <= 0 or not np.isfinite(dt) or dt < 0:
        raise ValueError("velocity limit must be positive and dt nonnegative")
    if not np.all(np.isfinite(previous)) or not np.all(np.isfinite(target)):
        raise ValueError("joint targets must be finite")
    return previous + np.clip(target - previous, -velocity_limit * dt, velocity_limit * dt)

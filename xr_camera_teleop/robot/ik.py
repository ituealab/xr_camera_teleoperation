"""Compatibility imports: canonical solver is installed g1-kinematics."""

from g1_kinematics.ik import *  # noqa: F403


def connect_ik(ik, wrapper, clock=None):
    """Keep teleoperation callback wiring local; all kinematics are shared."""
    if hasattr(ik, "reset_to_home"):
        wrapper.on_home_request = ik.reset_to_home
        if clock is not None:
            ik.clock = clock

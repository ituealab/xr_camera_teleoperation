"""Compatibility module alias; fitted geometry has one source."""

import sys
from g1_kinematics import g1_collision_spheres as _canonical

sys.modules[__name__] = _canonical

"""Compatibility module alias; no duplicate collision implementation."""

import sys
from g1_kinematics import self_collision as _canonical

sys.modules[__name__] = _canonical

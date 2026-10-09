# Vendored from unitreerobotics/xr_teleoperate @ 817fb00 (Apache-2.0, see LICENSE-THIRD-PARTY).
# Source file: teleop/utils/motion_switcher.py
# MODIFIED: only MotionSwitcher is kept (LocoClientWrapper is for controller input only); unitree_sdk2py imports
# are done lazily so that importing this module needs no DDS; demo main removed.
import time


class MotionSwitcher:
    """Switch the robot between its motion service ("ai" mode) and debug mode (rt/lowcmd)."""

    def __init__(self):
        from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
        self.msc = MotionSwitcherClient()
        self.msc.SetTimeout(1.0)
        self.msc.Init()

    def Enter_Debug_Mode(self):
        try:
            status, result = self.msc.CheckMode()
            while result['name']:
                self.msc.ReleaseMode()
                status, result = self.msc.CheckMode()
                time.sleep(1)
            return status, result
        except Exception as e:
            return None, None

    def Exit_Debug_Mode(self):
        try:
            status, result = self.msc.SelectMode(nameOrAlias='ai')
            return status, result
        except Exception as e:
            return None, None
